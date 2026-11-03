"""Offline synthetic samples; no rosbag or ROS runtime required."""
import importlib.util
import json
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    'analyze_elevator_run', Path(__file__).resolve().parents[4] / 'scripts/analyze_elevator_run.py')
analyzer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyzer)


def odom(v, w=0.):
    return {'twist': {'twist': {'linear': {'x': v, 'y': 0.}, 'angular': {'z': w}}}}


def test_timeline_repeated_status_and_unknown_final_duration():
    rows = [(0, {'data': json.dumps({'phase': 'WAIT_DOOR_OPEN'})}),
            (1, {'phase': 'WAIT_DOOR_OPEN'}), (3, {'phase': 'BOARDING'}), (8, {'phase': 'IN_CAR_ALIGN'})]
    timeline = analyzer.phase_timeline(rows, end_time=10)
    assert [p['duration_s'] for p in timeline] == [3, 5, 2]
    assert analyzer.phase_timeline(rows)[-1]['duration_s'] is None


def test_boarding_alighting_turns_and_missing_milestones():
    topics = {
        analyzer.STATUS: [(0, {'phase': 'WAIT_DOOR_OPEN'}), (3, {'phase': 'BOARDING'}),
                          (8, {'phase': 'IN_CAR_ALIGN'}), (10, {'phase': 'RIDING'}),
                          (19, {'phase': 'SWITCH_MAP'}), (20, {'phase': 'ALIGHTING'}),
                          (25, {'phase': 'RELOCALIZE'})],
        analyzer.DOOR: [(2, {'state': 2}), (2.5, {'state': 3}), (10, {'state': 3}),
                        (11, {'state': 1}), (14, {'state': 3}), (16, {'state': 1}), (18, {'state': 2})],
        analyzer.ODOM: [(3, odom(0)), (4, odom(.1)), (8, odom(0, .4)), (9, odom(0, -.5)),
                        (20, odom(0)), (21, odom(.2, .1)), (25, odom(0))],
        analyzer.FLOOR: [(0, {'zone': 1}), (7, {'zone': 2}), (24, {'zone': 1})],
    }
    report = analyzer.analyze(topics, end_time=26)
    b, a = report['boarding'][0], report['alighting'][0]
    assert (b['door_opening_or_open'], b['start'], b['first_motion'], b['zone_crossing']) == (2, 3, 4, 7)
    assert (b['align_start'], b['align_end'], b['align_duration_s']) == (8, 10, 2)
    assert b['door_to_start_s'] == 1 and b['motion_to_crossing_s'] == 3
    assert (a['door_opening_or_open'], a['start'], a['first_motion'], a['zone_crossing']) == (18, 20, 21, 24)
    assert report['in_car_turns'][0]['peak_angular_speed'] == .5
    topics[analyzer.FLOOR] = []
    assert analyzer.analyze(topics)['boarding'][0]['zone_crossing'] is None
    assert 'rear-bumper' in analyzer.markdown(report)
    json.dumps(report, allow_nan=False)


def test_hold_sums_clip_at_phase_end_and_count_stop_restart_only_in_drive():
    statuses = [(0, {'phase': 'BOARDING'}), (1, {'phase': 'IN_CAR_ALIGN'})]
    debug = [(0, {'phase': 'BOARDING', 'maneuver_phase': 'DRIVE', 'hold': 'obstacle'}),
             (.1, {'phase': 'BOARDING', 'maneuver_phase': 'DRIVE', 'hold': 'obstacle'}),
             (.2, {'phase': 'BOARDING', 'maneuver_phase': 'DRIVE', 'hold': None}),
             (.9, {'phase': 'BOARDING', 'maneuver_phase': 'DRIVE', 'hold': 'scan_stale'}),
             (1.5, {'phase': 'IN_CAR_ALIGN', 'maneuver_phase': 'TURN', 'hold': 'sweep_blocked'})]
    odometry = [(0, odom(.35)), (.1, odom(0)), (.2, odom(.35)), (.3, odom(0)), (.4, odom(.35)),
                (.9, odom(0)), (1.5, odom(.35))]
    report = analyzer.analyze({analyzer.STATUS: statuses, analyzer.DEBUG: debug, analyzer.ODOM: odometry},
                              end_time=1.6)
    assert report['hold_seconds']['BOARDING'] == pytest.approx({'obstacle': .2, 'scan_stale': .1})
    assert report['hold_seconds']['IN_CAR_ALIGN']['sweep_blocked'] == pytest.approx(.1)
    assert report['drive']['stop_start_cycles'] == 2
    assert report['drive']['stop_start_cycles_by_phase'] == {'BOARDING': 2}
    assert report['drive']['mean_speed'] == pytest.approx(.245)
    assert report['drive']['max_speed'] == .35
    assert report['drive']['time_at_maneuver_speed_s'] == pytest.approx(.7)


def test_retry_does_not_borrow_success_milestone_or_count_initial_start():
    statuses = [(0, {'phase': 'WAIT_DOOR_OPEN'}), (1, {'phase': 'BOARDING'}),
                (2, {'phase': 'BACKING_OUT'}), (3, {'phase': 'WAIT_DOOR_OPEN'}),
                (4, {'phase': 'BOARDING'}), (6, {'phase': 'IN_CAR_ALIGN'})]
    report = analyzer.analyze({
        analyzer.STATUS: statuses, analyzer.DOOR: [(.5, {'state': 'OPEN'}), (3.5, {'state': 'OPENING'})],
        analyzer.FLOOR: [(0, {'zone': 1}), (5, {'zone': 2})],
        analyzer.DEBUG: [(1, {'phase': 'BOARDING', 'maneuver_phase': 'DRIVE'}),
                         (2, {'phase': 'BACKING_OUT', 'maneuver_phase': 'ALIGN'}),
                         (4, {'phase': 'BOARDING', 'maneuver_phase': 'DRIVE'})],
        analyzer.ODOM: [(1, odom(0)), (1.5, odom(.2)), (4, odom(0)), (4.1, odom(.2))],
    }, end_time=7)
    assert len(report['boarding']) == 2
    assert report['boarding'][0]['zone_crossing'] is None
    assert report['boarding'][1]['door_opening_or_open'] == 3.5
    assert report['drive']['stop_start_cycles'] == 0


def test_missing_data_and_recorded_tuning(tmp_path):
    report = analyzer.analyze({})
    assert report['drive']['mean_speed'] is None
    assert len(report['missing_topics']) == 5
    bag = tmp_path / 'trial'
    manifest = bag.with_name('trial.manifest.json')
    manifest.write_text(json.dumps({'tuning_yaml': 'floor_mission_manager:\n  ros__parameters:\n    maneuver_speed: 0.4\n'}))
    assert analyzer.recorded_speed(bag) == .4
    assert analyzer.samples([(0, {'data': 'invalid JSON'})]) == []
