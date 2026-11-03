#!/usr/bin/env python3
"""Analyze MCAP elevator recordings offline; analysis functions accept (seconds, dict).

No ROS imports occur until read_bag(). Receive timestamps provide one common
clock across topics. Missing milestones are null, never invented. Zone edges
are explicitly a proxy, not a measurement of rear-bumper clearance.
"""

import argparse
from bisect import bisect_right
from collections import defaultdict
import json
import math
from pathlib import Path
from statistics import median

STATUS = '/floor_mission/status'
DEBUG = '/floor_mission/debug'
ODOM = '/odometry/filtered'
DOOR = '/elevator/door'
FLOOR = '/floor/state'
ANALYSIS_TOPICS = (STATUS, DEBUG, ODOM, DOOR, FLOOR)


def payload(message):
    """Accept both deserialized String dictionaries and already parsed payloads."""
    if isinstance(message, str):
        return json.loads(message)
    if isinstance(message, dict) and isinstance(message.get('data'), str):
        return json.loads(message['data'])
    return message


def samples(rows):
    result = []
    for t, message in rows:
        try:
            data = payload(message)
            if isinstance(data, dict) and math.isfinite(t):
                result.append((float(t), data))
        except (ValueError, TypeError):
            continue
    return sorted(result, key=lambda row: row[0])


def phase_timeline(statuses, end_time=None):
    timeline = []
    for t, message in samples(statuses):
        phase = message.get('phase')
        if not phase or (timeline and timeline[-1]['phase'] == phase):
            continue
        if timeline:
            timeline[-1].update(end=t, duration_s=t - timeline[-1]['start'])
        timeline.append(dict(phase=phase, start=t, end=None, duration_s=None))
    if timeline and end_time is not None:
        end = max(end_time, timeline[-1]['start'])
        timeline[-1].update(end=end, duration_s=end - timeline[-1]['start'])
    return timeline


def odometry_speed(message):
    twist = message.get('twist', {})
    twist = twist.get('twist', twist)
    linear, angular = twist.get('linear', {}), twist.get('angular', {})
    return math.hypot(linear.get('x', 0.), linear.get('y', 0.)), abs(angular.get('z', 0.))


def crossing_events(floors, target):
    result, previous = [], None
    for t, message in samples(floors):
        zone = message.get('zone')
        if zone == target and previous is not None and previous != target:
            result.append(t)
        previous = zone
    return result


def _first(times, start, end):
    return next((t for t in times if start <= t < end), None)


def _delta(end, start):
    return None if start is None or end is None else end - start


def crossing_breakdowns(timeline, doors, odometry, floors, phase='BOARDING'):
    """One record per attempt; retries cannot consume another attempt's milestones."""
    door_rows = samples(doors)
    door_times = [t for t, m in door_rows if m.get('state') in (2, 3, 'OPENING', 'OPEN')]
    moving = [t for t, m in samples(odometry) if odometry_speed(m)[0] > .03]
    zone = 2 if phase == 'BOARDING' else 1
    crossings = crossing_events(floors, zone)
    results = []
    for i, period in enumerate(timeline):
        if period['phase'] != phase or period['end'] is None:
            continue
        start, end = period['start'], period['end']
        # Restrict the door observation to this approach/ride, not the first
        # opening anywhere in the bag (which would corrupt alighting/retries).
        lower = start
        preceding = (('NAV_TO_HALL', 'WAIT_DOOR_OPEN', 'CALL_ELEVATOR') if phase == 'BOARDING'
                     else ('RIDING', 'SWITCH_MAP', 'RETURN_TO_CAR'))
        for previous in reversed(timeline[:i]):
            if previous['phase'] not in preceding:
                break
            lower = previous['start']
        if phase == 'ALIGHTING':
            # RIDING begins with the departure door still open and may include
            # other passenger stops. Use the opening after the latest closure.
            lower = max((t for t, m in door_rows if lower <= t < start
                         and m.get('state') in (1, 4, 'CLOSED', 'CLOSING')), default=lower)
        door = _first(door_times, lower, end)
        motion = _first(moving, start, end)
        crossing = _first(crossings, start, end)
        alignment = (timeline[i + 1] if i + 1 < len(timeline)
                     and timeline[i + 1]['phase'] == 'IN_CAR_ALIGN' else None)
        result = dict(start=start, end=end, duration_s=end - start,
                      door_opening_or_open=door, first_motion=motion, zone_crossing=crossing,
                      crossing_basis='floor/state zone edge (rear clearance proxy)',
                      door_to_start_s=_delta(start, door), start_to_motion_s=_delta(motion, start),
                      motion_to_crossing_s=_delta(crossing, motion),
                      crossing_to_end_s=_delta(end, crossing),
                      peak_angular_speed=max((odometry_speed(m)[1] for t, m in samples(odometry)
                                              if start <= t < end), default=None))
        if phase == 'BOARDING':
            result.update(align_start=None if alignment is None else alignment['start'],
                          align_end=None if alignment is None else alignment['end'])
            result['crossing_to_align_s'] = _delta(result['align_start'], crossing)
            result['align_duration_s'] = _delta(result['align_end'], result['align_start'])
        results.append(result)
    return results


def debug_intervals(debug, timeline, end_time):
    """Hold each observation until the next tick, clipped at a status phase edge.

    The last sample gets one median observed tick (or 0.1 s), bounded by the
    recording end. A telemetry gap is represented by its observed interval.
    """
    rows = samples(debug)
    intervals = [b[0] - a[0] for a, b in zip(rows, rows[1:]) if 0 < b[0] - a[0] <= 1.]
    last_dt = median(intervals) if intervals else .1
    for i, (t, message) in enumerate(rows):
        end = min(rows[i + 1][0] if i + 1 < len(rows) else t + last_dt, end_time)
        # Never carry an active sample through a terminal/post-roll gap.
        period = next((p for p in timeline if p['start'] <= t < p['end']), None)
        if period is not None:
            if period['phase'] in ('IDLE', 'SUCCEEDED', 'FAILED_SAFE', 'CANCELED'):
                continue
            end = min(end, period['end'])
        if end > t:
            yield t, end, message


def hold_times(debug, timeline, end_time):
    totals = defaultdict(lambda: defaultdict(float))
    for start, end, message in debug_intervals(debug, timeline, end_time):
        if message.get('hold'):
            totals[message.get('phase', 'UNKNOWN')][message['hold']] += end - start
    return {phase: dict(reasons) for phase, reasons in totals.items()}


def drive_statistics(debug, odometry, timeline, end_time, maneuver_speed=.35):
    """Time-weighted odometry in contiguous DRIVE windows; count moving-stop-moving."""
    windows = []
    for start, end, message in debug_intervals(debug, timeline, end_time):
        if message.get('maneuver_phase') != 'DRIVE':
            continue
        phase = message.get('phase')
        if windows and windows[-1][1] == start and windows[-1][2] == phase:
            windows[-1] = (windows[-1][0], end, phase)
        else:
            windows.append((start, end, phase))
    odom = samples(odometry)
    times = [t for t, _ in odom]
    duration = integral = at_speed = 0.
    maximum = None
    cycles, by_phase = 0, defaultdict(int)
    for start, end, phase in windows:
        moving_before = stopped = False
        # Include the last known speed at the DRIVE boundary.
        index = max(0, bisect_right(times, start) - 1)
        for j in range(index, len(odom)):
            t, message = odom[j]
            if t >= end:
                break
            speed, _ = odometry_speed(message)
            a, b = max(t, start), min(times[j + 1] if j + 1 < len(times) else end, end)
            if b <= a:
                continue
            if speed > .03:
                if moving_before and stopped:
                    cycles += 1
                    by_phase[phase] += 1
                moving_before, stopped = True, False
            elif speed < .03 and moving_before:
                stopped = True
            dt = b - a
            duration += dt
            integral += speed * dt
            maximum = speed if maximum is None else max(maximum, speed)
            if speed >= .95 * maneuver_speed:
                at_speed += dt
    return dict(mean_speed=integral / duration if duration else None, max_speed=maximum,
                observed_duration_s=duration, time_at_maneuver_speed_s=at_speed,
                maneuver_speed=maneuver_speed, at_speed_threshold=.95 * maneuver_speed,
                stop_start_cycles=cycles, stop_start_cycles_by_phase=dict(by_phase))


def analyze(topics, maneuver_speed=.35, end_time=None):
    data = {topic: samples(topics.get(topic, [])) for topic in ANALYSIS_TOPICS}
    all_times = [t for rows in data.values() for t, _ in rows]
    end = end_time if end_time is not None else max(all_times, default=0.)
    timeline = phase_timeline(data[STATUS], end)
    turns = []
    for period in timeline:
        if period['phase'] == 'IN_CAR_ALIGN':
            speeds = [odometry_speed(m)[1] for t, m in data[ODOM] if period['start'] <= t < period['end']]
            turns.append({**period, 'peak_angular_speed': max(speeds, default=None)})
    return dict(timeline=timeline,
                boarding=crossing_breakdowns(timeline, data[DOOR], data[ODOM], data[FLOOR]),
                alighting=crossing_breakdowns(timeline, data[DOOR], data[ODOM], data[FLOOR], 'ALIGHTING'),
                in_car_turns=turns, hold_seconds=hold_times(data[DEBUG], timeline, end),
                drive=drive_statistics(data[DEBUG], data[ODOM], timeline, end, maneuver_speed),
                missing_topics=[topic for topic, rows in data.items() if not rows])


def markdown(report):
    def number(value):
        return '—' if value is None else f'{value:.3f}'
    lines = ['# Elevator run analysis', '',
             'Times use bag receive timestamps (seconds). Missing observations are shown as —.',
             'Door-line crossing uses a floor zone edge; it does not measure rear-bumper clearance.',
             '', '## Phase timeline', '', '| Phase | Start | End | Duration (s) |', '|---|---:|---:|---:|']
    for p in report['timeline']:
        lines.append(f"| {p['phase']} | {number(p['start'])} | {number(p['end'])} | {number(p['duration_s'])} |")
    for key in ('boarding', 'alighting'):
        lines += ['', f'## {key.title()} attempts', '',
                  '| Attempt | Door OPENING/OPEN | Start | First motion | Zone crossing | End | Peak angular (rad/s) |',
                  '|---|---:|---:|---:|---:|---:|---:|']
        for i, attempt in enumerate(report[key], 1):
            fields = ('door_opening_or_open', 'start', 'first_motion', 'zone_crossing', 'end', 'peak_angular_speed')
            lines.append(f'| {i} | ' + ' | '.join(number(attempt[k]) for k in fields) + ' |')
        lines += ['', '| Attempt | Door → start (s) | Start → motion (s) | Motion → crossing (s) | Crossing → end (s) |',
                  '|---|---:|---:|---:|---:|']
        for i, attempt in enumerate(report[key], 1):
            fields = ('door_to_start_s', 'start_to_motion_s', 'motion_to_crossing_s', 'crossing_to_end_s')
            lines.append(f'| {i} | ' + ' | '.join(number(attempt[k]) for k in fields) + ' |')
        if key == 'boarding':
            lines += ['', '| Attempt | IN_CAR_ALIGN start | IN_CAR_ALIGN end | Duration (s) |',
                      '|---|---:|---:|---:|']
            for i, attempt in enumerate(report[key], 1):
                fields = ('align_start', 'align_end', 'align_duration_s')
                lines.append(f'| {i} | ' + ' | '.join(number(attempt[k]) for k in fields) + ' |')
    lines += ['', '## In-car turns', '']
    for turn in report['in_car_turns']:
        lines.append(f"- Start {number(turn['start'])}: {number(turn['duration_s'])} s; "
                     f"peak angular speed {number(turn['peak_angular_speed'])} rad/s.")
    lines += ['', '## Hold time', '', '| Phase | Reason | Seconds |', '|---|---|---:|']
    for phase, reasons in report['hold_seconds'].items():
        for reason, duration in sorted(reasons.items()):
            lines.append(f'| {phase} | {reason} | {number(duration)} |')
    d = report['drive']
    lines += ['', '## DRIVE speed and hesitation', '',
              f"- Mean / max measured speed: {number(d['mean_speed'])} / {number(d['max_speed'])} m/s.",
              f"- Time at ≥95% of maneuver_speed ({number(d['maneuver_speed'])} m/s): "
              f"{number(d['time_at_maneuver_speed_s'])} s (observed DRIVE: {number(d['observed_duration_s'])} s).",
              f"- Stop/start cycles (<0.03 then >0.03 m/s after moving): {d['stop_start_cycles']}.",
              f"- Cycles by phase: {json.dumps(d['stop_start_cycles_by_phase'], sort_keys=True)}."]
    if report['missing_topics']:
        lines += ['', 'Missing topics: ' + ', '.join(report['missing_topics'])]
    return '\n'.join(lines) + '\n'


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {info.name: info.type for info in reader.get_all_topics_and_types()}
    selected = [name for name in ANALYSIS_TOPICS if name in types]
    data = {name: [] for name in ANALYSIS_TOPICS}
    if not selected:
        return data
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    classes = {name: get_message(types[name]) for name in selected}
    while reader.has_next():
        name, serialized, timestamp = reader.read_next()
        message = deserialize_message(serialized, classes[name])
        data[name].append((timestamp * 1e-9, message_to_ordereddict(message)))
    return data


def recorded_speed(path):
    candidates = (path.with_name(path.name + '.manifest.json'), path / 'manifest.json',
                  path.parent / 'manifest.json')
    for manifest_path in candidates:
        if manifest_path.is_file():
            import yaml
            manifest = json.loads(manifest_path.read_text())
            config = yaml.safe_load(manifest.get('tuning_yaml') or '') or {}
            speed = config.get('floor_mission_manager', {}).get('ros__parameters', {}).get('maneuver_speed', .35)
            if not math.isfinite(speed) or speed <= 0:
                raise ValueError('Invalid maneuver_speed in manifest')
            return speed
    return .35


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_dir', type=Path)
    parser.add_argument('--out', type=Path, help='Markdown path; companion JSON uses the same stem')
    args = parser.parse_args(argv)
    report = analyze(read_bag(args.bag_dir), maneuver_speed=recorded_speed(args.bag_dir))
    text = markdown(report)
    json_path = args.out.with_suffix('.json') if args.out else args.bag_dir / 'elevator_report.json'
    if args.out:
        args.out.write_text(text)
    else:
        print(text, end='')
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
