"""Recorder lifecycle tests: no ROS initialization or real subprocesses."""
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from amr_elevator.elevator_recorder import (
    DEFAULT_TOPICS, RecorderCore, bag_folder, die_with_parent, starts_mission)


def status(phase='WAIT_DOOR_OPEN', **extra):
    return dict(phase=phase, start_floor='4F', goal_floor='3F', detail='4F -> 3F', **extra)


@pytest.fixture
def recorder(tmp_path):
    clock = [0.]
    process = Mock(pid=12345)
    process.poll.return_value = None
    process.wait.return_value = 0
    launcher, signals, warn = Mock(return_value=process), Mock(), Mock()
    tuning = tmp_path / 'tuning.yaml'
    tuning.write_text('floor_mission_manager:\n  ros__parameters:\n    maneuver_speed: 0.35\n')
    core = RecorderCore(tmp_path, launcher=launcher, clock=lambda: clock[0],
                        wall_clock=lambda: datetime(2026, 11, 2, 16, 2, 3, tzinfo=timezone.utc),
                        disk_free=lambda _: 10 * 1024**3, tuning_file=tuning,
                        identity=lambda _: dict(commit='abc123', branch='vision-floor-no-rf', dirty=True),
                        send_signal=signals, warn=warn)
    return SimpleNamespace(core=core, clock=clock, process=process, launcher=launcher,
                           signals=signals, warn=warn, tuning=tuning)


def test_idle_active_edge_launches_one_process_with_manifest(recorder):
    r = recorder
    r.core.on_status(status('IDLE'))
    r.launcher.assert_not_called()
    r.core.on_status(status())
    r.core.on_status(status('BOARDING'))
    r.core.on_status(status('BOARDING'))
    r.launcher.assert_called_once()
    args, kwargs = r.launcher.call_args
    assert args[0][:9] == ['nice', '-n', '10', 'ros2', 'bag', 'record', '-s', 'mcap', '-o']
    assert args[0][10:] == DEFAULT_TOPICS
    assert kwargs['start_new_session'] is True
    assert kwargs['stdin'] == subprocess.DEVNULL
    assert kwargs['preexec_fn'] is die_with_parent
    manifest = json.loads(r.core.manifest_path.read_text())
    assert manifest['git'] == dict(commit='abc123', branch='vision-floor-no-rf', dirty=True)
    assert manifest['mission'] == status()
    assert manifest['tuning_yaml'] == r.tuning.read_text()
    assert manifest['started_at'] == '2026-11-03T01:02:03+09:00'
    assert manifest['bag_dir'].endswith('2026-11-03/010203_elevator_4F_3F')


def test_manifest_moves_beside_mcap_after_recorder_creates_directory(recorder):
    r = recorder
    r.core.on_status(status())
    staging = r.core.manifest_path
    bag = Path(r.core.manifest['bag_dir'])
    assert not bag.exists()  # rosbag must be the one creating this directory
    bag.mkdir()  # imitate the fake recorder reaching storage initialization
    r.core.tick()
    assert r.core.manifest_path == bag / 'manifest.json'
    assert json.loads(r.core.manifest_path.read_text())['mission'] == status()
    assert not staging.exists()


@pytest.mark.parametrize('terminal', ['SUCCEEDED', 'FAILED_SAFE', 'CANCELED', 'IDLE'])
def test_terminal_postroll_escalation_and_final_manifest(recorder, terminal):
    r = recorder
    r.core.on_status(status())
    r.clock[0] = 10.
    r.core.on_status(status(terminal, error_code='TEST_ERROR'))
    r.clock[0] = 14.9
    r.core.on_status(status(terminal, error_code='TEST_ERROR'))
    r.core.tick()
    r.signals.assert_not_called()
    for now, sig in ((15., signal.SIGINT), (20., signal.SIGTERM), (25., signal.SIGKILL)):
        r.clock[0] = now
        r.core.tick()
        r.signals.assert_called_with(r.process, sig)
    assert r.signals.call_count == 3
    r.process.poll.return_value = -9
    r.clock[0] = 25.1
    r.core.tick()
    assert r.core.process is None
    manifest = json.loads(r.core.manifest_path.read_text())
    assert manifest['final_phase'] == terminal
    assert manifest['error_code'] == 'TEST_ERROR'
    assert manifest['duration_s'] == 25.1
    assert manifest['exit_code'] == -9


def test_max_duration_never_restarts_the_same_mission(recorder):
    r = recorder
    r.core.on_status(status())
    r.clock[0] = 900.
    r.core.tick()
    r.signals.assert_called_once_with(r.process, signal.SIGINT)
    r.process.poll.return_value = 0
    r.core.tick()
    r.core.on_status(status('BOARDING'))
    r.core.tick()
    r.launcher.assert_called_once()
    assert r.core.manifest['stop_reason'] == 'max_duration'


def test_disk_space_skip_warns_once_and_retries_next_mission(recorder):
    r = recorder
    r.core.disk_free = lambda _: 5 * 1024**3 - 1
    for _ in range(2):
        r.core.on_status(status())
        r.core.on_status(status('IDLE'))
    r.launcher.assert_not_called()
    r.warn.assert_called_once()
    r.core.disk_free = lambda _: 5 * 1024**3
    r.core.on_status(status())
    r.launcher.assert_called_once()


def test_disabled_and_pure_path_rules(recorder, tmp_path):
    recorder.core.enabled = False
    recorder.core.on_status(status())
    recorder.launcher.assert_not_called()
    path = bag_folder(tmp_path, datetime(2026, 1, 1, 16, tzinfo=timezone.utc), '../4F', '3/F')
    assert path.relative_to(tmp_path).as_posix() == 'bags/2026-01-02/010000_elevator____4F_3_F'
    assert starts_mission('CANCELED', 'NAV_TO_HALL')
    assert not starts_mission('BOARDING', 'IN_CAR_ALIGN')
    assert not starts_mission('IDLE', 'unknown')


def test_new_mission_during_postroll_reuses_bag(recorder):
    r = recorder
    r.core.on_status(status())
    r.core.on_status(status('SUCCEEDED'))
    r.clock[0] = 1.
    r.core.on_status(status('NAV_TO_HALL'))
    r.clock[0] = 6.
    r.core.tick()
    r.signals.assert_not_called()
    r.launcher.assert_called_once()
    assert len(r.core.manifest['missions']) == 2
    r.core.on_status(status('SUCCEEDED'))
    r.clock[0] = 11.
    r.core.tick()
    r.signals.assert_called_once_with(r.process, signal.SIGINT)


def test_new_mission_while_stopping_is_queued_and_folder_is_unique(recorder):
    r = recorder
    r.core.on_status(status())
    first_path = r.core.manifest_path
    r.core.on_status(status('SUCCEEDED'))
    r.clock[0] = 5.
    r.core.tick()
    r.core.on_status(status('NAV_TO_HALL'))
    r.launcher.assert_called_once()
    r.core.on_status(status('WAIT_DOOR_OPEN'))
    r.process.poll.return_value = 0
    r.core.tick()
    assert r.launcher.call_count == 2
    assert r.core.manifest_path != first_path
    assert r.core.manifest_path.name.endswith('_2.manifest.json')
    assert json.loads(first_path.read_text())['final_phase'] == 'SUCCEEDED'
    assert r.core.manifest['mission']['phase'] == 'WAIT_DOOR_OPEN'


def test_exit_before_timer_does_not_swallow_next_mission(recorder):
    r = recorder
    r.core.on_status(status())
    r.core.on_status(status('SUCCEEDED'))
    r.process.poll.return_value = 1
    r.core.on_status(status('NAV_TO_HALL'))
    assert r.launcher.call_count == 2
    r.warn.assert_called_once()


def test_queued_mission_that_ends_before_exit_is_not_launched(recorder):
    r = recorder
    r.core.on_status(status())
    r.core.on_status(status('SUCCEEDED'))
    r.clock[0] = 5.
    r.core.tick()
    r.core.on_status(status('NAV_TO_HALL'))
    r.core.on_status(status('CANCELED'))
    r.core.on_status(status('IDLE'))
    r.process.poll.return_value = 0
    r.core.tick()
    r.launcher.assert_called_once()
    assert r.core.manifest['final_phase'] == 'SUCCEEDED'


def test_shutdown_waits_and_escalates(recorder):
    r = recorder
    r.core.on_status(status())
    r.process.wait.side_effect = [subprocess.TimeoutExpired('fake', 5),
                                 subprocess.TimeoutExpired('fake', 5), -9]
    r.core.shutdown()
    assert [call.args[1] for call in r.signals.call_args_list] == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]
    assert r.core.process is None
    assert r.core.manifest['stop_reason'] == 'shutdown'


def test_launch_failure_is_reported_without_repeated_spawning(recorder):
    r = recorder
    r.launcher.side_effect = FileNotFoundError('fake missing ros2')
    r.core.on_status(status())
    r.core.on_status(status('BOARDING'))
    r.core.tick()
    assert r.core.process is None
    r.launcher.assert_called_once()
    assert 'fake missing ros2' in json.loads(r.core.manifest_path.read_text())['recorder_error']


@pytest.mark.parametrize('kwargs', [{'post_roll_s': -1}, {'max_duration_s': 0}, {'min_free_gb': float('nan')}])
def test_invalid_settings_are_rejected(tmp_path, kwargs):
    with pytest.raises(ValueError):
        RecorderCore(tmp_path, **kwargs)


def test_low_disk_during_recording_stops_it(recorder):
    r = recorder
    free = [10 * 1024**3]
    r.core.disk_free = lambda _: free[0]
    r.core.on_status(status())
    r.core.tick()
    r.signals.assert_not_called()
    free[0] = 2 * 1024**3  # below half of min_free_gb (5 GB)
    r.clock[0] = 6.
    r.core.tick()
    r.signals.assert_called_once_with(r.process, signal.SIGINT)
    assert json.loads(r.core.manifest_path.read_text())['stop_reason'] == 'disk_low'
