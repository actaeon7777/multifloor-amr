"""Automatic real elevator bags. RecorderCore has no ROS dependency.

The MCAP output is bags/YYYY-MM-DD/HHMMSS_elevator_START_GOAL, with manifest.json
beside the MCAP files. A sibling <bag-name>.manifest.json stages metadata until
rosbag creates its output directory (which must not already exist).
"""

import ctypes
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time
from zoneinfo import ZoneInfo

TERMINAL = frozenset(('IDLE', 'SUCCEEDED', 'FAILED_SAFE', 'CANCELED'))
ACTIVE = frozenset(('NAV_TO_HALL', 'CALL_ELEVATOR', 'WAIT_DOOR_OPEN', 'BOARDING',
                    'BACKING_OUT', 'IN_CAR_ALIGN', 'SELECT_FLOOR', 'RIDING', 'SWITCH_MAP',
                    'ALIGHTING', 'RETURN_TO_CAR', 'RELOCALIZE', 'NAV_TO_GOAL'))
DEFAULT_TOPICS = [
    '/scan', '/odometry/filtered', '/tf', '/tf_static', '/elevator/door', '/floor/state',
    '/floor/events', '/floor/pose_estimate', '/floor_mission/status', '/floor_mission/debug',
    '/elevator/request', '/floor/arrival_confirm', '/cmd_vel_maneuver', '/cmd_vel_auto',
    '/navigation/status', '/robot/announcement', '/estop/status', '/rosout',
    '/camera/depth/points_filtered', '/camera/color/image_raw/compressed',
]
KST = ZoneInfo('Asia/Seoul')


def starts_mission(previous, current):
    return previous in TERMINAL and current in ACTIVE


def bag_folder(workspace, instant, start, goal):
    """Pure KST path builder; untrusted floor names cannot escape the workspace."""
    def safe(value):
        return re.sub(r'[^A-Za-z0-9_-]', '_', str(value)) or 'unknown'
    stamp = instant.astimezone(KST)
    return (Path(workspace) / 'bags' / stamp.strftime('%Y-%m-%d')
            / f'{stamp:%H%M%S}_elevator_{safe(start)}_{safe(goal)}')


def git_identity(workspace):
    result = {'commit': None, 'branch': None, 'dirty': None}
    deadline = time.monotonic() + 2.
    for name, args in (('commit', ['rev-parse', 'HEAD']),
                       ('branch', ['rev-parse', '--abbrev-ref', 'HEAD']),
                       ('dirty', ['status', '--porcelain'])):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        try:
            value = subprocess.run(['git', '-C', str(workspace), *args],
                                   capture_output=True, text=True, check=True,
                                   timeout=remaining).stdout.strip()
            result[name] = bool(value) if name == 'dirty' else value
        except (OSError, subprocess.SubprocessError):
            pass
    return result


def signal_group(process, sig):
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        pass


def die_with_parent():
    """Child pre-exec hook: SIGINT the bag recorder if this node dies, even by SIGKILL."""
    try:
        ctypes.CDLL('libc.so.6', use_errno=True).prctl(1, signal.SIGINT)  # PR_SET_PDEATHSIG
    except OSError:
        pass


class RecorderCore:
    """Monotonic lifecycle with injected process, clock, disk and metadata providers.

    A new mission during post-roll shares the bag, with both mission identities
    retained. A new mission during SIGINT finalization is queued until exit.
    A max-duration stop cannot restart until a subsequent mission edge.
    """

    def __init__(self, workspace, *, enabled=True, post_roll_s=5., max_duration_s=900.,
                 min_free_gb=5., topics=None, tuning_file='', launcher=subprocess.Popen,
                 clock=time.monotonic, wall_clock=lambda: datetime.now(KST),
                 disk_free=lambda path: shutil.disk_usage(path).free,
                 identity=git_identity, send_signal=signal_group, warn=lambda text: None):
        for name, value in (('post_roll_s', post_roll_s), ('min_free_gb', min_free_gb),
                            ('max_duration_s', max_duration_s)):
            if not math.isfinite(value) or value < 0 or (name == 'max_duration_s' and value == 0):
                raise ValueError(f'{name} must be finite and nonnegative (duration must be positive)')
        self.workspace, self.enabled = Path(workspace), enabled
        self.post_roll_s, self.max_duration_s, self.min_free_gb = post_roll_s, max_duration_s, min_free_gb
        self.disk_checked = -math.inf
        self.topics = list(DEFAULT_TOPICS if topics is None else topics)
        self.tuning_file = str(tuning_file)
        self.launcher, self.clock, self.wall_clock = launcher, clock, wall_clock
        self.disk_free, self.identity, self.send_signal, self.warn = disk_free, identity, send_signal, warn
        self.phase = 'IDLE'
        self.process = None
        self.manifest = None
        self.pending = None
        self.queued_mission_seen = False
        self.disk_warned = False
        self.stop_at = None
        self.stopping_since = None
        self.signal_stage = 0

    def on_status(self, status):
        phase = status.get('phase')
        if phase not in ACTIVE | TERMINAL:
            return
        # Reap an already exited process before deciding whether a fresh mission
        # can share its bag. Timer/status callback ordering must not lose a run.
        if self.process is not None:
            code = self.process.poll()
            if code is not None:
                self._finished(code)
        edge = starts_mission(self.phase, phase)
        self.phase = phase
        if self.process is not None and self.stopping_since is not None:
            if edge and self.enabled:
                self.pending = dict(status)
                self.queued_mission_seen = True
                return
            if self.queued_mission_seen:
                self.pending = dict(status) if phase in ACTIVE else None
                return
        if self.process is not None:
            self.manifest['last_status'] = dict(status)
            if phase in TERMINAL and self.stop_at is None:
                self.stop_at = self.clock() + self.post_roll_s
                self.manifest.update(final_phase=phase, error_code=status.get('error_code', ''))
                self._write_manifest()
        if edge and self.enabled:
            if self.process is None:
                self._start(status)
            else:
                self.stop_at = None
                self.manifest['missions'].append(dict(status))
                self.manifest.update(final_phase=None, error_code=None)
                self._write_manifest()

    def _start(self, status):
        self.manifest = None
        try:
            if self.disk_free(self.workspace) < self.min_free_gb * 1024**3:
                if not self.disk_warned:
                    self.warn('Elevator recording skipped: free disk below min_free_gb')
                    self.disk_warned = True
                return
            instant = self.wall_clock()
            path = bag_folder(self.workspace, instant, status.get('start_floor', ''),
                              status.get('goal_floor', ''))
            base, sequence = path, 1
            while path.exists() or path.with_name(path.name + '.manifest.json').exists():
                sequence += 1
                path = base.with_name(f'{base.name}_{sequence}')
            path.parent.mkdir(parents=True, exist_ok=True)
            self.manifest_path = path.with_name(path.name + '.manifest.json')
            tuning, tuning_error = None, None
            try:
                if self.tuning_file:
                    tuning = Path(self.tuning_file).read_text()
            except OSError as error:
                tuning_error = str(error)
            self.started = self.clock()
            self.manifest = {
                'started_at': instant.astimezone(KST).isoformat(), 'timezone': 'Asia/Seoul',
                'workspace': str(self.workspace), 'bag_dir': str(path),
                'mission': dict(status), 'missions': [dict(status)], 'last_status': dict(status),
                'start_floor': status.get('start_floor'), 'goal_floor': status.get('goal_floor'),
                'detail': status.get('detail', ''), 'git': self.identity(self.workspace),
                'tuning_file': self.tuning_file, 'tuning_yaml': tuning, 'tuning_error': tuning_error,
                'topics': self.topics, 'final_phase': None, 'error_code': None,
            }
            self._write_manifest()
            self.process = self.launcher(
                ['nice', '-n', '10', 'ros2', 'bag', 'record', '-s', 'mcap', '-o', str(path), *self.topics],
                start_new_session=True, stdin=subprocess.DEVNULL, preexec_fn=die_with_parent)
            self.stop_at = self.stopping_since = None
            self.queued_mission_seen = False
            self.signal_stage = 0
        except (OSError, subprocess.SubprocessError) as error:
            self.warn(f'Elevator recorder could not start: {error}')
            if self.manifest is not None:
                self.manifest['recorder_error'] = str(error)
                self._write_manifest()

    def _write_manifest(self):
        try:
            bag_dir = Path(self.manifest['bag_dir'])
            target = bag_dir / 'manifest.json' if bag_dir.is_dir() else self.manifest_path
            temporary = target.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(self.manifest, ensure_ascii=False, indent=2) + '\n')
            temporary.replace(target)
            if target != self.manifest_path:
                self.manifest_path.unlink(missing_ok=True)
                self.manifest_path = target
        except OSError as error:
            self.warn(f'Elevator manifest write failed: {error}')

    def _stop(self, reason):
        if self.process is None or self.stopping_since is not None:
            return
        self.stopping_since = self.clock()
        self.manifest['stop_reason'] = reason
        self.send_signal(self.process, signal.SIGINT)
        self._write_manifest()

    def _finished(self, exit_code):
        if self.stopping_since is None:
            self.manifest['stop_reason'] = 'process_exit'
            self.warn(f'Elevator recorder exited before stop was requested: {exit_code}')
        self.manifest.update(
            finished_at=self.wall_clock().astimezone(KST).isoformat(),
            duration_s=self.clock() - self.started, exit_code=exit_code,
            final_phase=self.manifest['last_status'].get('phase'),
            error_code=self.manifest['last_status'].get('error_code', ''))
        self._write_manifest()
        self.process = None
        pending, self.pending = self.pending, None
        if pending is not None and self.enabled:
            self._start(pending)

    def tick(self):
        if self.process is None:
            return
        if self.manifest_path.name != 'manifest.json' and Path(self.manifest['bag_dir']).is_dir():
            self._write_manifest()
        code = self.process.poll()
        if code is not None:
            self._finished(code)
            return
        now = self.clock()
        if self.stopping_since is None and now - self.disk_checked >= 5.:
            self.disk_checked = now
            try:
                low = self.disk_free(self.workspace) < self.min_free_gb * 1024**3 / 2
            except OSError:
                low = False
            if low:
                self.warn('Elevator recording stopped: free disk below half of min_free_gb')
                self._stop('disk_low')
        if now - self.started >= self.max_duration_s:
            self._stop('max_duration')
        elif self.stop_at is not None and now >= self.stop_at:
            self._stop('terminal')
        if self.stopping_since is not None:
            elapsed = now - self.stopping_since
            if elapsed >= 10. and self.signal_stage < 2:
                self.send_signal(self.process, signal.SIGKILL)
                self.signal_stage = 2
            elif elapsed >= 5. and self.signal_stage < 1:
                self.send_signal(self.process, signal.SIGTERM)
                self.signal_stage = 1

    def shutdown(self):
        self.enabled, self.pending = False, None
        if self.process is None:
            return
        self._stop('shutdown')
        # Wait only during shutdown; the ROS timer never blocks on process exit.
        for sig, timeout in ((None, 5.), (signal.SIGTERM, 5.), (signal.SIGKILL, 5.)):
            if sig is not None:
                self.send_signal(self.process, sig)
            try:
                code = self.process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
            self._finished(code)
            return
        self.warn('Elevator recorder did not exit after SIGKILL')
        self.manifest.update(duration_s=self.clock() - self.started,
                             final_phase=self.phase, recorder_error='shutdown wait timed out')
        self._write_manifest()


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from ament_index_python.packages import get_package_share_directory
    from amr_stack.floor_config import find_workspace_root

    class ElevatorRecorder(Node):
        def __init__(self):
            super().__init__('elevator_recorder')
            defaults = dict(enabled=True, post_roll_s=5., max_duration_s=900., min_free_gb=5.,
                            topics=DEFAULT_TOPICS, tuning_file=str(
                                Path(get_package_share_directory('amr_elevator')) / 'config/elevator_tuning.yaml'))
            params = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
            self.core = RecorderCore(find_workspace_root(), **params, warn=self.get_logger().warning)
            self.create_subscription(String, '/floor_mission/status', self.on_status,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_timer(.1, self.core.tick)

        def on_status(self, message):
            try:
                status = json.loads(message.data)
                if isinstance(status, dict):
                    self.core.on_status(status)
            except (ValueError, TypeError) as error:
                self.get_logger().warning(f'Invalid floor mission status: {error}')

    rclpy.init(args=args)
    node = None
    try:
        node = ElevatorRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.core.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
