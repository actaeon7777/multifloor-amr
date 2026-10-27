"""Speak ``robot/announcement`` messages (SPS-B-KOLA-0001-7700 4.1.3 b) from WAV files.

The WAV files are generated off-robot by ``tools/tts/generate_google_tts.py``; this node
only maps a message to file names and plays them one at a time through ``pw-play``
(the PipeWire default sink, the HDMI display speaker on the robot). While the wheels
turn, a driving alert (``drive.wav``, one beep per 5 s file) loops between announcements.
The node itself announces two things nobody else does: Nav2 standing still while it runs a
goal (someone in the corridor) and the E-stop tripping.
"""
from collections import deque
import json
import math
import shlex
import subprocess
import threading
import time
from pathlib import Path

INTERRUPT = frozenset({'estop', 'robot_error', 'elevator_error', 'shock'})
ESTOP_CLEAR = frozenset({'READY', 'ARMING', 'BYPASS'})
# Annex C note c says the target floor once the car stands there, which is 'arrival'.
IGNORED = frozenset({'alight_floor'})


def sound_stem(message):
    """WAV file stem for one announcement message, or None to stay silent."""
    key = str(message.get('key', ''))
    if not key or key in IGNORED:
        return None
    if key == 'arrival':
        return f'arrival_{message.get("floor", "")}'
    if key == 'room_arrival':
        return f'room_{str(message.get("room", "")).split("-")[0]}'
    return key


class MotionGate:
    """Driving when measured motion lasted ``start_s``; stopped after ``stop_s`` still or stale."""

    def __init__(self, linear=0.03, angular=0.1, start_s=0.3, stop_s=2., stale_s=1.):
        self.linear, self.angular = linear, angular
        self.start_s, self.stop_s, self.stale_s = start_s, stop_s, stale_s
        self.moving_since = self.still_since = self.received = None
        self.driving = False

    def update(self, v, w, now):
        self.received = now
        if math.isfinite(v) and math.isfinite(w) and (abs(v) > self.linear or abs(w) > self.angular):
            self.still_since = None
            if self.moving_since is None:
                self.moving_since = now
        else:
            self.moving_since = None
            if self.still_since is None:
                self.still_since = now

    def still_for(self, now):
        """Seconds of fresh odometry below the thresholds; 0 when moving or stale."""
        if self.received is None or now - self.received > self.stale_s or self.still_since is None:
            return 0.
        return now - self.still_since

    def state(self, now):
        if self.received is None or now - self.received > self.stale_s:
            self.driving = False
        elif self.moving_since is not None and now - self.moving_since >= self.start_s:
            self.driving = True
        elif self.still_since is not None and now - self.still_since >= self.stop_s:
            self.driving = False
        return self.driving


class StallWatch:
    """True when Nav2 runs a goal but the robot stood still ``stall_s``, then every ``repeat_s``."""

    def __init__(self, stall_s=3., repeat_s=10.):
        self.stall_s, self.repeat_s = stall_s, repeat_s
        self.last = None

    def update(self, running, still_for, now):
        if not running or still_for < self.stall_s:
            self.last = None
            return False
        if self.last is not None and now - self.last < self.repeat_s:
            return False
        self.last = now
        return True


def estop_tripped(previous, state):
    """Announce only a trip from a working state, not the UNKNOWN/ESTOP state at start-up."""
    return state == 'ESTOP' and previous in ESTOP_CLEAR


class Player:
    """Queue of announcements played sequentially; safety keys cut in front.

    The driving sound only plays while ``set_driving(True)`` and nothing is queued; a new
    announcement or the robot stopping cuts it off at once. ``spawn`` and ``clock`` are
    injectable so the queue runs without audio in tests.
    """

    def __init__(self, sound_dir, command=('pw-play',), chime='chime', drive='drive', max_queue=3,
                 max_age=5., voice_volume=None, spawn=subprocess.Popen, clock=time.monotonic, warn=print):
        self.sound_dir = Path(sound_dir)
        self.command = list(command)
        # pw-play --volume for the chime and speech; drive.wav is already mixed quieter.
        self.voice_args = [] if voice_volume is None else ['--volume', f'{voice_volume:g}']
        self.chime = chime
        self.drive = self.sound_dir / f'{drive}.wav' if drive else None
        self.max_age = max_age
        self.spawn, self.clock, self.warn = spawn, clock, warn
        self.queue = deque(maxlen=max_queue)
        self.cond = threading.Condition()
        self.proc = None
        self.proc_is_drive = False
        self.current_key = None
        self.driving = False
        self.generation = 0
        self.closed = False
        self.thread = None
        self._missing = set()

    def files(self, stem):
        path = self.sound_dir / f'{stem}.wav'
        if not path.is_file():
            if stem not in self._missing:
                self._missing.add(stem)
                self.warn(f'Announcement sound missing: {path}')
            return []
        chime = self.sound_dir / f'{self.chime}.wav'
        return [chime, path] if self.chime and chime.is_file() else [path]

    def _stop_drive(self):
        if self.proc is not None and self.proc_is_drive:
            self.proc.terminate()

    def submit(self, message):
        """Queue one message; returns True when it has a sound to play."""
        stem = sound_stem(message)
        files = self.files(stem) if stem else []
        if not files:
            return False
        key = message.get('key')
        with self.cond:
            if key in INTERRUPT:
                # Cut ordinary speech, but let one safety message finish before the next.
                if self.current_key not in INTERRUPT:
                    self.generation += 1
                    if self.proc is not None:
                        self.proc.terminate()
                kept = [(k, f, t, self.generation) for k, f, t, _ in self.queue if k in INTERRUPT]
                self.queue.clear()
                self.queue.extend(kept)
            elif any(queued_key == key for queued_key, *_ in self.queue):
                return True  # Already waiting to be said; repeats add nothing.
            self._stop_drive()
            self.queue.append((key, files, self.clock(), self.generation))
            self.cond.notify()
        return True

    def set_driving(self, driving):
        with self.cond:
            if driving == self.driving:
                return
            self.driving = driving
            if not driving:
                self._stop_drive()
            self.cond.notify()

    def _drive_ready(self):
        return self.driving and self.drive is not None and self.drive.is_file()

    def _run(self, path, is_drive):
        """Spawn one file under the lock and wait; the caller already checked it may play."""
        self.proc = self.spawn([*self.command, *([] if is_drive else self.voice_args), str(path)],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.proc_is_drive = is_drive
        proc = self.proc
        self.cond.release()
        try:
            proc.wait()
        finally:
            self.cond.acquire()
            self.proc, self.proc_is_drive = None, False

    def play_next(self, timeout=None):
        """Play the oldest fresh entry, else one driving loop; False when nothing played."""
        with self.cond:
            ready = lambda: self.queue or self.closed or self._drive_ready()  # noqa: E731
            if not self.cond.wait_for(ready, timeout) or self.closed:
                return False
            if not self.queue:
                self._run(self.drive, True)
                return True
            key, files, queued_at, generation = self.queue.popleft()
            if self.clock() - queued_at > self.max_age:
                return False  # Stale: the situation it described has moved on.
            self.current_key = key
            try:
                for path in files:
                    if self.closed or generation != self.generation:
                        break
                    self._run(path, False)
            finally:
                self.current_key = None
            return True

    def start(self):
        def loop():
            while not self.closed:
                try:
                    self.play_next()
                except OSError as error:  # Missing player binary or audio server.
                    self.warn(f'Announcement playback failed: {error!r}')
                    time.sleep(1.)
        self.thread = threading.Thread(target=loop, name='announcement_player', daemon=True)
        self.thread.start()

    def close(self):
        with self.cond:
            self.closed = True
            self.queue.clear()
            if self.proc is not None:
                self.proc.terminate()
            self.cond.notify_all()
        if self.thread is not None:
            self.thread.join(timeout=2.)


def default_sound_dir():
    from amr_stack.floor_config import find_workspace_root
    try:
        return str(find_workspace_root() / 'tools/tts/wav')
    except Exception:  # noqa: BLE001 - installed without a workspace; require the parameter
        return ''


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    rclpy.init()
    node = Node('announcement_player')
    sound_dir = node.declare_parameter('sound_dir', default_sound_dir()).value
    command = shlex.split(node.declare_parameter('command', 'pw-play').value)
    max_age = node.declare_parameter('max_age_s', 5.).value
    volume = node.declare_parameter('voice_volume', 0.6).value  # pw-play only; < 0 leaves it out
    drive = node.declare_parameter('drive_sound', 'drive').value
    odom_topic = node.declare_parameter('odom_topic', '/odometry/filtered').value
    stall = StallWatch(node.declare_parameter('nav_stall_s', 3.).value,
                       node.declare_parameter('nav_stall_repeat_s', 10.).value)
    player = Player(sound_dir, command, drive=drive, max_age=max_age,
                    voice_volume=volume if volume >= 0 else None, warn=node.get_logger().warning)
    gate = MotionGate()
    status = {'navigation': '', 'estop': ''}

    def on_message(msg):
        try:
            message = json.loads(msg.data)
        except ValueError:
            node.get_logger().warning(f'Announcement is not JSON: {msg.data!r}')
            return
        if isinstance(message, dict):
            player.submit(message)

    def on_odom(msg):
        gate.update(msg.twist.twist.linear.x, msg.twist.twist.angular.z, time.monotonic())

    def on_navigation(msg):
        status['navigation'] = msg.data

    def on_estop(msg):
        try:
            state = json.loads(msg.data).get('state', '')
        except (ValueError, AttributeError):
            return
        if estop_tripped(status['estop'], state):
            player.submit({'key': 'estop'})
        status['estop'] = state

    def tick():
        now = time.monotonic()
        player.set_driving(bool(drive) and gate.state(now))
        if stall.update(status['navigation'] == 'RUNNING', gate.still_for(now), now):
            player.submit({'key': 'obstacle'})

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, 'robot/announcement', on_message, 10)
    node.create_subscription(Odometry, odom_topic, on_odom, 1)
    node.create_subscription(String, '/navigation/status', on_navigation, latched)
    node.create_subscription(String, '/estop/status', on_estop, latched)
    node.create_timer(0.2, tick)
    player.start()
    node.get_logger().info(f'Announcement sounds from {sound_dir} via {command[0]}'
                           + (f', driving sound from {odom_topic}' if drive else ''))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        player.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
