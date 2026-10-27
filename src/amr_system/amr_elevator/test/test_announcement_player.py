"""Announcement player queue and file mapping, without ROS or audio."""
import threading

from amr_elevator.announcement_player import MotionGate, Player, StallWatch, estop_tripped, sound_stem


class FakeProc:
    def __init__(self, args, block=None):
        self.args, self.block = args, block
        self.terminated = False

    def wait(self):
        if self.block is not None:
            self.block.wait(5.)

    def terminate(self):
        self.terminated = True
        if self.block is not None:
            self.block.set()


def make_player(tmp_path, stems=('chime', 'obstacle', 'call', 'robot_error', 'arrival_4F'), **kwargs):
    for stem in stems:
        (tmp_path / f'{stem}.wav').write_bytes(b'RIFF')
    played, warnings = [], []
    block = kwargs.pop('block', None)  # Only the first spawned process blocks.
    def spawn(args, **_):
        proc = FakeProc(args, None if played else block)
        played.append(proc)
        return proc
    clock = kwargs.pop('clock', lambda: 0.)
    player = Player(tmp_path, ['pw-play'], spawn=spawn, clock=clock, warn=warnings.append, **kwargs)
    return player, played, warnings


def names(played):
    return [p.args[-1].rsplit('/', 1)[-1] for p in played]


def test_sound_stem_mapping():
    assert sound_stem({'key': 'obstacle'}) == 'obstacle'
    assert sound_stem({'key': 'arrival', 'floor': '4F'}) == 'arrival_4F'
    assert sound_stem({'key': 'room_arrival', 'room': '413-앞문'}) == 'room_413'
    assert sound_stem({'key': 'alight_floor', 'floor': '4F'}) is None
    assert sound_stem({}) is None


def test_plays_chime_then_phrase_and_skips_missing_once(tmp_path):
    player, played, warnings = make_player(tmp_path)
    assert player.submit({'key': 'arrival', 'floor': '4F'})
    assert player.play_next(timeout=0)
    assert names(played) == ['chime.wav', 'arrival_4F.wav']
    assert played[0].args[0] == 'pw-play'
    assert not player.submit({'key': 'board_done'})
    assert not player.submit({'key': 'board_done'})
    assert len(warnings) == 1 and 'board_done.wav' in warnings[0]


def test_without_chime_file_plays_phrase_only(tmp_path):
    player, played, _ = make_player(tmp_path, stems=('obstacle',))
    player.submit({'key': 'obstacle'})
    player.play_next(timeout=0)
    assert names(played) == ['obstacle.wav']


def test_queued_repeat_is_coalesced_and_stale_entries_dropped(tmp_path):
    now = [0.]
    player, played, _ = make_player(tmp_path, clock=lambda: now[0], max_age=5.)
    player.submit({'key': 'obstacle'})
    player.submit({'key': 'obstacle'})
    player.submit({'key': 'call'})
    assert len(player.queue) == 2
    now[0] = 6.
    assert not player.play_next(timeout=0)
    assert not player.play_next(timeout=0)
    assert played == []


def test_safety_key_interrupts_current_and_clears_queue(tmp_path):
    block = threading.Event()
    player, played, _ = make_player(tmp_path, block=block)
    player.submit({'key': 'obstacle'})
    worker = threading.Thread(target=player.play_next, kwargs={'timeout': 1.})
    worker.start()
    for _ in range(100):
        if played:
            break
        threading.Event().wait(0.01)
    player.submit({'key': 'call'})
    player.submit({'key': 'robot_error'})
    worker.join(2.)
    assert not worker.is_alive()
    assert played[0].terminated
    assert names(played) == ['chime.wav']  # The interrupted entry does not continue.
    assert [key for key, *_ in player.queue] == ['robot_error']
    player.play_next(timeout=0)
    assert names(played)[1:] == ['chime.wav', 'robot_error.wav']


def test_close_stops_waiting_worker(tmp_path):
    player, _, _ = make_player(tmp_path)
    player.start()
    player.close()
    assert not player.thread.is_alive()


def wait_until(predicate):
    for _ in range(200):
        if predicate():
            return True
        threading.Event().wait(0.01)
    return False


def test_motion_gate_start_stop_and_stale():
    gate = MotionGate(start_s=0.3, stop_s=2., stale_s=1.)
    assert not gate.state(0.)
    gate.update(0.2, 0., 0.)
    assert not gate.state(0.2)
    gate.update(0.2, 0., 0.3)
    assert gate.state(0.3)
    gate.update(0., 0.05, 0.5)  # Below both thresholds: still.
    gate.update(0., 0., 1.4)
    assert gate.state(2.4)
    gate.update(0., 0., 2.5)
    assert not gate.state(2.5)
    gate.update(0., 0.3, 3.)  # Turning in place counts as driving.
    gate.update(0., 0.3, 3.4)
    assert gate.state(3.4)
    assert not gate.state(4.5)  # No odometry for over 1 s.
    gate.update(float('nan'), 0., 5.)
    gate.update(float('nan'), 0., 5.5)
    assert not gate.state(5.5)


def test_drive_sound_loops_only_while_driving(tmp_path):
    player, played, _ = make_player(tmp_path, stems=('chime', 'obstacle', 'drive'))
    assert not player.play_next(timeout=0)
    player.set_driving(True)
    assert player.play_next(timeout=0) and player.play_next(timeout=0)
    assert names(played) == ['drive.wav', 'drive.wav']
    player.set_driving(False)
    assert not player.play_next(timeout=0)


def test_announcement_and_stop_cut_the_drive_sound(tmp_path):
    block = threading.Event()
    player, played, _ = make_player(tmp_path, stems=('chime', 'obstacle', 'drive'), block=block)
    player.set_driving(True)
    worker = threading.Thread(target=player.play_next, kwargs={'timeout': 1.})
    worker.start()
    assert wait_until(lambda: played)
    player.submit({'key': 'obstacle'})
    worker.join(2.)
    assert played[0].terminated and names(played) == ['drive.wav']
    player.play_next(timeout=0)
    assert names(played)[1:] == ['chime.wav', 'obstacle.wav']  # Announcement before the next loop.

    block2 = threading.Event()
    player2, played2, _ = make_player(tmp_path, stems=(), block=block2)
    player2.set_driving(True)
    worker = threading.Thread(target=player2.play_next, kwargs={'timeout': 1.})
    worker.start()
    assert wait_until(lambda: played2)
    player2.set_driving(False)
    worker.join(2.)
    assert played2[0].terminated


def test_announcement_does_not_cut_another_announcement(tmp_path):
    block = threading.Event()
    player, played, _ = make_player(tmp_path, stems=('obstacle', 'call'), block=block)
    player.submit({'key': 'obstacle'})
    worker = threading.Thread(target=player.play_next, kwargs={'timeout': 1.})
    worker.start()
    assert wait_until(lambda: played)
    player.submit({'key': 'call'})
    assert not played[0].terminated
    block.set()
    worker.join(2.)
    player.play_next(timeout=0)
    assert names(played) == ['obstacle.wav', 'call.wav']


def test_voice_volume_applies_to_speech_not_drive(tmp_path):
    player, played, _ = make_player(tmp_path, stems=('chime', 'obstacle', 'drive'), voice_volume=0.6)
    player.submit({'key': 'obstacle'})
    player.play_next(timeout=0)
    player.set_driving(True)
    player.play_next(timeout=0)
    assert [p.args[:-1] for p in played] == [['pw-play', '--volume', '0.6']] * 2 + [['pw-play']]


def test_still_for_counts_fresh_stillness_only():
    gate = MotionGate(stale_s=1.)
    assert gate.still_for(0.) == 0.
    gate.update(0., 0., 0.)
    gate.update(0., 0., 0.9)
    assert gate.still_for(1.5) == 1.5
    assert gate.still_for(2.) == 0.  # Odometry stale: not known to be still.
    gate.update(0.2, 0., 2.)
    assert gate.still_for(2.) == 0.


def test_stall_watch_waits_then_repeats_and_resets():
    watch = StallWatch(stall_s=3., repeat_s=10.)
    assert not watch.update(True, 2.9, 0.)
    assert watch.update(True, 3., 1.)
    assert not watch.update(True, 9., 10.9)
    assert watch.update(True, 10., 11.)
    assert not watch.update(False, 20., 12.)  # Goal ended.
    assert watch.update(True, 3., 13.)  # A new hold speaks again at once.
    assert not watch.update(True, 0., 14.)  # Moving again resets.
    assert watch.update(True, 3., 15.)


def test_estop_announced_only_on_trip_from_working_state():
    assert estop_tripped('READY', 'ESTOP')
    assert estop_tripped('ARMING', 'ESTOP')
    assert not estop_tripped('', 'ESTOP')  # Node start: latched state, not a new trip.
    assert not estop_tripped('UNKNOWN', 'ESTOP')
    assert not estop_tripped('ESTOP', 'ESTOP')
    assert not estop_tripped('READY', 'READY')


def test_safety_message_is_not_cut_by_another_safety_message(tmp_path):
    block = threading.Event()
    player, played, _ = make_player(tmp_path, stems=('estop', 'robot_error', 'obstacle'), block=block)
    player.submit({'key': 'estop'})
    worker = threading.Thread(target=player.play_next, kwargs={'timeout': 1.})
    worker.start()
    assert wait_until(lambda: played)
    player.submit({'key': 'obstacle'})
    player.submit({'key': 'robot_error'})
    assert not played[0].terminated
    assert [key for key, *_ in player.queue] == ['robot_error']  # Ordinary speech dropped.
    block.set()
    worker.join(2.)
    player.play_next(timeout=0)
    assert names(played) == ['estop.wav', 'robot_error.wav']
