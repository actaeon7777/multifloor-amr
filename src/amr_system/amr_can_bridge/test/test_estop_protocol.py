"""0x202 decoding and the Jetson restart interlock, without ROS or CAN."""

import struct

import pytest

from amr_can_bridge.estop_protocol import (
    ARMING, ESTOP, READY, SEND_COMMAND, SEND_NOTHING, SEND_ZERO, UNKNOWN,
    EstopSupervisor, InvalidStatus, decode_estop_status,
)


def frame(flags=0, trip=0, sequence=0, release=0, rejected=0, version=1):
    return struct.pack('<BBHHBB', version, flags, trip, sequence, release, rejected)


def status(**kwargs):
    return decode_estop_status(0x202, frame(**kwargs))


@pytest.mark.parametrize('data, stopped, expected', [
    ('0100000000000000', False, {'drive_enabled': False, 'trip_count': 0}),
    ('010F010034120000', True, {'raw': True, 'latched': True, 'inhibit': True,
                                'trip_count': 1, 'sequence': 0x1234}),
    ('010E010000000000', True, {'raw': False, 'debounced': True, 'latched': True}),
    ('0100010000000100', False, {'latched': False, 'release_count': 1}),
    ('0108000000000000', True, {'latched': False, 'inhibit': True}),
    ('01C0000000000000', False, {'drive_enabled': True, 'odrv_hb_ok': True}),
])
def test_decode_documented_frames(data, stopped, expected):
    decoded = decode_estop_status(0x202, bytes.fromhex(data))
    assert decoded['stopped'] is stopped
    for key, value in expected.items():
        assert decoded[key] == value


def test_decode_ignores_other_traffic_and_rejects_malformed_status():
    assert decode_estop_status(0x201, frame()) is None
    assert decode_estop_status(0x202, frame(), extended=True) is None
    for kwargs in ({'remote': True}, {}):
        with pytest.raises(InvalidStatus):
            decode_estop_status(0x202, frame()[:7] if not kwargs else frame(), **kwargs)
    with pytest.raises(InvalidStatus):
        decode_estop_status(0x202, frame(version=2))


def ready(supervisor, now=0.0):
    supervisor.receive(status(), now)
    assert supervisor.state == ARMING
    assert supervisor.output(now) == SEND_ZERO
    supervisor.zero_sent(now)
    supervisor.receive(status(flags=0x80), now + 0.05)
    assert supervisor.state == READY
    return now + 0.05


def test_boot_waits_for_status_then_arms_by_itself():
    supervisor = EstopSupervisor()
    assert supervisor.state == UNKNOWN
    assert supervisor.output(0.0) == SEND_NOTHING
    supervisor.receive(status(), 0.1)
    assert supervisor.state == ARMING and supervisor.reason == 'released'
    assert supervisor.output(0.1) == SEND_ZERO


def test_drive_enabled_before_first_zero_does_not_skip_arming():
    supervisor = EstopSupervisor()
    supervisor.receive(status(), 0.0)
    supervisor.receive(status(flags=0x80), 0.05)  # Stale gate state before the first zero.
    assert supervisor.state == ARMING
    supervisor.zero_sent(0.06)
    supervisor.receive(status(flags=0x80), 0.1)
    assert supervisor.state == READY
    assert supervisor.output(0.1) == SEND_COMMAND


def test_press_blocks_until_release_then_rearms():
    supervisor = EstopSupervisor()
    now = ready(supervisor)
    supervisor.receive(status(flags=0x0F, trip=1), now + 0.1)
    assert supervisor.state == ESTOP and supervisor.epoch == 1
    assert supervisor.output(now + 0.1) == SEND_NOTHING
    supervisor.receive(status(flags=0x0F, trip=1), now + 0.2)  # Doubled frame while latched.
    supervisor.receive(status(flags=0x0E, trip=1), now + 0.3)  # Release not yet confirmed.
    assert supervisor.state == ESTOP and supervisor.epoch == 1
    supervisor.receive(status(trip=1, release=1), now + 0.6)
    assert supervisor.state == ARMING and supervisor.epoch == 1
    assert supervisor.output(now + 0.6) == SEND_ZERO


def test_short_press_seen_only_as_trip_count_is_an_estop():
    supervisor = EstopSupervisor()
    now = ready(supervisor)
    supervisor.receive(status(flags=0x80, trip=1, release=1), now + 0.1)
    assert supervisor.epoch == 1
    # The open gate is from before the press: zeros must be sent again first.
    assert supervisor.state == ARMING
    assert supervisor.output(now + 0.1) == SEND_ZERO


def test_status_timeout_and_invalid_frame_trip():
    supervisor = EstopSupervisor(status_timeout=0.5)
    now = ready(supervisor)
    assert supervisor.output(now + 0.49) == SEND_COMMAND
    assert supervisor.output(now + 0.5) == SEND_NOTHING
    assert supervisor.state == ESTOP and supervisor.reason == 'status_timeout'
    supervisor.receive(status(flags=0x80), now + 0.6)
    assert supervisor.state == ARMING  # Recovered status re-arms with zeros first.
    supervisor.receive_invalid('invalid_status')
    assert supervisor.state == ESTOP and supervisor.epoch == 2


def test_gate_close_returns_to_arming_without_estop():
    supervisor = EstopSupervisor()
    now = ready(supervisor)
    supervisor.receive(status(), now + 0.1)
    assert supervisor.state == ARMING and supervisor.epoch == 0
    assert supervisor.output(now + 0.1) == SEND_ZERO
    supervisor.zero_sent(now + 0.15)
    supervisor.receive(status(flags=0x80), now + 0.2)
    assert supervisor.state == READY


def test_counter_reset_flags_reboot_and_wrap_does_not():
    supervisor = EstopSupervisor()
    supervisor.receive(status(sequence=0xfffd), 0.0)
    supervisor.zero_sent(0.0)
    supervisor.receive(status(flags=0x80, sequence=0xfffe), 0.1)
    supervisor.receive(status(flags=0x80, sequence=0xffff), 0.2)
    supervisor.receive(status(flags=0x80, sequence=1), 0.3)  # Wrap with a lost frame.
    assert supervisor.state == READY and supervisor.epoch == 0
    assert not supervisor.reboot_suspected
    supervisor.receive(status(sequence=500), 0.4)  # Jump: lost status.
    supervisor.receive(status(sequence=3), 0.5)
    assert supervisor.reboot_suspected and supervisor.epoch == 2
    report = supervisor.report(0.5)
    assert report['stm32']['sequence'] == 3 and report['status_fresh']


def test_fast_reboot_inside_status_timeout_is_an_estop():
    # IWDG reset: the stream resumes before the timeout with trip_count still 0.
    supervisor = EstopSupervisor(status_timeout=0.5)
    supervisor.receive(status(sequence=400), 0.0)
    supervisor.zero_sent(0.01)
    supervisor.receive(status(flags=0x80, sequence=401), 0.05)
    assert supervisor.state == READY
    supervisor.receive(status(flags=0x02, sequence=0), 0.4)  # Boot: gate closed.
    assert supervisor.epoch == 1 and supervisor.reason != 'drive_gate_closed'
    assert supervisor.state == ARMING
    assert supervisor.output(0.4) == SEND_ZERO


def test_large_forward_sequence_jump_trips():
    supervisor = EstopSupervisor()
    now = ready(supervisor)
    supervisor.receive(status(flags=0x80, sequence=64), now + 0.1)
    assert supervisor.state == READY
    supervisor.receive(status(flags=0x80, sequence=64 + 65), now + 0.2)
    assert supervisor.epoch == 1 and supervisor.state == ARMING


def test_slow_arming_is_reported_but_not_an_estop():
    supervisor = EstopSupervisor(arming_warn=5.0)
    supervisor.receive(status(), 0.0)
    for tick in range(1, 30):
        supervisor.receive(status(), tick * 0.2)  # ODrives never become ready.
    assert supervisor.state == ARMING and supervisor.arming_slow(5.8)
    report = supervisor.report(5.8)
    assert report['arming_slow'] and report['arming_for'] == pytest.approx(5.8)
    supervisor.zero_sent(5.8)
    supervisor.receive(status(flags=0x80), 6.0)
    assert supervisor.state == READY and not supervisor.arming_slow(6.0)
    assert supervisor.report(6.0)['arming_for'] is None
