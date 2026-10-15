"""Byte-level compatibility with the working STM32 sender and watchdog rules."""

import math
import struct

import pytest

from amr_can_bridge.drive_protocol import CommandWatchdog, make_payload


@pytest.mark.parametrize('v,w,hex_data', [
    (0.0, 0.0, '0000000000000000'),
    (0.2, 0.4, 'cdcc4c3ecdcccc3e'),
    (-0.2, -0.4, 'cdcc4cbecdccccbe'),
    (0.2, -0.4, 'cdcc4c3ecdccccbe'),
])
def test_known_wire_bytes(v, w, hex_data):
    payload = make_payload(v, w)
    assert len(payload) == 8
    assert payload.hex() == hex_data
    assert struct.unpack('<ff', payload) == pytest.approx((v, w))


@pytest.mark.parametrize('v,w', [(math.nan, 0.0), (0.0, math.inf), (-math.inf, 0), (1e100, 0)])
def test_unrepresentable_payload_rejected(v, w):
    with pytest.raises(ValueError):
        make_payload(v, w)


def receive(watchdog, v, w, now):
    watchdog.receive((v, 0.0, 0.0, 0.0, 0.0, w), now)


def moving():
    watchdog = CommandWatchdog()
    receive(watchdog, 0.0, 0.0, 0.0)
    receive(watchdog, 0.2, 0.4, 0.01)
    assert watchdog.sample(0.02) == (0.2, 0.4)
    return watchdog


def test_startup_requires_zero_but_command_delay_recovers_with_fresh_input():
    watchdog = CommandWatchdog()
    receive(watchdog, 0.2, 0.4, 0.0)
    assert watchdog.sample(0.01) == (0.0, 0.0)
    receive(watchdog, 0.0, 0.0, 0.02)
    receive(watchdog, 0.2, 0.4, 0.03)
    assert watchdog.sample(0.04) == (0.2, 0.4)
    assert watchdog.sample(0.54) == (0.0, 0.0)
    receive(watchdog, -0.2, -0.4, 0.55)
    assert watchdog.sample(0.56) == (-0.2, -0.4)


def test_timeout_and_continuous_stop():
    watchdog = moving()
    assert watchdog.sample(0.52) == (0.0, 0.0)
    assert watchdog.reason == 'command_timeout'
    assert watchdog.sample(10.0) == (0.0, 0.0)


def test_speed_limits_and_explicit_stop():
    watchdog = moving()
    receive(watchdog, 100.0, -100.0, 0.03)
    assert watchdog.sample(0.04) == (0.2, -0.4)
    receive(watchdog, 0.0, 0.0, 0.05)
    assert watchdog.sample(0.06) == (0.0, 0.0)


@pytest.mark.parametrize('values', [
    [], (math.nan, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, math.inf),
    (0, 1, 0, 0, 0, 0), (0, 0, 0, 1, 0, 0),
])
def test_invalid_command_discards_motion(values):
    watchdog = moving()
    watchdog.receive(values, 0.03)
    assert watchdog.sample(0.04) == (0.0, 0.0)
    receive(watchdog, 0.2, 0.4, 0.05)
    assert watchdog.sample(0.06) == (0.0, 0.0)


def test_fault_discards_motion_and_requires_zero():
    watchdog = moving()
    watchdog.stop('can_tx_error')
    receive(watchdog, 0.2, 0.4, 0.03)
    assert watchdog.sample(0.04) == (0.0, 0.0)


@pytest.mark.parametrize('kwargs', [
    {'timeout': 0}, {'timeout': math.inf}, {'max_linear_speed': -1},
    {'max_angular_speed': math.nan}, {'max_linear_speed': 1e100},
])
def test_bad_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        CommandWatchdog(**kwargs)
