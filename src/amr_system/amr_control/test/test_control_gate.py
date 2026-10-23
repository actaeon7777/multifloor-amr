"""Mode arbitration regression tests with a deterministic monotonic clock."""

import pytest

from amr_control.control_gate import ControlGate


def joy(gate, now, lb=False, rb=False):
    gate.receive_joy([0.0] * 8, [0] * 6 + [int(lb), int(rb)], now)


def twist(v=0.1, w=-0.2):
    return (v, 0.0, 0.0, 0.0, 0.0, w)


def auto(connected=True):
    """AUTO with a fresh command; optionally an idle connected joystick (LB up)."""
    gate = ControlGate()
    if connected:
        joy(gate, 0.0)
    assert gate.mode == 'AUTO'
    gate.receive_auto_command(twist(), 0.01)
    assert gate.output(0.02) == (0.1, -0.2)
    return gate


def takeover(gate, now):
    """Press LB, then hold it with a manual stick command."""
    joy(gate, now, lb=True)
    assert gate.mode == 'MANUAL'
    gate.receive_command(twist(-0.1, 0.3), now + 0.01)
    assert gate.output(now + 0.02) == (0.0, 0.0)  # mode transition stop
    joy(gate, now + 0.11, lb=True)
    gate.receive_command(twist(-0.1, 0.3), now + 0.11)
    assert gate.output(now + 0.12) == (-0.1, 0.3)


def test_startup_is_auto_without_waiting_for_a_joystick():
    gate = ControlGate()
    assert gate.mode == 'AUTO'
    assert gate.reason == 'auto_waiting_command'
    gate.receive_auto_command(twist(), 0.0)
    assert gate.output(0.01) == (0.1, -0.2)


@pytest.mark.parametrize('rb', [False, True])
def test_connected_idle_joystick_and_rb_do_not_stop_auto(rb):
    gate = auto(connected=False)
    for step in range(20):
        joy(gate, 0.03 + step * 0.05, rb=rb)
        gate.receive_auto_command(twist(), 0.04 + step * 0.05)
        gate.receive_command(twist(-0.2, 0.4), 0.04 + step * 0.05)
    assert gate.mode == 'AUTO'
    assert gate.output(1.0) == (0.1, -0.2)


def test_joystick_loss_and_reconnect_do_not_change_auto():
    gate = auto()
    gate.receive_auto_command(twist(), 0.9)
    assert gate.output(0.95) == (0.1, -0.2)  # Joy silent for 0.95 s.
    joy(gate, 1.0)
    gate.receive_auto_command(twist(), 1.01)
    assert gate.mode == 'AUTO'
    assert gate.output(1.02) == (0.1, -0.2)


def test_lb_press_takes_over_immediately_and_arms_with_one_press():
    gate = auto()
    joy(gate, 0.03, lb=True)
    assert gate.mode == 'MANUAL'
    assert gate.reason == 'waiting_command'
    assert gate.output(0.03) == (0.0, 0.0)
    gate.receive_auto_command(twist(), 0.04)
    assert gate.auto_command is None
    gate.receive_command(twist(-0.1, 0.3), 0.14)
    assert gate.output(0.15) == (-0.1, 0.3)


def test_held_lb_keeps_manual_and_ignores_auto():
    gate = auto()
    takeover(gate, 0.03)
    for now in (0.20, 0.40, 0.60, 0.80):
        joy(gate, now, lb=True)
        gate.receive_command(twist(-0.1, 0.3), now)
        gate.receive_auto_command(twist(0.2, 0.4), now)
    assert gate.mode == 'MANUAL'
    assert gate.output(0.81) == (-0.1, 0.3)


def test_lb_release_stops_then_auto_returns_after_holdoff_without_old_command():
    gate = auto()
    takeover(gate, 0.03)
    joy(gate, 0.20)  # LB released; last LB sample was 0.14.
    assert gate.output(0.31) == (0.0, 0.0)
    assert gate.reason == 'deadman_released'
    gate.receive_auto_command(twist(), 0.60)
    assert gate.mode == 'MANUAL' and gate.output(0.61) == (0.0, 0.0)
    assert gate.output(0.64) == (0.0, 0.0)
    assert gate.mode == 'AUTO'
    assert gate.reason == 'auto_waiting_command'
    assert gate.output(0.80) == (0.0, 0.0)  # Nothing resumes on its own.
    gate.receive_auto_command(twist(0.05, 0.1), 0.81)
    assert gate.output(0.82) == (0.05, 0.1)


def test_brief_lb_bounce_does_not_end_manual():
    gate = auto()
    takeover(gate, 0.03)
    joy(gate, 0.20)
    joy(gate, 0.24, lb=True)
    gate.receive_command(twist(-0.1, 0.3), 0.25)
    assert gate.mode == 'MANUAL'
    assert gate.output(0.26) == (-0.1, 0.3)


def test_second_takeover_arms_after_auto_returns():
    gate = auto()
    takeover(gate, 0.03)
    joy(gate, 0.20)
    joy(gate, 0.70)
    assert gate.mode == 'AUTO'
    gate.receive_auto_command(twist(), 0.71)
    assert gate.output(0.82) == (0.1, -0.2)
    takeover(gate, 0.90)


def test_lb_held_when_joystick_connects_stops_auto_but_requires_release():
    gate = auto(connected=False)
    for now in (0.03, 0.10, 0.20, 0.30):
        joy(gate, now, lb=True)
        gate.receive_command(twist(-0.1, 0.3), now)
    assert gate.mode == 'MANUAL'
    assert gate.reason == 'awaiting_release'
    assert gate.output(0.31) == (0.0, 0.0)
    joy(gate, 0.32)
    joy(gate, 0.33, lb=True)
    gate.receive_command(twist(-0.1, 0.3), 0.34)
    assert gate.output(0.35) == (-0.1, 0.3)


def test_joy_loss_with_lb_held_stops_manual_then_returns_to_auto():
    gate = auto()
    takeover(gate, 0.03)  # Last Joy and command at 0.14, then Joy is lost.
    assert gate.output(0.63) == (-0.1, 0.3)
    assert gate.mode == 'MANUAL'
    assert gate.output(0.65) == (0.0, 0.0)  # Joy timeout and holdoff from 0.14.
    assert gate.mode == 'AUTO'
    joy(gate, 0.80, lb=True)  # Reconnect with LB held: stop AUTO, no drive.
    gate.receive_command(twist(-0.1, 0.3), 0.81)
    assert gate.mode == 'MANUAL' and gate.output(0.92) == (0.0, 0.0)


@pytest.mark.parametrize('axes,buttons', [([0.0] * 8, [0] * 5),
                                         ([float('nan')] * 8, [0] * 6 + [1, 0]),
                                         ([0.0] * 8, [0] * 6 + [1, 2])])
def test_invalid_joy_is_not_a_takeover(axes, buttons):
    gate = auto()
    gate.receive_joy(axes, buttons, 0.03)
    gate.receive_auto_command(twist(), 0.04)
    assert gate.mode == 'AUTO'
    assert gate.output(0.05) == (0.1, -0.2)


def test_auto_timeout_stops_without_manual_fallback_and_fresh_auto_resumes():
    gate = auto()
    gate.receive_command(twist(-0.2, 0.4), 0.3)
    assert gate.output(0.52) == (0.0, 0.0)
    assert gate.mode == 'AUTO'
    assert gate.reason == 'auto_command_timeout'
    gate.receive_auto_command(twist(0.05, 0.1), 0.53)
    assert gate.output(0.54) == (0.05, 0.1)


@pytest.mark.parametrize('values', [twist(float('nan')), twist(w=float('inf')),
                                    (0.1, 0.1, 0.0, 0.0, 0.0, 0.0), ()])
def test_invalid_auto_command_stops_and_valid_command_recovers(values):
    gate = auto()
    gate.receive_auto_command(values, 0.03)
    assert gate.output(0.04) == (0.0, 0.0)
    assert gate.reason == 'auto_invalid_command'
    gate.receive_auto_command(twist(), 0.05)
    assert gate.output(0.06) == (0.1, -0.2)


def test_auto_limits_and_inactive_auto_cannot_affect_manual():
    gate = auto()
    gate.receive_auto_command(twist(10.0, -10.0), 0.03)
    assert gate.output(0.04) == (0.2, -0.4)
    gate = auto()
    takeover(gate, 0.03)
    gate.receive_auto_command(twist(float('nan')), 0.13)
    assert gate.output(0.14) == (-0.1, 0.3)


def test_manual_release_bounce_and_command_delay_still_recover():
    gate = auto()
    takeover(gate, 0.03)
    joy(gate, 0.15)
    assert gate.output(0.18) == (-0.1, 0.3)
    joy(gate, 0.19, lb=True)
    joy(gate, 0.50, lb=True)
    assert gate.output(0.65) == (0.0, 0.0)  # Manual command timeout from 0.14.
    assert gate.mode == 'MANUAL'
    gate.receive_command(twist(-0.1, 0.3), 0.66)
    assert gate.output(0.67) == (-0.1, 0.3)


@pytest.mark.parametrize('kwargs', [
    {'auto_timeout': 0.0}, {'auto_timeout': float('nan')},
    {'manual_holdoff': 0.4}, {'manual_holdoff': float('inf')},
    {'mode_stop_duration': -0.1}, {'mode_stop_duration': 0.5},
    {'mode_stop_duration': float('inf')},
])
def test_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        ControlGate(**kwargs)
