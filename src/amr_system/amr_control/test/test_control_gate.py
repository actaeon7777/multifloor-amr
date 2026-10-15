"""Mode arbitration regression tests with a deterministic monotonic clock."""

import pytest

from amr_control.control_gate import ControlGate


def joy(gate, now, lb=False, rb=False):
    gate.receive_joy([0.0] * 8, [0] * 6 + [int(lb), int(rb)], now)


def twist(v=0.1, w=-0.2):
    return (v, 0.0, 0.0, 0.0, 0.0, w)


def auto():
    gate = ControlGate()
    joy(gate, 0.0)
    joy(gate, 0.12, rb=True)
    gate.receive_auto_command(twist(), 0.14)
    assert gate.output(0.23) == (0.1, -0.2)
    return gate


def test_startup_held_rb_and_pre_entry_commands_do_not_drive():
    gate = ControlGate()
    gate.receive_auto_command(twist(), 0.0)
    joy(gate, 0.01, lb=True, rb=True)
    assert gate.mode == 'MANUAL'
    assert gate.output(0.02) == (0.0, 0.0)
    joy(gate, 0.03)
    joy(gate, 0.15, rb=True)
    assert gate.mode == 'AUTO'
    assert gate.output(0.26) == (0.0, 0.0)
    assert gate.reason == 'auto_waiting_command'


def test_mode_transition_holds_zero_before_using_fresh_command():
    gate = ControlGate()
    joy(gate, 0.0)
    joy(gate, 0.12, rb=True)
    gate.receive_auto_command(twist(), 0.13)
    assert gate.output(0.15) == (0.0, 0.0)
    assert gate.output(0.23) == (0.1, -0.2)


def test_auto_ignores_lb_and_even_invalid_manual_commands():
    gate = auto()
    joy(gate, 0.24, lb=True, rb=True)
    gate.receive_command(twist(-0.2, 0.4), 0.25)
    gate.receive_command(twist(float('nan')), 0.26)
    joy(gate, 0.27, lb=False, rb=True)
    assert gate.output(0.28) == (0.1, -0.2)
    assert gate.mode == 'AUTO'


def test_held_rb_and_brief_release_bounce_do_not_retoggle():
    gate = auto()
    for now in (0.24, 0.28, 0.32):
        joy(gate, now, rb=True)
    joy(gate, 0.34, rb=False)
    joy(gate, 0.38, rb=True)
    joy(gate, 0.50, rb=True)
    assert gate.mode == 'AUTO'
    joy(gate, 0.52, rb=False)
    joy(gate, 0.58, rb=False)
    joy(gate, 0.64, rb=True)
    assert gate.mode == 'MANUAL'


def test_auto_exit_with_lb_held_stops_until_release_press_and_new_command():
    gate = auto()
    joy(gate, 0.24, lb=True)
    gate.receive_command(twist(-0.2, 0.4), 0.25)
    joy(gate, 0.36, lb=True, rb=True)
    assert gate.output(0.36) == (0.0, 0.0)
    joy(gate, 0.48, lb=True, rb=True)
    gate.receive_command(twist(), 0.49)
    assert gate.output(0.50) == (0.0, 0.0)
    joy(gate, 0.51, lb=False, rb=True)
    joy(gate, 0.52, lb=True, rb=True)
    assert gate.output(0.53) == (0.0, 0.0)
    gate.receive_command(twist(-0.1, 0.3), 0.54)
    assert gate.output(0.55) == (-0.1, 0.3)


def test_auto_timeout_stops_without_manual_fallback_and_fresh_auto_resumes():
    gate = auto()
    joy(gate, 0.4, lb=True, rb=True)
    gate.receive_command(twist(-0.2, 0.4), 0.6)
    assert gate.output(0.65) == (0.0, 0.0)
    assert gate.mode == 'AUTO'
    assert gate.reason == 'auto_command_timeout'
    gate.receive_auto_command(twist(0.05, 0.1), 0.66)
    assert gate.output(0.67) == (0.05, 0.1)


@pytest.mark.parametrize('callback', ['timer', 'joy', 'auto', 'manual'])
def test_joy_loss_disables_auto_even_before_timer_and_requires_new_rb_press(callback):
    gate = auto()
    if callback == 'timer':
        gate.output(0.63)
    elif callback == 'joy':
        joy(gate, 0.63, lb=True, rb=True)
    elif callback == 'auto':
        gate.receive_auto_command(twist(), 0.63)
    else:
        gate.receive_command(twist(), 0.63)
    assert gate.mode == 'MANUAL'
    joy(gate, 0.64, lb=True, rb=True)
    gate.receive_auto_command(twist(), 0.65)
    gate.receive_command(twist(), 0.66)
    assert gate.output(0.67) == (0.0, 0.0)
    joy(gate, 0.68)
    joy(gate, 0.80, rb=True)
    assert gate.mode == 'AUTO'
    assert gate.output(0.91) == (0.0, 0.0)


@pytest.mark.parametrize('values', [twist(float('nan')), twist(w=float('inf')),
                                    (0.1, 0.1, 0.0, 0.0, 0.0, 0.0), ()])
def test_invalid_auto_command_stops_and_valid_command_recovers(values):
    gate = auto()
    gate.receive_auto_command(values, 0.24)
    assert gate.output(0.25) == (0.0, 0.0)
    assert gate.reason == 'auto_invalid_command'
    gate.receive_auto_command(twist(), 0.26)
    assert gate.output(0.27) == (0.1, -0.2)


@pytest.mark.parametrize('axes,buttons', [([0.0] * 8, [0] * 7),
                                         ([float('nan')] * 8, [0] * 8),
                                         ([0.0] * 8, [0] * 7 + [2])])
def test_invalid_joy_disables_auto(axes, buttons):
    gate = auto()
    gate.receive_joy(axes, buttons, 0.24)
    assert gate.output(0.25) == (0.0, 0.0)
    assert gate.mode == 'MANUAL'
    assert gate.reason == 'invalid_joy'


def test_auto_limits_and_inactive_auto_cannot_affect_manual():
    gate = auto()
    gate.receive_auto_command(twist(10.0, -10.0), 0.24)
    assert gate.output(0.25) == (0.2, -0.4)
    gate = ControlGate()
    joy(gate, 0.0)
    joy(gate, 0.01, lb=True)
    gate.receive_command(twist(), 0.02)
    gate.receive_auto_command(twist(float('nan')), 0.03)
    assert gate.output(0.04) == (0.1, -0.2)


def test_manual_release_bounce_and_command_delay_still_recover():
    gate = ControlGate()
    joy(gate, 0.0)
    joy(gate, 0.01, lb=True)
    gate.receive_command(twist(), 0.02)
    joy(gate, 0.03)
    assert gate.output(0.06) == (0.1, -0.2)
    joy(gate, 0.07, lb=True)
    joy(gate, 0.40, lb=True)
    assert gate.output(0.53) == (0.0, 0.0)
    gate.receive_command(twist(), 0.54)
    assert gate.output(0.55) == (0.1, -0.2)


@pytest.mark.parametrize('kwargs', [
    {'auto_toggle_button': 6}, {'auto_toggle_button': -1}, {'auto_toggle_button': True},
    {'auto_timeout': 0.0}, {'auto_timeout': float('nan')},
    {'toggle_release_debounce': -0.1}, {'toggle_release_debounce': 0.5},
    {'mode_stop_duration': float('inf')},
])
def test_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        ControlGate(**kwargs)
