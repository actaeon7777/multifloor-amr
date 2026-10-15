"""Deadman regression tests without ROS or physical hardware."""

import pytest

from amr_control.manual_gate import ManualGate


def joy(gate, held, now):
    gate.receive_joy([0.0] * 8, [0] * 6 + [int(held)] + [0] * 8, now)


def command(gate, now, linear=0.2, angular=0.4):
    gate.receive_command((linear, 0.0, 0.0, 0.0, 0.0, angular), now)


def moving():
    gate = ManualGate()
    joy(gate, False, 0.0)
    joy(gate, True, 0.01)
    command(gate, 0.02)
    assert gate.output(0.03) == (0.2, 0.4)
    return gate


def test_startup_requires_release_even_with_command_and_lb_held():
    gate = ManualGate()
    joy(gate, True, 0.0)
    command(gate, 0.01)
    assert gate.output(0.02) == (0.0, 0.0)


def test_confirmed_lb_release_stops_with_other_button_and_discards_old_command():
    gate = moving()
    gate.receive_joy([0.0] * 8, [1] + [0] * 14, 0.04)
    assert gate.output(0.09) == (0.2, 0.4)
    assert gate.output(0.15) == (0.0, 0.0)
    command(gate, 0.16)
    joy(gate, True, 0.17)
    assert gate.output(0.18) == (0.0, 0.0)
    command(gate, 0.19, linear=-0.1, angular=-0.3)
    assert gate.output(0.20) == (-0.1, -0.3)


def test_brief_release_does_not_interrupt_motion():
    gate = moving()
    joy(gate, False, 0.04)
    assert gate.output(0.08) == (0.2, 0.4)
    joy(gate, True, 0.09)
    command(gate, 0.10, linear=-0.1, angular=0.1)
    assert gate.output(0.11) == (-0.1, 0.1)


def test_repeated_release_samples_do_not_reset_debounce_timer():
    gate = moving()
    joy(gate, False, 0.04)
    joy(gate, False, 0.08)
    joy(gate, False, 0.12)
    assert gate.output(0.15) == (0.0, 0.0)


def test_commands_during_release_cannot_extend_motion():
    gate = moving()
    joy(gate, False, 0.04)
    command(gate, 0.10, linear=-0.2)
    assert gate.output(0.11) == (0.2, 0.4)
    assert gate.output(0.15) == (0.0, 0.0)


def test_zero_debounce_preserves_immediate_release():
    gate = ManualGate(release_debounce=0.0)
    joy(gate, False, 0.0)
    joy(gate, True, 0.01)
    command(gate, 0.02)
    joy(gate, False, 0.04)
    assert gate.output(0.04) == (0.0, 0.0)


def test_270_ms_delay_no_longer_latches_zero():
    gate = moving()
    joy(gate, True, 0.20)
    joy(gate, True, 0.28)
    assert gate.output(0.28) == (0.2, 0.4)
    command(gate, 0.29, linear=0.1, angular=0.2)
    assert gate.output(0.30) == (0.1, 0.2)


def test_joy_timeout_even_if_manual_commands_keep_arriving():
    gate = moving()
    command(gate, 0.40)
    assert gate.output(0.52) == (0.0, 0.0)
    assert gate.reason == 'joy_timeout'
    joy(gate, True, 0.53)
    command(gate, 0.54)
    assert gate.output(0.55) == (0.0, 0.0)
    joy(gate, False, 0.56)
    joy(gate, True, 0.57)
    command(gate, 0.58)
    assert gate.output(0.59) == (0.2, 0.4)


def test_command_timeout_pauses_and_fresh_command_resumes_with_lb_held():
    gate = moving()
    joy(gate, True, 0.40)
    assert gate.output(0.53) == (0.0, 0.0)
    assert gate.reason == 'command_timeout'
    joy(gate, True, 0.54)
    command(gate, 0.55)
    assert gate.output(0.56) == (0.2, 0.4)


def test_late_command_after_timeout_recovers_without_a_timer_callback():
    gate = moving()
    joy(gate, True, 0.40)
    command(gate, 0.55, linear=0.1, angular=0.0)
    assert gate.output(0.56) == (0.1, 0.0)


def test_late_joy_cannot_bypass_timeout_before_timer_runs():
    gate = moving()
    command(gate, 0.40)
    joy(gate, True, 0.52)
    command(gate, 0.53)
    assert gate.output(0.54) == (0.0, 0.0)


def test_stale_release_does_not_allow_rearming():
    gate = ManualGate()
    joy(gate, False, 0.0)
    joy(gate, True, 0.51)
    command(gate, 0.52)
    assert gate.output(0.53) == (0.0, 0.0)


@pytest.mark.parametrize('axes,buttons', [
    ([], [0] * 6 + [1]),
    ([0.0] * 8, []),
    ([0.0, float('nan'), 0.0], [0] * 6 + [1]),
    ([0.0, float('inf'), 0.0], [0] * 6 + [1]),
    ([0.0, 2.0, 0.0], [0] * 6 + [1]),
    ([0.0] * 8, [0] * 6 + [2]),
])
def test_malformed_joy_stops(axes, buttons):
    gate = moving()
    gate.receive_joy(axes, buttons, 0.04)
    assert gate.output(0.05) == (0.0, 0.0)


@pytest.mark.parametrize('values', [
    (float('nan'), 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 0.0, 0.0, float('inf')),
    (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
])
def test_invalid_manual_command_stops(values):
    gate = moving()
    gate.receive_command(values, 0.04)
    assert gate.output(0.05) == (0.0, 0.0)


def test_limits_and_neutral():
    gate = moving()
    command(gate, 0.04, linear=3.0, angular=-3.0)
    assert gate.output(0.05) == (0.2, -0.4)
    command(gate, 0.06, linear=0.0, angular=0.0)
    assert gate.output(0.07) == (0.0, 0.0)


@pytest.mark.parametrize('kwargs', [
    {'deadman_button': -1}, {'required_axes': []}, {'required_axes': [-1]},
    {'joy_timeout': 0.0}, {'command_timeout': float('nan')},
    {'max_linear_speed': -1.0}, {'max_angular_speed': float('inf')},
    {'release_debounce': -0.1}, {'release_debounce': float('nan')},
    {'release_debounce': 0.5},
])
def test_bad_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        ManualGate(**kwargs)
