"""Joystick restart approval gesture, without ROS."""

import pytest

from amr_control.approve_gesture import ApproveGesture


APPROVE, LB = 11, 6


def joy(approve=0, lb=0, linear=0.0, angular=0.0):
    axes = [0.0] * 6
    axes[1], axes[2] = linear, angular
    buttons = [0] * 15
    buttons[APPROVE], buttons[LB] = approve, lb
    return axes, buttons


def hold(gesture, start, end, step=0.05, required=True, **kwargs):
    fired = []
    for index in range(round((end - start) / step) + 1):
        t = round(start + index * step, 6)
        if gesture.receive_joy(*joy(approve=1, **kwargs), t, required):
            fired.append(t)
    return fired


def test_long_press_fires_once_per_press():
    gesture = ApproveGesture(APPROVE, LB, hold=2.0)
    assert hold(gesture, 0.0, 3.0) == [2.0]
    assert not gesture.receive_joy(*joy(), 3.1, True)  # Release re-enables.
    assert hold(gesture, 4.0, 6.5) == [6.0]


def test_short_press_does_nothing():
    gesture = ApproveGesture(APPROVE, LB, hold=2.0)
    assert hold(gesture, 0.0, 1.5) == []
    assert not gesture.receive_joy(*joy(), 1.6, True)
    assert hold(gesture, 1.7, 3.0) == []


@pytest.mark.parametrize('kwargs', [{'lb': 1}, {'linear': 0.5}, {'angular': -0.3},
                                    {'required': False}])
def test_press_spoiled_by_lb_stick_or_no_approval_needed(kwargs):
    gesture = ApproveGesture(APPROVE, LB, hold=0.5)
    required = kwargs.pop('required', True)
    assert hold(gesture, 0.0, 0.2, required=required, **kwargs) == []
    # Conditions clear while the button stays held: still spoiled until release.
    assert hold(gesture, 0.25, 2.0) == []
    gesture.receive_joy(*joy(), 2.1, True)
    assert hold(gesture, 2.2, 3.0) == [2.7]


def test_button_held_through_estop_does_not_approve_on_release():
    gesture = ApproveGesture(APPROVE, LB, hold=0.5)
    assert hold(gesture, 0.0, 1.0, required=False) == []  # Held during ESTOP.
    assert hold(gesture, 1.05, 3.0, required=True) == []  # DISARMED arrives while held.


def test_invalid_joy_resets_and_bad_settings_are_rejected():
    gesture = ApproveGesture(APPROVE, LB, hold=0.5)
    hold(gesture, 0.0, 0.4)
    axes, buttons = joy(approve=1)
    assert not gesture.receive_joy(axes, buttons[:5], 0.45, True)
    assert gesture.pressed_since is None
    axes[1] = float('nan')
    assert not gesture.receive_joy(axes, buttons, 0.5, True)
    with pytest.raises(ValueError):
        ApproveGesture(LB, LB)
    with pytest.raises(ValueError):
        ApproveGesture(-1, LB)
    with pytest.raises(ValueError):
        ApproveGesture(APPROVE, LB, hold=0.0)
