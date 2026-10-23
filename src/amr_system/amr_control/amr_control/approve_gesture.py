"""Joystick E-stop restart approval: one deliberate long press, independent of ROS.

The approve button must be pressed while approval is required, LB is up and the
drive sticks are centred, then held for ``hold`` seconds. A press that starts
or continues without those conditions is spoiled until the button is released,
so a button held through an E-stop never approves by itself.
"""

from math import isfinite


class ApproveGesture:
    def __init__(self, button, deadman_button=6, hold=2.0, axes=(1, 2), neutral=0.10):
        if type(button) is not int or button < 0:
            raise ValueError('approve_button must be a non-negative button index')
        if button == deadman_button:
            raise ValueError('approve_button must differ from the LB deadman button')
        if not isfinite(hold) or hold <= 0:
            raise ValueError('approve_hold must be finite and positive')
        if not isfinite(neutral) or not 0 < neutral < 1:
            raise ValueError('approve_neutral must be in (0, 1)')
        self.button = button
        self.deadman_button = deadman_button
        self.hold = hold
        self.axes = tuple(axes)
        self.neutral = neutral
        self.reset()

    def reset(self):
        self.pressed_since = None
        self.spoiled = False

    def receive_joy(self, axes, buttons, now, approval_required):
        """Return True exactly once when a valid hold completes."""
        if (len(buttons) <= max(self.button, self.deadman_button)
                or len(axes) <= max(self.axes)
                or any(button not in (0, 1) for button in buttons)
                or any(not isfinite(axis) for axis in axes)):
            self.reset()
            return False
        if buttons[self.button] == 0:
            self.reset()
            return False
        if self.spoiled:
            return False
        clear = (approval_required and buttons[self.deadman_button] == 0
                 and all(abs(axes[index]) <= self.neutral for index in self.axes))
        if not clear:
            self.spoiled = True
            self.pressed_since = None
            return False
        if self.pressed_since is None:
            self.pressed_since = now
        if now - self.pressed_since >= self.hold:
            self.spoiled = True  # One request per press.
            return True
        return False
