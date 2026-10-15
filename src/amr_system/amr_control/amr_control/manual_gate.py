"""Manual deadman state with release debounce and monotonic reception times."""

from math import isfinite


class ManualGate:
    def __init__(self, deadman_button=6, required_axes=(1, 2), joy_timeout=0.50,
                 command_timeout=0.50, release_debounce=0.10,
                 max_linear_speed=0.20, max_angular_speed=0.40):
        if type(deadman_button) is not int or deadman_button < 0:
            raise ValueError('deadman_button must be a non-negative integer')
        if not required_axes or any(type(axis) is not int or axis < 0 for axis in required_axes):
            raise ValueError('required_axes must contain non-negative integers')
        for value in (joy_timeout, command_timeout, max_linear_speed, max_angular_speed):
            if not isfinite(value) or value <= 0:
                raise ValueError('timeouts and speed limits must be finite and positive')
        if (not isfinite(release_debounce) or release_debounce < 0
                or release_debounce >= min(joy_timeout, command_timeout)):
            raise ValueError('release_debounce must be non-negative and shorter than both timeouts')
        self.deadman_button = deadman_button
        self.required_axes = tuple(required_axes)
        self.joy_timeout = joy_timeout
        self.command_timeout = command_timeout
        self.release_debounce = release_debounce
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.joy_time = None
        self.stop('awaiting_release')

    def stop(self, reason='awaiting_release'):
        """Latch startup, invalid input or loss of Joy until release/press."""
        self.armed = False
        self.ready_to_arm = False
        self.armed_time = None
        self.command_time = None
        self.command = None
        self.release_since = None
        self.reason = reason

    def _confirm_release(self):
        self.stop('deadman_released')
        self.ready_to_arm = True

    def _expire(self, now):
        if self.joy_time is not None and now - self.joy_time >= self.joy_timeout:
            self.joy_time = None
            self.stop('joy_timeout')
        elif self.armed:
            if self.release_since is not None and now - self.release_since >= self.release_debounce:
                self._confirm_release()
                return
            reference = self.command_time if self.command_time is not None else self.armed_time
            if now - reference >= self.command_timeout:
                # Pause output, but do not latch a brief loss of the separate
                # joy_teleop topic when valid held-LB Joy messages still arrive.
                self.command = None
                self.command_time = None
                self.reason = 'command_timeout'

    def receive_joy(self, axes, buttons, now):
        self._expire(now)
        if (len(buttons) <= self.deadman_button or len(axes) <= max(self.required_axes)
                or any(button not in (0, 1) for button in buttons)
                or any(not isfinite(axis) or abs(axis) > 1.001 for axis in axes)):
            self.joy_time = None
            self.stop('invalid_joy')
            return
        self.joy_time = now
        if buttons[self.deadman_button] == 0:
            if self.armed:
                if self.release_since is None:
                    self.release_since = now
                self.reason = 'release_pending'
                self._expire(now)
            else:
                self._confirm_release()
        elif self.armed:
            self.release_since = None
            if self.command is not None:
                self.reason = 'active'
        elif self.ready_to_arm:
            self.ready_to_arm = False
            self.armed = True
            self.armed_time = now
            self.reason = 'waiting_command'

    def receive_command(self, values, now):
        """Accept only fresh planar Twist values in x,y,z,roll,pitch,yaw order."""
        self._expire(now)
        if len(values) != 6 or any(not isfinite(value) for value in values):
            self.stop('invalid_command')
            return
        if any(values[index] != 0.0 for index in (1, 2, 3, 4)):
            self.stop('invalid_command')
            return
        if not self.armed or self.release_since is not None:
            return
        self.command = (
            max(-self.max_linear_speed, min(self.max_linear_speed, values[0])),
            max(-self.max_angular_speed, min(self.max_angular_speed, values[5])),
        )
        self.command_time = now
        self.reason = 'active'

    def output(self, now):
        self._expire(now)
        return self.command if self.armed and self.command is not None else (0.0, 0.0)
