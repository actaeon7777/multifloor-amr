"""Mode selection, RB toggle and command expiry, independent of ROS."""

from math import isfinite

from amr_control.manual_gate import ManualGate
from amr_control.wheel_limiter import WheelSpeedLimiter


class ControlGate:
    def __init__(self, auto_toggle_button=7, auto_timeout=0.50,
                 toggle_release_debounce=0.10, mode_stop_duration=0.10,
                 wheel_diameter=0.172, track_width=0.499, max_wheel_rev_s=1.8,
                 **manual_options):
        self.manual = ManualGate(**manual_options)
        self.wheel_limiter = WheelSpeedLimiter(wheel_diameter, track_width, max_wheel_rev_s)
        if (type(auto_toggle_button) is not int or auto_toggle_button < 0
                or auto_toggle_button == self.manual.deadman_button):
            raise ValueError('auto_toggle_button must be a distinct non-negative integer')
        if not isfinite(auto_timeout) or auto_timeout <= 0:
            raise ValueError('auto_timeout must be finite and positive')
        for value in (toggle_release_debounce, mode_stop_duration):
            if not isfinite(value) or value < 0 or value >= self.manual.joy_timeout:
                raise ValueError('debounce/transition time must be non-negative and < joy_timeout')
        self.auto_toggle_button = auto_toggle_button
        self.auto_timeout = auto_timeout
        self.toggle_release_debounce = toggle_release_debounce
        self.mode_stop_duration = mode_stop_duration
        self.joy_time = None
        self._reset('awaiting_release')

    @property
    def mode(self):
        return 'AUTO' if self.auto_enabled else 'MANUAL'

    @property
    def reason(self):
        return self.auto_reason if self.auto_enabled else self.manual.reason

    def _reset(self, reason):
        self.auto_enabled = False
        self.auto_command = None
        self.auto_time = None
        self.auto_reason = 'auto_waiting_command'
        self.rb_ready = False
        self.rb_release_since = None
        self.stop_until = 0.0
        self.manual.stop(reason)
        self.manual.joy_time = None

    def _expire(self, now):
        if self.joy_time is not None and now - self.joy_time >= self.manual.joy_timeout:
            self.joy_time = None
            self._reset('joy_timeout')
        if (self.auto_enabled and self.auto_time is not None
                and now - self.auto_time >= self.auto_timeout):
            self.auto_command = None
            self.auto_time = None
            self.auto_reason = 'auto_command_timeout'

    def receive_joy(self, axes, buttons, now):
        self._expire(now)
        if (len(buttons) <= max(self.auto_toggle_button, self.manual.deadman_button)
                or len(axes) <= max(self.manual.required_axes)
                or any(button not in (0, 1) for button in buttons)
                or any(not isfinite(axis) or abs(axis) > 1.001 for axis in axes)):
            self.joy_time = None
            self._reset('invalid_joy')
            return
        self.joy_time = now
        toggled = False
        if buttons[self.auto_toggle_button] == 0:
            if self.rb_release_since is None:
                self.rb_release_since = now
            if now - self.rb_release_since >= self.toggle_release_debounce:
                self.rb_ready = True
        else:
            # Also allow a press following a sufficiently long observed release
            # without depending on an exact autorepeat sample at the boundary.
            if (self.rb_release_since is not None
                    and now - self.rb_release_since >= self.toggle_release_debounce):
                self.rb_ready = True
            self.rb_release_since = None
            if self.rb_ready:
                self.rb_ready = False
                self.auto_enabled = not self.auto_enabled
                self.auto_command = None
                self.auto_time = None
                self.auto_reason = 'auto_waiting_command'
                self.stop_until = now + self.mode_stop_duration
                self.manual.stop('awaiting_release')
                self.manual.joy_time = None
                toggled = True
        # Never let the manual gate's release/invalid-command path stop AUTO.
        # On AUTO exit, require a subsequent LB release before manual can arm.
        if not self.auto_enabled and not toggled:
            self.manual.receive_joy(axes, buttons, now)

    def receive_command(self, values, now):
        self._expire(now)
        if not self.auto_enabled:
            self.manual.receive_command(values, now)

    def receive_auto_command(self, values, now):
        self._expire(now)
        if not self.auto_enabled:
            return  # Do not cache commands received before AUTO entry.
        if (len(values) != 6 or any(not isfinite(value) for value in values)
                or any(values[index] != 0.0 for index in (1, 2, 3, 4))):
            self.auto_command = None
            self.auto_time = None
            self.auto_reason = 'auto_invalid_command'
            return
        self.auto_command = (
            max(-self.manual.max_linear_speed, min(self.manual.max_linear_speed, values[0])),
            max(-self.manual.max_angular_speed, min(self.manual.max_angular_speed, values[5])),
        )
        self.auto_time = now
        self.auto_reason = 'auto_active'

    def output(self, now):
        self._expire(now)
        if now < self.stop_until:
            return (0.0, 0.0)
        if self.auto_enabled:
            command = self.auto_command if self.auto_command is not None else (0.0, 0.0)
        else:
            command = self.manual.output(now)
        return self.wheel_limiter.limit(*command)
