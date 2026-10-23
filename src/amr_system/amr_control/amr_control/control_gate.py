"""LB-takeover mode selection and command expiry, independent of ROS.

AUTO is the default, whether or not a joystick is connected. A valid Joy sample
with LB held takes over at once (MANUAL) and discards autonomous commands. AUTO
returns manual_holdoff after the last LB sample; Joy connection or loss alone
never changes the mode.
"""

from math import isfinite

from amr_control.manual_gate import ManualGate
from amr_control.wheel_limiter import WheelSpeedLimiter


class ControlGate:
    def __init__(self, auto_timeout=0.50, manual_holdoff=0.50, mode_stop_duration=0.10,
                 wheel_diameter=0.172, track_width=0.529, max_wheel_rev_s=1.8,
                 **manual_options):
        self.manual = ManualGate(**manual_options)
        self.wheel_limiter = WheelSpeedLimiter(wheel_diameter, track_width, max_wheel_rev_s)
        if not isfinite(auto_timeout) or auto_timeout <= 0:
            raise ValueError('auto_timeout must be finite and positive')
        if not isfinite(manual_holdoff) or manual_holdoff < self.manual.joy_timeout:
            raise ValueError('manual_holdoff must be finite and >= joy_timeout')
        if (not isfinite(mode_stop_duration) or mode_stop_duration < 0
                or mode_stop_duration >= self.manual.joy_timeout):
            raise ValueError('mode_stop_duration must be non-negative and < joy_timeout')
        self.auto_timeout = auto_timeout
        self.manual_holdoff = manual_holdoff
        self.mode_stop_duration = mode_stop_duration
        self.lb_time = None
        self.auto_enabled = True
        self.auto_command = None
        self.auto_time = None
        self.auto_reason = 'auto_waiting_command'
        self.stop_until = 0.0
        self.manual.stop('awaiting_release')

    @property
    def mode(self):
        return 'AUTO' if self.auto_enabled else 'MANUAL'

    @property
    def reason(self):
        return self.auto_reason if self.auto_enabled else self.manual.reason

    def _switch(self, auto, now):
        # Leave the manual gate untouched: an LB press after an observed
        # release both takes over and arms manual input.
        self.auto_enabled = auto
        self.auto_command = None
        self.auto_time = None
        self.auto_reason = 'auto_waiting_command'
        self.stop_until = now + self.mode_stop_duration

    def _expire(self, now):
        if not self.auto_enabled:
            if self.lb_time is None or now - self.lb_time >= self.manual_holdoff:
                self.lb_time = None
                self._switch(True, now)
        elif self.auto_time is not None and now - self.auto_time >= self.auto_timeout:
            self.auto_command = None
            self.auto_time = None
            self.auto_reason = 'auto_command_timeout'

    def receive_joy(self, axes, buttons, now):
        self._expire(now)
        # Track LB release/arming in AUTO too; manual commands still need MANUAL.
        self.manual.receive_joy(axes, buttons, now)
        if self.manual.joy_time != now or buttons[self.manual.deadman_button] != 1:
            return  # Invalid Joy or LB up: no takeover.
        self.lb_time = now
        if self.auto_enabled:
            self._switch(False, now)

    def receive_command(self, values, now):
        self._expire(now)
        if not self.auto_enabled:
            self.manual.receive_command(values, now)

    def receive_auto_command(self, values, now):
        self._expire(now)
        if not self.auto_enabled:
            return  # Do not cache commands received during a manual takeover.
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
