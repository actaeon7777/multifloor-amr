"""Limit body velocity before CAN transmission, preserving the requested curvature."""

from math import isfinite, pi


class WheelSpeedLimiter:
    def __init__(self, wheel_diameter=0.172, track_width=0.499, max_wheel_rev_s=1.8):
        for value in (wheel_diameter, track_width, max_wheel_rev_s):
            if not isfinite(value) or value <= 0:
                raise ValueError('wheel geometry and speed limit must be finite and positive')
        self.max_wheel_speed = pi * wheel_diameter * max_wheel_rev_s
        self.half_track = track_width / 2.0
        if (not isfinite(self.max_wheel_speed) or self.max_wheel_speed <= 0
                or self.half_track <= 0):
            raise ValueError('derived wheel limits must be finite and positive')

    def limit(self, linear, angular):
        if not isfinite(linear) or not isfinite(angular):
            raise ValueError('body velocity must be finite')
        # max(|v + w*track/2|, |v - w*track/2|) = |v| + |w|*track/2.
        # Scale both components together to preserve wheel ratio and curvature.
        # This is a speed limit only; ODrive owns acceleration/deceleration.
        required_speed = abs(linear) + abs(angular) * self.half_track
        if required_speed <= self.max_wheel_speed:
            return linear, angular
        scale = self.max_wheel_speed / required_speed
        return linear * scale, angular * scale
