"""STM32 drive protocol from the working keyboard sender: standard 0x201, <ff>."""

from math import isfinite
import struct


DRIVE_CAN_ID = 0x201
STOP_PAYLOAD = bytes(8)
FLOAT32_MAX = 3.4028234663852886e38


def make_payload(v_mps, w_radps):
    if any(not isfinite(value) or abs(value) > FLOAT32_MAX for value in (v_mps, w_radps)):
        raise ValueError('CAN velocities must be finite float32 values')
    return struct.pack('<ff', v_mps, w_radps)


class CommandWatchdog:
    """Timeouts pause output; startup and invalid input/fault require a fresh zero."""

    def __init__(self, timeout=0.50, max_linear_speed=0.20, max_angular_speed=0.40):
        if not isfinite(timeout) or timeout <= 0:
            raise ValueError('command_timeout must be finite and positive')
        for limit in (max_linear_speed, max_angular_speed):
            if not isfinite(limit) or not 0 < limit <= FLOAT32_MAX:
                raise ValueError('speed limits must be finite positive float32 values')
        self.timeout = timeout
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.stop('awaiting_zero')

    def stop(self, reason, requires_zero=True):
        self.command = (0.0, 0.0)
        self.received_at = None
        self.requires_zero = requires_zero
        self.reason = reason

    def _expire(self, now):
        if self.received_at is not None and now - self.received_at >= self.timeout:
            self.stop('command_timeout', requires_zero=False)

    def receive(self, values, now):
        self._expire(now)
        if len(values) != 6 or any(not isfinite(value) for value in values):
            self.stop('invalid_command')
            return
        if any(values[index] != 0.0 for index in (1, 2, 3, 4)):
            self.stop('nonplanar_command')
            return
        v, w = values[0], values[5]
        if self.requires_zero and (v != 0.0 or w != 0.0):
            return
        self.requires_zero = False
        self.received_at = now
        self.command = (
            max(-self.max_linear_speed, min(self.max_linear_speed, v)),
            max(-self.max_angular_speed, min(self.max_angular_speed, w)),
        )
        self.reason = 'command'

    def sample(self, now):
        self._expire(now)
        return self.command
