"""ROS/Isaac independent command transport model. No body-state integration."""
from collections import deque
from math import copysign, isfinite, pi


class PhysXWheelAngle:
    """Unwrap PhysX's [-2pi, 2pi] wheel position for the simulated encoder only.

    Only a plausible 4pi boundary crossing within the encoder timeout is fixed.
    Other jumps are passed through so WheelOdometry still detects resets/gaps.
    Raw JointState positions remain available separately for diagnostics.
    """
    def __init__(self, max_rev_s=5., timeout=.25):
        if not all(isfinite(v) and v > 0 for v in (max_rev_s, timeout)):
            raise ValueError('Invalid wheel angle limits')
        self.max_rate = max_rev_s * 2*pi
        self.timeout = timeout
        self.previous = None
        self.continuous = 0.
        self.wraps = 0

    def update(self, position, stamp):
        if not all(isfinite(v) for v in (position, stamp)):
            raise ValueError('Invalid simulated wheel position')
        if self.previous is None:
            self.continuous = position
        else:
            old_position, old_stamp = self.previous
            elapsed = stamp - old_stamp
            if elapsed <= 0:
                # Let WheelOdometry reject stale timestamps without corrupting
                # the angle baseline for the next fresh sample.
                return self.continuous
            delta = position - old_position
            if (elapsed <= self.timeout and abs(delta) > 2*pi
                    and max(abs(position), abs(old_position)) <= 2*pi + 1e-5):
                candidate = delta - copysign(4*pi, delta)
                # Same motion bound and encoder-noise margin as WheelOdometry.
                if abs(candidate) <= self.max_rate*elapsed + .02*2*pi:
                    delta = candidate
                    self.wraps += 1
            self.continuous += delta
        self.previous = (position, stamp)
        return self.continuous


class CommandTransport:
    def __init__(self, period=.05, timeout=.3, delay=0.):
        if not all(isfinite(v) for v in (period, timeout, delay)) or not 0 < period < timeout or delay < 0:
            raise ValueError('Invalid command timing')
        self.period, self.timeout, self.delay = period, timeout, delay
        self.reset()

    def reset(self):
        self.pending = deque()
        self.received = None
        self.next_update = 0.
        self.output = (0., 0.)

    def receive(self, linear, angular, sim_time, wall_time):
        if not all(isfinite(v) for v in (linear, angular, sim_time, wall_time)):
            self.reset()
            return False
        # Bound memory while stopped/overloaded; latest queued command wins.
        self.pending.append((sim_time + self.delay, wall_time, linear, angular))
        while len(self.pending) > 100:
            self.pending.popleft()
        return True

    def step(self, sim_time, wall_time):
        newest = None
        while self.pending and self.pending[0][0] <= sim_time:
            newest = self.pending.popleft()
        if newest is not None:
            _, self.received, linear, angular = newest
            self.latest = (linear, angular)
        if self.received is None or wall_time - self.received >= self.timeout:
            self.output = (0., 0.)
        elif sim_time + 1e-6 >= self.next_update:
            self.output = self.latest
        if sim_time + 1e-6 >= self.next_update:
            self.next_update = (int((sim_time + 1e-6) / self.period) + 1) * self.period
        return self.output
