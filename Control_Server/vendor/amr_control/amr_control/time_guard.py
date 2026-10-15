"""Clock discontinuities and heartbeat ordering, independent of ROS and hardware."""
import json
import math
import time


class ClockMonitor:
    def __init__(self, jump_limit=.5):
        if not math.isfinite(jump_limit) or jump_limit <= 0:
            raise ValueError('jump_limit must be positive and finite')
        self.limit = jump_limit
        self.previous = None

    def observe(self, ros, steady, simulated=False):
        previous, self.previous = self.previous, (ros, steady)
        if previous is None:
            return False
        dr, ds = ros-previous[0], steady-previous[1]
        # Pausing and changing simulation speed are normal. Real clocks should
        # advance with monotonic time; both forward and backward steps matter.
        return dr < -1e-6 if simulated else abs(dr-ds) > self.limit


class SessionOrder:
    def __init__(self):
        self.session = None
        self.generation = -1
        self.stamp = -math.inf
        self.retired = set()

    def accept(self, session, generation, stamp):
        if (not isinstance(session, str) or not session or session in self.retired
                or type(generation) is not int or generation < 0
                or type(stamp) not in (int, float) or not math.isfinite(stamp)):
            return False
        if session == self.session:
            if generation < self.generation or (generation == self.generation and stamp <= self.stamp):
                return False
        else:
            if self.session is not None:
                self.retired.add(self.session)
            self.session = session
        self.generation, self.stamp = generation, stamp
        return True


class HealthLease:
    """A retained healthy message cannot grant an unlimited command lease."""
    def __init__(self, timeout=.6):
        self.timeout = timeout
        self.order = SessionOrder()
        self.received = -math.inf
        self.status = {}
        self.identity = None

    def receive(self, text, now=None, wall=None):
        now = time.monotonic() if now is None else now
        wall = time.time() if wall is None else wall
        try:
            status = json.loads(text)
            if (type(status.get('healthy')) is not bool
                    or type(status.get('latched')) is not bool
                    or (status['healthy'] and status['latched'])
                    or type(status.get('epoch')) is not int or status['epoch'] < 0
                    or type(status.get('stamp')) not in (int, float)
                    or not math.isfinite(status['stamp'])
                    or not -0.1 <= wall-status['stamp'] <= self.timeout):
                raise ValueError('invalid or expired health')
            if not self.order.accept(status['session'], status['sequence'], status['stamp']):
                return False
            self.status, self.received = status, now
            self.identity = (status['session'], status['epoch'])
            return True
        except (ValueError, TypeError, KeyError, AttributeError):
            self.status = {}
            return False

    def healthy(self, now=None):
        now = time.monotonic() if now is None else now
        return self.status.get('healthy') is True and 0 <= now-self.received < self.timeout
