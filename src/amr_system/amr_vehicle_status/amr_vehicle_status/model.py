"""Source freshness, sample ordering and odom continuity without ROS or motor IO."""
from collections import deque
from copy import deepcopy
import math

TIMEOUTS = dict(scan=1., wheel=.75, imu=.75, ekf=.75, joints=.75, command=.75,
                diagnostics=3., pose=2., localization=2., time=.6, estop=.6)
for _side in ('left', 'right'):
    TIMEOUTS[_side+'_power'] = 2.5  # Bus voltage broadcasts at 1 Hz.
    TIMEOUTS[_side+'_heartbeat'] = .75


class StatusModel:
    def __init__(self):
        self.samples = {}
        self.history = {}
        self.stamps = {}
        self.last_wheel = None
        self.distance = 0.
        self.distance_epoch = 0

    def reset(self):
        self.samples.clear()
        self.history.clear()
        self.stamps.clear()
        self.last_wheel = None
        self.distance = 0.
        self.distance_epoch += 1

    def reject(self, name, now):
        self.samples[name] = dict(values={}, received_at=now, valid=False, stamp=None, base_age=0.)

    def observe(self, name, values, now, stamp=None, base_age=0.):
        if stamp is not None:
            if not math.isfinite(stamp):
                self.reject(name, now)
                return False
            if stamp <= self.stamps.get(name, -math.inf):
                return False
            self.stamps[name] = stamp
        history = self.history.setdefault(name, deque(maxlen=200))
        history.append(now)
        while len(history) > 1 and now-history[0] > 2:
            history.popleft()
        self.samples[name] = dict(values=deepcopy(values), received_at=now, valid=True,
                                  stamp=stamp, base_age=max(0., base_age))
        return True

    def wheel(self, x, y, stamp):
        previous = self.last_wheel
        self.last_wheel = (x, y, stamp)
        if previous is None:
            return
        dt = stamp-previous[2]
        distance = math.hypot(x-previous[0], y-previous[1])
        if not 0 < dt <= 1 or distance > 2*dt+.05:
            self.distance_epoch += 1
            self.distance = 0.
        else:
            self.distance += distance

    def snapshot(self, name, now, ros_now):
        sample = self.samples.get(name)
        result = dict(received=sample is not None, valid=False, fresh=False, age_sec=0.,
                      timeout_sec=TIMEOUTS[name], stamp=None, rate_hz=None, values={})
        if sample is None:
            return result
        age = max(0., now-sample['received_at']+sample['base_age'])
        if sample['stamp'] is not None:
            age = max(age, ros_now-sample['stamp'])
        valid = sample['valid'] and (sample['stamp'] is None or ros_now-sample['stamp'] >= -.1)
        fresh = valid and age < TIMEOUTS[name]
        history = self.history.get(name, ())
        duration = history[-1]-history[0] if len(history) > 1 else 0.
        result.update(valid=valid, fresh=fresh, age_sec=age, stamp=sample['stamp'],
                      rate_hz=(len(history)-1)/duration if fresh and duration > 0 else None,
                      values=deepcopy(sample['values']))
        return result
