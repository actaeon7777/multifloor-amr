"""Clock-driven elevator dynamics and delayed human button presses.

Floors are names ordered bottom-up by building level. Times use the caller's clock.
"""
from dataclasses import dataclass, field
from math import isfinite, sqrt
import random

KIND_CALL, KIND_CAR_FLOOR, KIND_CANCEL = 1, 2, 3
DIR_NONE, DIR_UP, DIR_DOWN = 0, 1, 2
DOOR_STATES = ('CLOSED', 'OPENING', 'OPEN', 'CLOSING')
FAULT_DEFAULTS = {'stop_at_floor': None, 'hold_time_s': None,
                  'door_never_opens': False, 'button_fails': False}


@dataclass
class _Motion:
    start: float
    origin: float
    target: float
    distance: float
    cruise: float
    accel: float
    duration: float


@dataclass
class ElevatorSim:
    floors: list
    floor_height_m: float = 3.6
    speed_m_s: float = 1.0
    accel_m_s2: float = .5
    open_time_s: tuple = (6., 10.)
    door_move_s: float = 1.5
    press_delay_s: float = 1.0
    seed: int = 0
    start_floor: str = None
    faults: dict = field(default_factory=dict)

    def __post_init__(self):
        if (not self.floors or any(not isinstance(n, str) or not n for n in self.floors)
                or len(set(self.floors)) != len(self.floors)):
            raise ValueError('floors need unique nonempty names')
        for value in (self.floor_height_m, self.speed_m_s, self.accel_m_s2, self.door_move_s):
            if not isfinite(value) or value <= 0:
                raise ValueError('elevator timings and dimensions must be positive')
        if not isfinite(self.press_delay_s) or self.press_delay_s < 0:
            raise ValueError('press_delay_s must be nonnegative')
        if not 0 < self.open_time_s[0] <= self.open_time_s[1]:
            raise ValueError('open_time_s must be 0 < min <= max')
        self.rng = random.Random(self.seed)
        initial_faults = self.faults
        self.faults = dict(FAULT_DEFAULTS)
        self.set_faults(**initial_faults)
        self.position = float(self.floors.index(self.start_floor) if self.start_floor else 0)
        self.door, self.door_ratio = 'CLOSED', 0.
        self.hold_until = None
        self.obstructed = False
        self.motion = None
        self.direction = DIR_NONE
        self.hall_calls, self.car_calls = set(), set()
        self.hall_directions = {}
        self.pending = []
        self.now = 0.
        self.robot_level, self.robot_in_car = self.position, False

    def index_of_floor(self, name):
        return self.floors.index(name) if name in self.floors else None

    def set_robot(self, level, in_car):
        self.robot_level, self.robot_in_car = float(level), bool(in_car)

    def set_faults(self, **faults):
        unknown = set(faults) - set(FAULT_DEFAULTS)
        if unknown:
            raise ValueError(f'unknown faults: {sorted(unknown)}')
        self.faults.update(faults)

    def obstruct(self, blocked):
        self.obstructed = bool(blocked)

    def request(self, now, kind, start_floor, target_floor, direction):
        """A person presses the requested button after press_delay_s; cancel leaves calls intact."""
        if kind == KIND_CALL:
            index = self.index_of_floor(start_floor)
            if direction not in (DIR_UP, DIR_DOWN):
                return
        elif kind == KIND_CAR_FLOOR:
            index = self.index_of_floor(target_floor)
        else:
            return
        if index is not None and not self.faults['button_fails']:
            self.pending.append((now + self.press_delay_s, kind, index, direction))

    def step(self, now):
        dt = max(0., now - self.now)
        self.now = now
        due = sorted(p for p in self.pending if p[0] <= now)
        self.pending = [p for p in self.pending if p[0] > now]
        for _, kind, index, direction in due:
            if kind == KIND_CALL:
                self.hall_directions[index] = direction
            if self.motion is None and round(self.position) == index:
                self._open()
            else:
                (self.hall_calls if kind == KIND_CALL else self.car_calls).add(index)
        self._advance_car(now, dt)

    def status(self):
        stopped_at = None if self.motion else round(self.position)
        return {
            'car_position': self.position,
            'car_floor': self.floors[stopped_at] if stopped_at is not None else None,
            'moving': self.motion is not None,
            'direction': {DIR_NONE: 'none', DIR_UP: 'up', DIR_DOWN: 'down'}[self.direction],
            'door': self.door,
            'door_ratio': self.door_ratio,
            'hold_remaining_s': None if self.hold_until is None else max(0., self.hold_until - self.now),
            'hall_calls': sorted(self.floors[i] for i in self.hall_calls),
            'car_calls': sorted(self.floors[i] for i in self.car_calls),
            'obstructed': self.obstructed,
            'faults': {k: v for k, v in self.faults.items() if v != FAULT_DEFAULTS[k]},
        }

    def _open(self):
        if self.faults['door_never_opens']:
            return
        if self.door in ('CLOSED', 'CLOSING'):
            self.door = 'OPENING'
        hold = self.faults['hold_time_s']
        if hold is None:
            hold = self.rng.uniform(*self.open_time_s)
        remaining = (1. - self.door_ratio) * self.door_move_s
        self.hold_until = self.now + remaining + hold

    def _advance_car(self, now, dt):
        rate = dt / self.door_move_s
        if self.door == 'OPENING':
            self.door_ratio = min(1., self.door_ratio + rate)
            if self.door_ratio >= 1.:
                self.door = 'OPEN'
        elif self.door == 'OPEN':
            if self.obstructed:
                self._open()
            elif self.hold_until is not None and now >= self.hold_until:
                self.door = 'CLOSING'
        elif self.door == 'CLOSING':
            if self.obstructed:
                self._open()
            else:
                self.door_ratio = max(0., self.door_ratio - rate)
                if self.door_ratio <= 0.:
                    self.door, self.hold_until = 'CLOSED', None
        if self.motion is not None:
            self._move(now)
        elif self.door == 'CLOSED':
            self._depart(now)

    def _depart(self, now):
        calls = self.hall_calls | self.car_calls
        here = round(self.position)
        if here in calls:
            self.hall_calls.discard(here)
            self.car_calls.discard(here)
            self._open()
            return
        if not calls:
            self.direction = DIR_NONE
            return
        above = [c for c in calls if c > here]
        below = [c for c in calls if c < here]
        if self.direction == DIR_UP and above or not below:
            target, self.direction = min(above), DIR_UP
        else:
            target, self.direction = max(below), DIR_DOWN
        stop = self.index_of_floor(self.faults['stop_at_floor']) \
            if self.faults['stop_at_floor'] is not None else None
        if stop is not None and min(here, target) < stop < max(here, target):
            target = stop
            self.faults['stop_at_floor'] = None  # one unexpected stop per injection
        distance = abs(target - here) * self.floor_height_m
        accel, speed = self.accel_m_s2, self.speed_m_s
        if distance < speed * speed / accel:  # triangular profile
            speed = sqrt(distance * accel)
        duration = distance / speed + speed / accel
        self.motion = _Motion(now, float(here), float(target), distance, speed, accel, duration)

    def _move(self, now):
        m = self.motion
        t = min(now - m.start, m.duration)
        ramp = m.cruise / m.accel
        if t < ramp:
            travelled = .5 * m.accel * t * t
        elif t < m.duration - ramp:
            travelled = .5 * m.accel * ramp * ramp + m.cruise * (t - ramp)
        else:
            left = m.duration - t
            travelled = m.distance - .5 * m.accel * left * left
        sign = 1. if m.target > m.origin else -1.
        self.position = m.origin + sign * travelled / self.floor_height_m
        if t >= m.duration:
            self.position = m.target
            self.motion = None
            index = int(m.target)
            self.car_calls.discard(index)
            self.hall_calls.discard(index)
            if not (self.hall_calls | self.car_calls):
                self.direction = DIR_NONE
            self._open()
