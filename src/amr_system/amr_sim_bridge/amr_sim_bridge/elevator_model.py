"""ROS/Isaac independent elevator car, RF button devices and hall beacons.

Replaces the real RF -> STM32 -> CAN path in simulation.  It only produces what
the real devices report: hall beacons announcing their floor every period, and
an acknowledgement after physically pressing a button.  Door state is exposed in
``status()`` for animation and evaluation only; the AMR must sense doors with
its LiDAR.  Faults can be injected to exercise the AMR's recovery rules.

Floors are indexed bottom-up; positions are in floor indices (float while
moving).  Times are seconds of the caller's (simulation) clock.
"""

from dataclasses import dataclass, field
from math import isfinite, sqrt
import random

CMD_HALL_CALL, CMD_CAR_FLOOR, CMD_DOOR_OPEN = 1, 2, 3
DIR_NONE, DIR_UP, DIR_DOWN = 0, 1, 2
EVENT_PRESENCE, EVENT_BUTTON_ACK = 1, 2
LOCATION_HALL, LOCATION_CAR = 0, 1
DOOR_STATES = ('CLOSED', 'OPENING', 'OPEN', 'CLOSING')

FAULT_DEFAULTS = {
    'drop_ack_probability': 0.,      # button pressed but acknowledgement lost
    'drop_next_acks': 0,             # deterministic: lose the next N acknowledgements
    'ack_extra_delay_s': 0.,         # late acknowledgement
    'duplicate_probability': 0.,     # same frame received twice
    'reorder_probability': 0.,       # frame held back and delivered after the next one
    'drop_presence_probability': 0., # hall beacon announcement lost
    'wrong_floor_code': None,        # extra presence frames claiming this floor code
    'stamp_lag_s': 0.,               # reception stamps lag the actual time (stale data)
    'stop_at_floor_code': None,      # car also stops at this floor (another passenger)
    'hold_time_s': None,             # override the random door hold time
    'door_never_opens': False,       # car arrives but the door stays shut
    'button_fails': False,           # device presses nothing and reports nothing
}


@dataclass
class Frame:
    stamp: float
    event: int
    location: int
    floor_code: int
    sequence: int
    ack_sequence: int
    rssi_dbm: int


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
class _Pending:
    due: float
    frame: Frame
    effect: object = None


@dataclass
class ElevatorSim:
    floors: list                       # [(name, beacon_floor_code)] bottom-up
    floor_height_m: float = 3.6
    speed_m_s: float = 1.0
    accel_m_s2: float = .5
    open_time_s: tuple = (6., 10.)
    door_move_s: float = 1.5
    ack_delay_s: float = .2
    presence_period_s: float = 1.
    leak_probability: float = .2
    seed: int = 0
    start_floor: str = None
    faults: dict = field(default_factory=dict)

    def __post_init__(self):
        names = [name for name, _ in self.floors]
        codes = [code for _, code in self.floors]
        if not self.floors or len(set(names)) != len(names) or len(set(codes)) != len(codes) or 0 in codes:
            raise ValueError('floors need unique names and non-zero unique codes')
        for value in (self.floor_height_m, self.speed_m_s, self.accel_m_s2, self.door_move_s,
                      self.presence_period_s):
            if not isfinite(value) or value <= 0:
                raise ValueError('elevator timings and dimensions must be positive')
        if not 0 < self.open_time_s[0] <= self.open_time_s[1]:
            raise ValueError('open_time_s must be 0 < min <= max')
        self.rng = random.Random(self.seed)
        self.faults = {**FAULT_DEFAULTS, **self.faults}
        self.position = float(names.index(self.start_floor) if self.start_floor else 0)
        self.door = 'CLOSED'
        self.door_ratio = 0.
        self.hold_until = None
        self.obstructed = False
        self.motion = None
        self.direction = DIR_NONE
        self.hall_calls = set()
        self.car_calls = set()
        self.extra_stop = None
        self.pending = []
        self.held = None
        self.sequences = {}
        self.next_presence = None
        self.now = 0.
        self.robot_level = self.position
        self.robot_in_car = False

    # Inputs -----------------------------------------------------------------

    def index_of_code(self, code):
        return next((i for i, (_, c) in enumerate(self.floors) if c == code), None)

    def set_robot(self, level, in_car):
        """Robot height in floor indices and whether it stands in the car."""
        self.robot_level = float(level)
        self.robot_in_car = bool(in_car)

    def set_faults(self, **faults):
        unknown = set(faults) - set(FAULT_DEFAULTS)
        if unknown:
            raise ValueError(f'unknown faults: {sorted(unknown)}')
        self.faults.update(faults)

    def obstruct(self, blocked):
        """Something stands in the door plane (simulated safety edge)."""
        self.obstructed = bool(blocked)

    def command(self, now, command, floor_code, direction, sequence):
        """An AMR RF command; the device presses the button after ack_delay_s."""
        if command == CMD_HALL_CALL:
            index = self.index_of_code(floor_code)
            if index is None or direction not in (DIR_UP, DIR_DOWN):
                return
            location, device_code = LOCATION_HALL, floor_code
            effect = ('hall', index)
        elif command == CMD_CAR_FLOOR:
            index = self.index_of_code(floor_code)
            if index is None:
                return
            location, device_code, effect = LOCATION_CAR, 0, ('car', index)
        elif command == CMD_DOOR_OPEN:
            location, device_code, effect = LOCATION_CAR, 0, ('open', None)
        else:
            return
        if self.faults['button_fails']:
            return
        due = now + self.ack_delay_s
        self.pending.append(_Pending(due, None, effect))
        rssi = -55 if location == LOCATION_CAR else self._hall_rssi(self.index_of_code(device_code))
        dropped = self.faults['drop_next_acks'] > 0
        if dropped:
            self.faults['drop_next_acks'] -= 1
        if rssi is not None and not dropped and self.rng.random() >= self.faults['drop_ack_probability']:
            frame = Frame(0., EVENT_BUTTON_ACK, location, device_code,
                          self._next_sequence((location, device_code)), sequence & 0xff, rssi)
            self.pending.append(_Pending(due + self.faults['ack_extra_delay_s'], frame))

    # Simulation -------------------------------------------------------------

    def step(self, now):
        """Advance to ``now`` and return the frames the AMR receives."""
        dt = max(0., now - self.now)
        self.now = now
        out = []
        for item in sorted([p for p in self.pending if p.due <= now], key=lambda p: p.due):
            self.pending.remove(item)
            if item.effect is not None:
                self._press(*item.effect)
            if item.frame is not None:
                out.append(item.frame)
        out.extend(self._presence(now))
        self._advance_car(now, dt)
        return self._deliver(out, now)

    def status(self):
        stopped_at = None if self.motion else round(self.position)
        return {
            'car_position': self.position,
            'car_floor': self.floors[stopped_at][0] if stopped_at is not None else None,
            'moving': self.motion is not None,
            'direction': {DIR_NONE: 'none', DIR_UP: 'up', DIR_DOWN: 'down'}[self.direction],
            'door': self.door,
            'door_ratio': self.door_ratio,
            'hold_remaining_s': None if self.hold_until is None else max(0., self.hold_until - self.now),
            'hall_calls': sorted(self.floors[i][0] for i in self.hall_calls),
            'car_calls': sorted(self.floors[i][0] for i in self.car_calls),
            'obstructed': self.obstructed,
            'faults': {k: v for k, v in self.faults.items() if v != FAULT_DEFAULTS[k]},
        }

    # Internals --------------------------------------------------------------

    def _next_sequence(self, device):
        value = self.sequences.get(device, -1) + 1 & 0xff
        self.sequences[device] = value
        return value

    def _hall_rssi(self, index):
        """RSSI of a hall beacon at the robot, or None when not received."""
        if index is None:
            return None
        if self.robot_in_car:
            at_floor = self.motion is None and abs(self.position - index) < .05
            if at_floor and self.door_ratio > .5:
                return -58
            if abs(self.position - index) < .5 and self.rng.random() < self.leak_probability:
                return -85
            return None
        distance = abs(self.robot_level - index)
        if distance < .5:
            return -50
        if distance < 1.5 and self.rng.random() < self.leak_probability:
            return -82
        return None

    def _presence(self, now):
        if self.next_presence is None:
            self.next_presence = now
        frames = []
        while self.next_presence <= now:
            for index, (_, code) in enumerate(self.floors):
                rssi = self._hall_rssi(index)
                if rssi is None or self.rng.random() < self.faults['drop_presence_probability']:
                    continue
                frames.append(Frame(0., EVENT_PRESENCE, LOCATION_HALL, code,
                                    self._next_sequence((LOCATION_HALL, code)), 0,
                                    rssi + self.rng.randint(-3, 3)))
            wrong = self.faults['wrong_floor_code']
            if wrong is not None:
                frames.append(Frame(0., EVENT_PRESENCE, LOCATION_HALL, wrong,
                                    self._next_sequence((LOCATION_HALL, wrong)), 0, -50))
            self.next_presence += self.presence_period_s
        return frames

    def _deliver(self, frames, now):
        stamp = now - self.faults['stamp_lag_s']
        delivered = []
        for frame in frames:
            frame.stamp = stamp
            copies = 2 if self.rng.random() < self.faults['duplicate_probability'] else 1
            for _ in range(copies):
                if self.held is None and self.rng.random() < self.faults['reorder_probability']:
                    self.held = frame
                    continue
                delivered.append(frame)
                if self.held is not None and self.held is not frame:
                    delivered.append(self.held)
                    self.held = None
        return delivered

    def _stopped_at(self):
        return None if self.motion else round(self.position)

    def _press(self, kind, index):
        here = self._stopped_at()
        if kind == 'open':
            if here is not None:
                self._open()
            return
        if here == index:
            self._open()
            return
        (self.hall_calls if kind == 'hall' else self.car_calls).add(index)

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
        stop = self.index_of_code(self.faults['stop_at_floor_code']) \
            if self.faults['stop_at_floor_code'] is not None else None
        if stop is not None and min(here, target) < stop < max(here, target):
            target = stop
            self.faults['stop_at_floor_code'] = None  # one unexpected stop per injection
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
