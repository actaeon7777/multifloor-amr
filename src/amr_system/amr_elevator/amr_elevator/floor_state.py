"""Current floor and inside/outside-the-car estimation (no ROS).

Evidence: hall beacons (RF, 1 Hz, "I am the N-floor beacon"), the acknowledged
car-floor command, the LiDAR door state, and the robot position relative to the
car (pose from the last verified localization carried by odometry, plus LiDAR
enclosure).  Rules (docs/ELEVATOR.md 4.2):

* A floor changes only through the car: depart (door closed with an acknowledged
  target) -> arrive (door open + a strong hall beacon, or a weak one from the
  target floor after at least min_travel_closed_s of closed-door travel).  Outside the car a
  different floor's beacon is RF leakage: flagged, never applied.
* Beacons are filtered per device for staleness, duplicates and old sequence
  numbers.  Floor beacons heard while the car travels are ignored.
* A stop at a pass floor (beacon installed, no map) is another passenger's stop:
  the ride continues and the floor stays in transit.  Car beacon PRESENCE is only
  device liveness and never moves the floor.
* Crossing the door line needs the door to have been seen open; otherwise the
  zone becomes UNKNOWN (stop) until the operator sets the floor.
* Leaving the car before the floor is confirmed by a beacon is a violation.
"""

from dataclasses import dataclass, field, replace

from amr_elevator.door_detector import CLOSED, OPEN, OPENING

ZONE_UNKNOWN, ZONE_OUTSIDE, ZONE_IN_ELEVATOR = 0, 1, 2
ZONE_NAMES = {ZONE_UNKNOWN: 'UNKNOWN', ZONE_OUTSIDE: 'OUTSIDE', ZONE_IN_ELEVATOR: 'IN_ELEVATOR'}
EVENT_PRESENCE = 1
LOCATION_HALL = 0


@dataclass(frozen=True)
class FloorSnapshot:
    zone: int = ZONE_OUTSIDE
    floor: str = ''
    transit_from: str = ''
    transit_to: str = ''
    confidence: str = 'ASSUMED'
    last_event: str = ''
    floor_conflict: bool = False
    duplicate_count: int = 0
    out_of_order_count: int = 0
    stale_count: int = 0


@dataclass(frozen=True)
class FloorEventRecord:
    event: str
    from_floor: str
    to_floor: str
    detail: str


class BeaconFilter:
    """Per-device staleness, duplicate and sequence-order checks."""

    def __init__(self, max_age=2., duplicate_window=5., future_tolerance=.5):
        self.max_age, self.window, self.future = max_age, duplicate_window, future_tolerance
        self.last = {}

    def accept(self, device, sequence, stamp, now):
        if now - stamp > self.max_age or stamp - now > self.future:
            return 'stale'
        previous = self.last.get(device)
        if previous is not None and now - previous[1] < self.window:
            delta = (sequence - previous[0]) % 256
            if delta == 0:
                return 'duplicate'
            if delta >= 128:
                return 'out_of_order'
        self.last[device] = (sequence, now)
        return 'ok'


@dataclass
class _Timers:
    door_state: int = 0
    door_open_seen: float = None
    door_closed_since: float = None
    arrival_wait_since: float = None
    closed_during_transit: float = 0.
    conflict_seen: float = None
    exit_time: float = None
    ready: bool = False
    pass_stop: str = ''
    unknown_codes: set = field(default_factory=set)


class FloorStateMachine:
    def __init__(self, building, initial_floor=None, presence_min_rssi=-70, depart_closed_s=1.,
                 min_travel_closed_s=3., arrival_beacon_timeout_s=3., door_recent_s=5.,
                 conflict_hold_s=5., beacon_filter=None):
        self.building = building
        self.presence_min_rssi = presence_min_rssi
        self.depart_closed_s = depart_closed_s
        self.min_travel_closed_s = min_travel_closed_s
        self.arrival_timeout = arrival_beacon_timeout_s
        self.door_recent_s = door_recent_s
        self.conflict_hold_s = conflict_hold_s
        self.filter = beacon_filter or BeaconFilter()
        floor = initial_floor or building.default_floor
        building.floor(floor)
        self.state = FloorSnapshot(floor=floor)
        self.car_target = ''
        self.t = _Timers()
        self.events = []

    # Helpers -----------------------------------------------------------------

    def _emit(self, event, from_floor='', to_floor='', detail=''):
        self.events.append(FloorEventRecord(event, from_floor, to_floor, detail))
        self.state = replace(self.state, last_event=event)

    def drain_events(self):
        events, self.events = self.events, []
        return events

    def _door_recently_open(self, now):
        return self.t.door_state in (OPEN, OPENING) or (
            self.t.door_open_seen is not None and now - self.t.door_open_seen <= self.door_recent_s)

    def _violation(self, now, detail):
        self.state = replace(self.state, zone=ZONE_UNKNOWN, confidence='UNKNOWN')
        self._emit('RULE_VIOLATION', self.state.floor, '', detail)

    # Inputs ------------------------------------------------------------------

    def on_car_target(self, floor):
        """The car device acknowledged pressing ``floor``."""
        self.building.floor(floor)
        self.car_target = floor

    def on_beacon(self, event, location, floor_code, sequence, stamp, rssi, now):
        verdict = self.filter.accept((location, floor_code), sequence, stamp, now)
        if verdict != 'ok':
            key = {'stale': 'stale_count', 'duplicate': 'duplicate_count',
                   'out_of_order': 'out_of_order_count'}[verdict]
            self.state = replace(self.state, **{key: getattr(self.state, key) + 1})
            return verdict
        if event != EVENT_PRESENCE or location != LOCATION_HALL:
            return 'ok'
        heard = self.building.floor_by_beacon_code(floor_code)
        if heard is None:
            return self._unmapped_beacon(floor_code, rssi, now)
        if rssi != 0 and rssi < self.presence_min_rssi:
            if not self._weak_target_arrival(heard.name):
                return 'weak'
            self._arrive(heard.name, 'BEACON', now, 'weak hall beacon at the target floor after travel')
            return 'ok'
        s = self.state
        if s.zone == ZONE_IN_ELEVATOR and s.confidence == 'TRANSIT':
            if self.t.arrival_wait_since is None:
                return 'ignored_in_transit'
            self._arrive(heard.name, 'BEACON', now)
            return 'ok'
        if s.zone == ZONE_IN_ELEVATOR and s.confidence == 'ASSUMED' and self.t.door_state in (OPEN, OPENING):
            self._arrive(heard.name, 'BEACON', now)  # late beacon upgrades an assumed arrival
            return 'ok'
        if heard.name == s.floor:
            if s.confidence == 'ASSUMED' and s.zone == ZONE_OUTSIDE:
                self.state = replace(s, confidence='BEACON')
                self._emit('FLOOR_BEACON', s.floor, s.floor)
            return 'ok'
        if s.zone != ZONE_UNKNOWN:
            if not s.floor_conflict:
                self._emit('FLOOR_CONFLICT', s.floor, heard.name, 'other floor beacon heard; not applied')
            self.state = replace(self.state, floor_conflict=True)
            self.t.conflict_seen = now
        return 'conflict'

    def _weak_target_arrival(self, floor):
        """A weak beacon confirms only the target floor, after real travel, with the door open."""
        s, t = self.state, self.t
        if (s.zone != ZONE_IN_ELEVATOR or floor != self.car_target or t.door_state not in (OPEN, OPENING)
                or t.closed_during_transit < self.min_travel_closed_s):
            return False
        if s.confidence == 'TRANSIT':
            return t.arrival_wait_since is not None
        return s.confidence == 'ASSUMED'

    def _unmapped_beacon(self, floor_code, rssi, now):
        passing = self.building.pass_floor_by_beacon_code(floor_code)
        if passing is None:
            if floor_code not in self.t.unknown_codes:  # Once per code, not every second.
                self.t.unknown_codes.add(floor_code)
                self._emit('UNKNOWN_BEACON', detail=f'floor code {floor_code}')
            return 'unknown'
        if rssi != 0 and rssi < self.presence_min_rssi:
            return 'weak'
        s = self.state
        if s.zone == ZONE_IN_ELEVATOR and s.confidence == 'TRANSIT':
            if self.t.arrival_wait_since is None:
                return 'ignored_in_transit'
            # Someone else's stop at a floor without a map: keep riding, never assume
            # the target from this door opening.
            self.t.arrival_wait_since = None
            if self.t.pass_stop != passing.name:
                self.t.pass_stop = passing.name
                self._emit('PASS_FLOOR_STOP', s.transit_from, passing.name,
                           f'target {s.transit_to}; staying in the car')
            return 'pass_floor'
        if s.zone != ZONE_UNKNOWN:
            if not s.floor_conflict:
                self._emit('FLOOR_CONFLICT', s.floor, passing.name, 'pass floor beacon heard; not applied')
            self.state = replace(self.state, floor_conflict=True)
            self.t.conflict_seen = now
        return 'conflict'

    def on_door(self, door_state, now):
        previous = self.t.door_state
        self.t.door_state = door_state
        if door_state in (OPEN, OPENING):
            self.t.door_open_seen = now
            if self.t.door_closed_since is not None and self.state.confidence == 'TRANSIT':
                self.t.closed_during_transit += now - self.t.door_closed_since
            self.t.door_closed_since = None
        elif door_state == CLOSED and self.t.door_closed_since is None:
            self.t.door_closed_since = now
            self.t.pass_stop = ''
        s = self.state
        if (s.zone == ZONE_IN_ELEVATOR and s.confidence == 'TRANSIT' and door_state == OPEN
                and previous != OPEN and self.t.arrival_wait_since is None):
            self.t.arrival_wait_since = now

    def on_position(self, in_car, past_hall_door, enclosed, now):
        """``in_car``: centre inside the car polygon; ``past_hall_door``: clearly on the hall side."""
        s = self.state
        if s.zone == ZONE_OUTSIDE and in_car and enclosed:
            if not self._door_recently_open(now):
                self._violation(now, 'inside the car without the door seen open')
                return
            self.state = replace(s, zone=ZONE_IN_ELEVATOR)
            self._emit('ENTERED_ELEVATOR', s.floor, s.floor)
        elif s.zone == ZONE_IN_ELEVATOR and past_hall_door and not enclosed:
            if s.confidence == 'TRANSIT' or not self._door_recently_open(now):
                self._violation(now, 'left the car while travelling or through a closed door')
                return
            self.state = replace(s, zone=ZONE_OUTSIDE, transit_from='', transit_to='',
                                 floor_conflict=False)
            self.t.exit_time, self.t.ready = now, False
            self.t.arrival_wait_since = None
            self.car_target = ''
            self._emit('EXITED_ELEVATOR', s.floor, s.floor)
            if s.confidence == 'ASSUMED':
                self._emit('RULE_VIOLATION', s.floor, s.floor, 'left the car at an unconfirmed floor')

    def on_localization(self, ready, now):
        rising = ready and not self.t.ready
        self.t.ready = ready
        s = self.state
        if rising and s.zone == ZONE_OUTSIDE and s.confidence == 'BEACON' and self.t.exit_time is not None:
            self.state = replace(s, confidence='CONFIRMED')
            self._emit('FLOOR_CONFIRMED', s.floor, s.floor, 'localization verified after the ride')

    def set_floor(self, floor, stationary):
        """Operator override; only while stopped outside the car (or after a violation)."""
        self.building.floor(floor)
        if not stationary:
            raise ValueError('the robot must be stationary')
        if self.state.zone == ZONE_IN_ELEVATOR:
            raise ValueError('not allowed inside the car')
        old = self.state.floor
        self.state = replace(self.state, zone=ZONE_OUTSIDE, floor=floor, transit_from='',
                             transit_to='', confidence='CONFIRMED', floor_conflict=False)
        self.car_target = ''
        self.t.arrival_wait_since = None
        self._emit('FLOOR_SET_BY_OPERATOR', old, floor)

    def tick(self, now):
        s = self.state
        t = self.t
        if (s.zone == ZONE_IN_ELEVATOR and s.confidence != 'TRANSIT' and t.door_state == CLOSED
                and t.door_closed_since is not None and now - t.door_closed_since >= self.depart_closed_s
                and self.car_target and self.car_target != s.floor):
            self.state = replace(s, floor='', transit_from=s.floor, transit_to=self.car_target,
                                 confidence='TRANSIT')
            t.closed_during_transit = 0.  # door_closed_since keeps counting until it opens
            t.arrival_wait_since = None
            self._emit('DEPARTED', s.floor, self.car_target)
        s = self.state
        if (s.confidence == 'TRANSIT' and t.arrival_wait_since is not None
                and now - t.arrival_wait_since >= self.arrival_timeout):
            moved = t.closed_during_transit >= self.min_travel_closed_s
            assumed = s.transit_to if moved else s.transit_from
            self._arrive(assumed, 'ASSUMED', now,
                         'no hall beacon after the door opened' + ('' if moved else '; door only reopened'))
        if s.floor_conflict and t.conflict_seen is not None and now - t.conflict_seen > self.conflict_hold_s:
            self.state = replace(self.state, floor_conflict=False)
        return self.state

    def _arrive(self, floor, confidence, now, detail=''):
        s = self.state
        origin = s.transit_from or s.floor
        target = self.car_target
        self.t.arrival_wait_since = None
        self.state = replace(s, floor=floor, confidence=confidence, transit_from='', transit_to='')
        if floor == target:
            if confidence != 'ASSUMED':
                self.car_target = ''  # an assumed arrival keeps the target until a beacon agrees
            self._emit('ARRIVED', origin, floor, detail or confidence)
        else:
            # Another passenger's stop, or the door only reopened: keep the target for the next ride.
            self._emit('ARRIVED_OTHER_FLOOR', origin, floor, detail or f'target {target}')
        if confidence == 'ASSUMED':
            self._emit('ARRIVAL_UNCONFIRMED', origin, floor, detail)
