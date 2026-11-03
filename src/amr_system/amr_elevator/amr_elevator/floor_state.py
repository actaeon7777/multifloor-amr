"""Current floor and inside/outside-the-car estimation (no ROS).

The operator sets the start floor. A requested car target plus closed doors
starts transit; a settled open door proposes an ASSUMED arrival candidate.
With the camera enabled, its guess selects the asked floor; unknown or missing
evidence falls back to the ride target. A negative answer may name the real floor.
Only an explicit operator, vision or simulator confirmation for the current
candidate ID, while the door is open or opening, verifies arrival. Closure
while the whole body is inside the car, or position reports have gone stale,
invalidates that verification and requires a new candidate and confirmation,
even if the door simply reopens. A fresh position straddling the doorway keeps
the verified floor while the robot exits.
Rejected stops require another closed-door journey before proposing again.
Door and position evidence guard crossings; leaving an unconfirmed floor is a
rule violation. Localization READY after a verified exit confirms the floor.
"""

from dataclasses import dataclass, replace

from amr_elevator.door_detector import CLOSED, OPEN, OPENING

ZONE_UNKNOWN, ZONE_OUTSIDE, ZONE_IN_ELEVATOR = 0, 1, 2
ZONE_NAMES = {ZONE_UNKNOWN: 'UNKNOWN', ZONE_OUTSIDE: 'OUTSIDE', ZONE_IN_ELEVATOR: 'IN_ELEVATOR'}
SOURCE_OPERATOR, SOURCE_VISION, SOURCE_SIM = 1, 2, 3
SOURCE_NAMES = {SOURCE_OPERATOR: 'OPERATOR', SOURCE_VISION: 'VISION', SOURCE_SIM: 'SIM'}


@dataclass(frozen=True)
class FloorSnapshot:
    zone: int = ZONE_OUTSIDE
    floor: str = ''
    transit_from: str = ''
    transit_to: str = ''
    confidence: str = 'ASSUMED'
    last_event: str = ''
    awaiting_confirm: bool = False
    candidate_id: int = 0
    candidate_source: str = ''


@dataclass(frozen=True)
class FloorEventRecord:
    event: str
    from_floor: str
    to_floor: str
    detail: str


@dataclass
class _Timers:
    door_state: int = 0
    door_open_seen: float = None
    door_closed_since: float = None
    arrival_wait_since: float = None
    closed_during_transit: float = 0.
    exit_time: float = None
    ready: bool = False
    confirmation_rejected: bool = False
    vision_guess: str = None  # None: no report; '': reported unknown
    vision_confident: bool = False  # score and margin clear the automatic-decision bar
    vision_references: frozenset = frozenset()  # floors with reference images at guess time
    origin_verified: bool = False  # transit_from was a verified/confirmed floor at departure


class FloorStateMachine:
    def __init__(self, building, initial_floor=None, depart_closed_s=1.,
                 min_travel_closed_s=3., arrival_settle_s=1., door_recent_s=5., position_stale_s=1.,
                 vision_wait_s=0., vision_auto=False, vision_auto_min_score=.93,
                 vision_auto_min_margin=.03):
        self.building = building
        self.depart_closed_s = depart_closed_s
        self.min_travel_closed_s = min_travel_closed_s
        self.arrival_settle_s = arrival_settle_s
        self.door_recent_s = door_recent_s
        self.position_stale_s = position_stale_s
        self.vision_wait_s = vision_wait_s
        # Automatic arrival (2026-11-02): a confident camera guess verifies the stop without a
        # person; below the bar the kiosk question remains the fallback.
        self.vision_auto = vision_auto
        self.vision_auto_min_score = vision_auto_min_score
        self.vision_auto_min_margin = vision_auto_min_margin
        floor = initial_floor or building.default_floor
        building.floor(floor)
        self.state = FloorSnapshot(floor=floor)
        self.car_target = ''
        self.t = _Timers()
        self.in_car = self.past_hall_door = self.enclosed = False
        self.fully_inside = False
        self.position_time = None
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
        self.state = replace(self.state, zone=ZONE_UNKNOWN, confidence='UNKNOWN',
                             awaiting_confirm=False, candidate_source='')
        self.car_target = ''
        self._emit('RULE_VIOLATION', self.state.floor, '', detail)

    # Inputs ------------------------------------------------------------------

    def on_car_target(self, floor):
        """The mission requested ``floor`` from inside the car."""
        self.building.floor(floor)
        self.car_target = floor

    def _known_floor(self, floor):
        return floor in self.building.floors or floor in self.building.pass_floors

    def on_vision_guess(self, floor, now, score=0., margin=0., reference_floors=()):
        """Accept evidence only for this travelled, open-door arrival window."""
        s, t = self.state, self.t
        if (self.vision_wait_s <= 0 or s.zone != ZONE_IN_ELEVATOR or s.confidence != 'TRANSIT'
                or t.arrival_wait_since is None or not now >= t.arrival_wait_since
                or now > t.arrival_wait_since + self.arrival_settle_s + self.vision_wait_s
                or t.closed_during_transit < self.min_travel_closed_s):
            return 'ignored'
        t.vision_guess = floor if self._known_floor(floor) else ''
        t.vision_confident = bool(t.vision_guess) and (
            score >= self.vision_auto_min_score and margin >= self.vision_auto_min_margin)
        t.vision_references = frozenset(reference_floors)
        return 'stored' if t.vision_guess else 'unknown'

    def _target_recognisable(self, origin, references):
        """A camera "this is the target" is trusted only if every floor the car can stop at on
        the way (strictly between origin and target) and the target itself have references:
        an unreferenced floor that resembles the target would otherwise pass as the target.
        Non-target decisions only keep the robot riding and need no such check."""
        levels = {f.name: f.level for f in (*self.building.floors.values(), *self.building.pass_floors.values())}
        if not self.t.origin_verified or origin not in levels or self.car_target not in levels:
            return False  # unverified origin (e.g. after a rejected stop): no bound on the stops
        low, high = sorted((levels[origin], levels[self.car_target]))
        on_the_way = {name for name, level in levels.items() if low < level < high} | {self.car_target}
        return on_the_way <= set(references)

    def on_arrival_confirm(self, source, floor, candidate_id, accepted, now, actual_floor=''):
        s = self.state
        if (s.zone != ZONE_IN_ELEVATOR or not s.awaiting_confirm or floor != s.floor
                or candidate_id != s.candidate_id or self.t.door_state not in (OPEN, OPENING)):
            return 'ignored'
        detail = SOURCE_NAMES.get(source, str(source))
        if not accepted and actual_floor == floor:
            return 'ignored'
        corrected = not accepted and self._known_floor(actual_floor)
        if accepted or corrected:
            verified_floor = actual_floor if corrected else floor
            if corrected:
                detail += f' corrected {floor}->{actual_floor}'
            self.state = replace(s, floor=verified_floor, confidence='VERIFIED',
                                 awaiting_confirm=False, candidate_source='')
            self.t.confirmation_rejected = False
            self._emit('ARRIVED', s.floor, verified_floor, detail)
            return 'accepted'
        self.state = replace(s, floor='', confidence='TRANSIT', transit_from=s.floor,
                             transit_to=self.car_target, awaiting_confirm=False, candidate_source='')
        self.t.origin_verified = False  # transit_from is the rejected guess, not a known floor
        self.t.closed_during_transit = 0.
        self.t.arrival_wait_since = None
        self.t.vision_guess = None
        self.t.vision_confident = False
        self.t.confirmation_rejected = True
        self.t.door_closed_since = None
        self._emit('ARRIVAL_REJECTED', s.floor, self.car_target, detail)
        return 'rejected'

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
        if door_state != OPEN:
            self.t.arrival_wait_since = None
            self.t.vision_guess = None
            self.t.vision_confident = False
        s = self.state
        if (s.zone == ZONE_IN_ELEVATOR and s.confidence == 'TRANSIT' and door_state == OPEN
                and previous != OPEN and self.t.arrival_wait_since is None
                and (not self.t.confirmation_rejected
                     or self.t.closed_during_transit >= self.min_travel_closed_s)):
            self.t.arrival_wait_since = now
            self.t.vision_guess = None
            self.t.vision_confident = False

    def on_position(self, in_car, past_hall_door, enclosed, now, *, fully_inside=False):
        """Report centre/zone evidence and whether the whole body clears the car door."""
        self.in_car, self.past_hall_door, self.enclosed = in_car, past_hall_door, enclosed
        self.fully_inside, self.position_time = fully_inside, now
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
                                 awaiting_confirm=False, candidate_source='')
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
        if rising and s.zone == ZONE_OUTSIDE and s.confidence == 'VERIFIED' and self.t.exit_time is not None:
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
                             transit_to='', confidence='CONFIRMED', awaiting_confirm=False,
                             candidate_source='')
        self.car_target = ''
        self.t.arrival_wait_since = None
        self.t.confirmation_rejected = False
        self._emit('FLOOR_SET_BY_OPERATOR', old, floor)

    def tick(self, now):
        s, t = self.state, self.t
        if s.confidence == 'VERIFIED':
            position_stale = self.position_time is None or now - self.position_time > self.position_stale_s
            # A verified stop that is not the ride target is never exited by the mission, so the
            # robot is inside: depart without the whole-body check, which only guards an exit.
            can_depart = self.car_target != s.floor or position_stale or self.fully_inside
        else:
            can_depart = self.car_target != s.floor or s.confidence == 'ASSUMED'
        if (s.zone == ZONE_IN_ELEVATOR and s.confidence != 'TRANSIT' and t.door_state == CLOSED
                and t.door_closed_since is not None and now - t.door_closed_since >= self.depart_closed_s
                and self.car_target and can_depart):
            # A camera-only verification is not a known origin for bounding the next stop.
            t.origin_verified = (s.confidence == 'CONFIRMED'
                                 or (s.confidence == 'VERIFIED' and s.candidate_source != 'VISION_AUTO'))
            self.state = replace(s, floor='', transit_from=s.floor, transit_to=self.car_target,
                                 confidence='TRANSIT', awaiting_confirm=False, candidate_source='')
            t.closed_during_transit = 0.  # door_closed_since counts until opening
            t.arrival_wait_since = None
            t.vision_guess = None
            t.vision_confident = False
            t.vision_references = frozenset()
            t.confirmation_rejected = False
            self._emit('DEPARTED', s.floor, self.car_target)
        s = self.state
        if (s.zone == ZONE_IN_ELEVATOR and s.confidence == 'TRANSIT' and t.door_state == OPEN
                and t.arrival_wait_since is not None
                and now - t.arrival_wait_since >= self.arrival_settle_s):
            moved = t.closed_during_transit >= self.min_travel_closed_s
            use_vision = self.vision_wait_s > 0 and moved
            if (use_vision and t.vision_guess is None
                    and now < t.arrival_wait_since + self.arrival_settle_s + self.vision_wait_s):
                return s
            candidate = s.transit_to if moved else s.transit_from
            source = 'TARGET'
            if use_vision:
                candidate = t.vision_guess or self.car_target
                if t.vision_guess:
                    source = 'VISION'
            awaiting = (moved if self.vision_wait_s > 0 else candidate == self.car_target)
            automatic = (use_vision and source == 'VISION' and self.vision_auto and t.vision_confident
                         and (candidate != self.car_target
                              or self._target_recognisable(s.transit_from, t.vision_references)))
            t.arrival_wait_since = None
            t.vision_guess = None
            t.vision_confident = False
            if automatic:
                # Same effect as an accepted confirmation while the door is open: the mission
                # alights only if this is the ride target, otherwise the robot keeps riding.
                self.state = replace(s, floor=candidate, confidence='VERIFIED', transit_from='',
                                     transit_to='', awaiting_confirm=False, candidate_source='VISION_AUTO',
                                     candidate_id=s.candidate_id % 0xffffffff + 1)
                self._emit('ARRIVAL_CANDIDATE', s.transit_from, candidate, 'after travel source VISION_AUTO')
                self._emit('ARRIVED', s.transit_from, candidate, 'VISION auto')
                if candidate != self.car_target:
                    self._emit('ARRIVED_OTHER_FLOOR', s.transit_from, candidate, f'target {self.car_target}')
                return self.state
            self.state = replace(s, floor=candidate, confidence='ASSUMED', transit_from='',
                                 transit_to='', awaiting_confirm=awaiting,
                                 candidate_source=source if awaiting else '',
                                 candidate_id=s.candidate_id % 0xffffffff + 1)
            self._emit('ARRIVAL_CANDIDATE', s.transit_from, candidate,
                       f'after travel source {source}' if moved else 'door only reopened')
            if candidate != self.car_target:
                self._emit('ARRIVED_OTHER_FLOOR', s.transit_from, candidate,
                           f'target {self.car_target}')
        return self.state
