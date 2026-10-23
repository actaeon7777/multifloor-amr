"""Floor/zone rules for normal rides and abnormal signal sequences."""

import pytest

from amr_elevator.door_detector import CLOSED, CLOSING, OPEN, OPENING
from amr_elevator.floor_state import (
    ZONE_IN_ELEVATOR, ZONE_OUTSIDE, ZONE_UNKNOWN, FloorStateMachine,
)
from amr_stack.floor_config import load_floor_config

PRESENCE, ACK = 1, 2
HALL, CAR = 0, 1


@pytest.fixture(scope='module')
def building():
    return load_floor_config()


class Ride:
    """Drive a FloorStateMachine with timestamped evidence."""

    def __init__(self, building, floor='4F'):
        self.m = FloorStateMachine(building, initial_floor=floor)
        self.seq = {}
        self.now = 0.

    def at(self, now):
        self.now = now
        self.m.tick(now)
        return self

    def beacon(self, code, rssi=-50, sequence=None, stamp=None, event=PRESENCE, location=HALL):
        key = (location, code)
        if sequence is None:
            sequence = self.seq.get(key, -1) + 1 & 0xff
        self.seq[key] = sequence
        return self.m.on_beacon(event, location, code, sequence,
                                self.now if stamp is None else stamp, rssi, self.now)

    def door(self, state):
        self.m.on_door(state, self.now)
        self.m.tick(self.now)

    def position(self, in_car=False, past_hall=False, enclosed=None):
        self.m.on_position(in_car, past_hall, in_car if enclosed is None else enclosed, self.now)

    def events(self):
        return [e.event for e in self.m.drain_events()]

    @property
    def s(self):
        return self.m.state


def board(ride, start=10.):
    ride.at(start).door(OPENING)
    ride.at(start + 1.5).door(OPEN)
    ride.at(start + 3.).position(in_car=True)
    return ride


def depart(ride, target, start=20.):
    ride.m.on_car_target(target)
    ride.at(start).door(CLOSING)
    ride.at(start + 1.5).door(CLOSED)
    ride.at(start + 2.6)
    return ride


def arrive(ride, start, code=None):
    ride.at(start).door(OPENING)
    ride.at(start + 1.5).door(OPEN)
    if code is not None:
        ride.at(start + 2.).beacon(code, rssi=-58)
    return ride


def test_normal_ride_4f_to_3f(building):
    ride = Ride(building)
    assert (ride.s.zone, ride.s.floor, ride.s.confidence) == (ZONE_OUTSIDE, '4F', 'ASSUMED')
    ride.at(1.).beacon(4)
    assert ride.s.confidence == 'BEACON'
    board(ride)
    assert ride.s.zone == ZONE_IN_ELEVATOR and ride.s.floor == '4F'
    depart(ride, '3F')
    assert (ride.s.floor, ride.s.transit_from, ride.s.transit_to, ride.s.confidence) == \
        ('', '4F', '3F', 'TRANSIT')
    assert ride.at(24.).beacon(4) == 'ignored_in_transit'  # still near 4F while moving
    arrive(ride, 30., code=3)
    assert (ride.s.floor, ride.s.confidence, ride.s.zone) == ('3F', 'BEACON', ZONE_IN_ELEVATOR)
    ride.at(34.).position(past_hall=True, enclosed=False)
    assert ride.s.zone == ZONE_OUTSIDE
    ride.at(36.).m.on_localization(True, 36.)
    assert ride.s.confidence == 'CONFIRMED'
    assert ride.events() == ['FLOOR_BEACON', 'ENTERED_ELEVATOR', 'DEPARTED', 'ARRIVED',
                             'EXITED_ELEVATOR', 'FLOOR_CONFIRMED']


def test_duplicate_reordered_and_stale_beacons_are_counted_not_used(building):
    ride = Ride(building)
    ride.at(1.)
    assert ride.beacon(4, sequence=10) == 'ok'
    assert ride.beacon(4, sequence=10) == 'duplicate'
    assert ride.beacon(4, sequence=9) == 'out_of_order'
    assert ride.beacon(4, sequence=11, stamp=-2.) == 'stale'
    assert ride.beacon(4, sequence=12, stamp=3.) == 'stale'  # from the future
    assert ride.beacon(4, sequence=0) == 'out_of_order'      # wrapped backwards within the window
    assert (ride.s.duplicate_count, ride.s.out_of_order_count, ride.s.stale_count) == (1, 2, 2)
    ride.at(10.)
    assert ride.beacon(4, sequence=0) == 'ok'                # device restart after the window
    assert ride.beacon(4, sequence=1) == 'ok'
    assert ride.beacon(4, sequence=255, stamp=10.) == 'out_of_order'


def test_other_floor_beacons_outside_the_car_never_change_the_floor(building):
    ride = Ride(building)
    ride.at(1.)
    assert ride.beacon(5, rssi=-85) == 'weak'
    assert ride.beacon(5, rssi=-60) == 'conflict'
    assert ride.s.floor == '4F' and ride.s.floor_conflict
    assert ride.beacon(99) == 'unknown'
    ride.at(4.).m.tick(4.)
    assert ride.s.floor_conflict
    ride.at(7.)
    assert not ride.s.floor_conflict
    assert ride.events() == ['FLOOR_CONFLICT', 'UNKNOWN_BEACON']


def test_unexpected_stop_keeps_the_target_and_rides_again(building):
    ride = Ride(building, '3F')
    board(ride)
    depart(ride, '5F')
    arrive(ride, 28., code=4)  # someone called 4F
    assert (ride.s.floor, ride.s.confidence, ride.m.car_target) == ('4F', 'BEACON', '5F')
    depart(ride, '5F', start=40.)
    assert (ride.s.transit_from, ride.s.transit_to) == ('4F', '5F')
    arrive(ride, 50., code=5)
    assert (ride.s.floor, ride.m.car_target) == ('5F', '')
    assert ride.events() == ['ENTERED_ELEVATOR', 'DEPARTED', 'ARRIVED_OTHER_FLOOR', 'DEPARTED', 'ARRIVED']


def test_stop_at_a_pass_floor_keeps_riding_to_the_target(building):
    ride = Ride(building, '5F')
    board(ride)
    depart(ride, '3F')
    arrive(ride, 28., code=7)  # Someone called 7F (beacon installed, no map) first.
    assert (ride.s.confidence, ride.s.transit_to, ride.s.floor) == ('TRANSIT', '3F', '')
    ride.at(40.)  # Long after arrival_beacon_timeout_s: nothing is assumed.
    assert ride.s.confidence == 'TRANSIT'
    ride.at(41.).door(CLOSING)
    ride.at(42.5).door(CLOSED)
    assert ride.at(45.).beacon(6) == 'ignored_in_transit'  # Passing 6F, door closed.
    arrive(ride, 50., code=3)
    assert (ride.s.floor, ride.s.confidence) == ('3F', 'BEACON')
    assert ride.events() == ['ENTERED_ELEVATOR', 'DEPARTED', 'PASS_FLOOR_STOP', 'ARRIVED']


def test_pass_floor_beacon_outside_is_a_conflict_and_unknown_codes_report_once(building):
    ride = Ride(building)
    ride.at(1.)
    assert ride.beacon(2, rssi=-80) == 'weak'
    assert ride.beacon(2, rssi=-60) == 'conflict'
    assert ride.s.floor == '4F' and ride.s.floor_conflict
    assert [ride.beacon(15) for _ in range(3)] == ['unknown'] * 3
    assert ride.beacon(0, location=CAR) == 'ok'  # Car PRESENCE: liveness only.
    assert ride.s.floor == '4F'
    assert ride.events() == ['FLOOR_CONFLICT', 'UNKNOWN_BEACON']


def test_missing_arrival_beacon_is_assumed_and_blocks_a_clean_exit(building):
    ride = Ride(building)
    board(ride)
    depart(ride, '5F')
    arrive(ride, 30.)            # door opens, no beacon
    ride.at(33.4)
    assert ride.s.confidence == 'TRANSIT'
    ride.at(34.6)
    assert (ride.s.floor, ride.s.confidence, ride.m.car_target) == ('5F', 'ASSUMED', '5F')
    ride.at(36.).position(past_hall=True, enclosed=False)
    assert ride.s.zone == ZONE_OUTSIDE
    events = ride.events()
    assert events[-4:] == ['ARRIVED', 'ARRIVAL_UNCONFIRMED', 'EXITED_ELEVATOR', 'RULE_VIOLATION']
    ride.at(37.).m.on_localization(True, 37.)
    assert ride.s.confidence == 'ASSUMED'  # localization alone cannot confirm a copied map


def test_late_beacon_upgrades_an_assumed_arrival(building):
    ride = Ride(building)
    board(ride)
    depart(ride, '5F')
    arrive(ride, 30.)
    ride.at(35.)
    assert ride.s.confidence == 'ASSUMED'
    ride.at(35.5).beacon(5, rssi=-58)
    assert (ride.s.floor, ride.s.confidence, ride.m.car_target) == ('5F', 'BEACON', '')


def test_reopened_door_without_travel_assumes_the_departure_floor(building):
    ride = Ride(building)
    board(ride)
    depart(ride, '3F')           # closed for 1.5 s + 1.1 s
    ride.at(22.6).door(OPENING)  # door-open pressed: only 1.1 s closed after departure
    ride.at(23.).door(OPEN)
    ride.at(26.5)
    assert (ride.s.floor, ride.s.confidence, ride.m.car_target) == ('4F', 'ASSUMED', '3F')


def test_illegal_crossings_stop_until_the_operator_sets_the_floor(building):
    ride = Ride(building)
    ride.at(5.).position(in_car=True)  # door never seen open
    assert ride.s.zone == ZONE_UNKNOWN
    with pytest.raises(ValueError):
        ride.m.set_floor('4F', stationary=False)
    ride.m.set_floor('4F', stationary=True)
    assert (ride.s.zone, ride.s.floor, ride.s.confidence) == (ZONE_OUTSIDE, '4F', 'CONFIRMED')

    ride = Ride(building)
    board(ride)
    depart(ride, '3F')
    ride.at(25.).position(past_hall=True, enclosed=False)  # "exit" while travelling
    assert ride.s.zone == ZONE_UNKNOWN
    assert 'RULE_VIOLATION' in ride.events()

    ride = Ride(building)
    board(ride)
    with pytest.raises(ValueError):
        ride.m.set_floor('3F', stationary=True)  # not inside the car
    with pytest.raises(Exception):
        ride.m.set_floor('9F', stationary=True)


def test_position_without_lidar_agreement_does_not_switch_zone(building):
    ride = Ride(building)
    ride.at(10.).door(OPEN)
    ride.at(11.).position(in_car=True, enclosed=False)
    assert ride.s.zone == ZONE_OUTSIDE
    ride.at(12.).position(in_car=True, enclosed=True)
    assert ride.s.zone == ZONE_IN_ELEVATOR
    ride.at(13.).position(past_hall=True, enclosed=True)
    assert ride.s.zone == ZONE_IN_ELEVATOR
