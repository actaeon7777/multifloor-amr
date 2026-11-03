"""Floor/zone rules for explicit arrival confirmation (no ROS)."""

from dataclasses import replace

import pytest

from amr_elevator.door_detector import CLOSED, CLOSING, OPEN, OPENING
from amr_elevator.floor_state import (
    SOURCE_OPERATOR, SOURCE_VISION, SOURCE_SIM,
    ZONE_IN_ELEVATOR, ZONE_OUTSIDE, ZONE_UNKNOWN, FloorStateMachine,
)
from amr_stack.floor_config import load_floor_config


@pytest.fixture
def machine():
    return FloorStateMachine(load_floor_config(), initial_floor='4F')


def board(machine):
    machine.set_floor('4F', stationary=True)
    machine.on_door(OPEN, 1.)
    machine.on_position(True, False, True, 2.)


def depart(machine):
    board(machine)
    machine.on_car_target('3F')
    machine.on_door(CLOSED, 3.)
    machine.tick(4.)


def candidate(machine):
    depart(machine)
    machine.on_door(OPENING, 7.)
    machine.on_door(OPEN, 8.)
    machine.tick(9.)


def events(machine):
    return [e.event for e in machine.drain_events()]


def test_start_needs_operator_even_with_ready_localization(machine):
    assert (machine.state.floor, machine.state.confidence) == ('4F', 'ASSUMED')
    assert machine.state.candidate_id == 0
    machine.on_localization(True, 1.)
    assert machine.state.confidence == 'ASSUMED'
    machine.set_floor('3F', stationary=True)
    assert (machine.state.floor, machine.state.confidence) == ('3F', 'CONFIRMED')
    assert not machine.state.awaiting_confirm


def test_departure_needs_target_and_closed_interval(machine):
    board(machine)
    machine.on_door(CLOSED, 3.)
    machine.tick(10.)
    assert machine.state.confidence == 'CONFIRMED'
    machine.on_car_target('4F')
    machine.tick(11.)
    assert machine.state.confidence == 'CONFIRMED'
    machine.on_car_target('3F')
    machine.tick(12.)
    s = machine.state
    assert (s.floor, s.transit_from, s.transit_to, s.confidence) == ('', '4F', '3F', 'TRANSIT')
    assert not s.awaiting_confirm


def test_candidate_requires_settled_open_door_after_travel(machine):
    depart(machine)
    machine.on_door(OPENING, 7.)
    machine.tick(9.)
    assert machine.state.confidence == 'TRANSIT'
    machine.on_door(OPEN, 10.)
    machine.tick(10.9)
    assert machine.state.confidence == 'TRANSIT'
    machine.on_door(OPEN, 10.9)  # repeated observations do not reset the edge timer
    machine.tick(11.)
    s = machine.state
    assert (s.floor, s.confidence, s.awaiting_confirm) == ('3F', 'ASSUMED', True)
    assert (s.transit_from, s.transit_to) == ('', '')
    record = machine.drain_events()[-1]
    assert (record.event, record.from_floor, record.to_floor) == ('ARRIVAL_CANDIDATE', '4F', '3F')


def test_open_must_remain_open_for_settle_interval(machine):
    depart(machine)
    machine.on_door(OPEN, 8.)
    machine.on_door(CLOSING, 8.5)
    machine.tick(10.)
    assert machine.state.confidence == 'TRANSIT'
    machine.on_door(OPEN, 11.)
    machine.tick(11.9)
    assert machine.state.confidence == 'TRANSIT'
    machine.tick(12.)
    assert machine.state.awaiting_confirm


def test_door_only_reopened_proposes_origin_without_confirmation(machine):
    depart(machine)
    machine.on_door(OPENING, 4.5)
    machine.on_door(OPEN, 5.)
    machine.tick(6.)
    s = machine.state
    assert (s.floor, s.confidence, s.awaiting_confirm) == ('4F', 'ASSUMED', False)
    assert machine.car_target == '3F'
    assert events(machine)[-2:] == ['ARRIVAL_CANDIDATE', 'ARRIVED_OTHER_FLOOR']


@pytest.mark.parametrize('source, name', [(SOURCE_OPERATOR, 'OPERATOR'), (SOURCE_VISION, 'VISION'),
                                         (SOURCE_SIM, 'SIM')])
def test_accept_and_localize_after_exit(machine, source, name):
    candidate(machine)
    assert machine.on_arrival_confirm(source, '3F', machine.state.candidate_id, True, 9.) == 'accepted'
    assert machine.state.confidence == 'VERIFIED'
    assert not machine.state.awaiting_confirm and machine.car_target == '3F'
    assert machine.drain_events()[-1].detail == name
    machine.on_localization(True, 9.1)  # READY inside cannot confirm the floor
    assert machine.state.confidence == 'VERIFIED'
    machine.on_position(False, True, False, 9.2)
    assert machine.state.zone == ZONE_OUTSIDE
    assert machine.car_target == ''
    machine.on_localization(False, 9.3)
    machine.on_localization(True, 10.)
    assert machine.state.confidence == 'CONFIRMED'
    assert events(machine) == ['EXITED_ELEVATOR', 'FLOOR_CONFIRMED']


def test_reject_requires_new_closed_travel_then_accept_second_stop(machine):
    candidate(machine)
    assert machine.on_arrival_confirm(SOURCE_OPERATOR, '3F', machine.state.candidate_id, False, 9.) == 'rejected'
    s = machine.state
    assert (s.floor, s.transit_from, s.transit_to, s.confidence) == ('', '3F', '3F', 'TRANSIT')
    assert not s.awaiting_confirm and machine.t.closed_during_transit == 0.
    machine.on_door(OPEN, 10.)
    machine.tick(30.)
    assert machine.state.confidence == 'TRANSIT'
    machine.on_door(OPENING, 31.)  # no closed-door travel: cannot re-propose
    machine.on_door(OPEN, 32.)
    machine.tick(34.)
    assert machine.state.confidence == 'TRANSIT'
    machine.on_door(CLOSED, 35.)
    machine.on_door(OPEN, 36.)  # one second is insufficient
    machine.tick(38.)
    assert machine.state.confidence == 'TRANSIT'
    machine.on_door(CLOSED, 40.)
    machine.on_door(OPEN, 44.)
    machine.tick(45.)
    assert machine.state.awaiting_confirm
    assert machine.on_arrival_confirm(SOURCE_SIM, '3F', machine.state.candidate_id, True, 45.) == 'accepted'
    assert events(machine).count('ARRIVAL_CANDIDATE') == 2


@pytest.mark.parametrize('case', ['mismatch', 'not_awaiting', 'outside', 'long_closed'])
def test_ignored_confirmations_do_not_change_state(machine, case):
    candidate(machine)
    floor, now = '3F', 9.
    if case == 'mismatch':
        floor = '5F'
    elif case == 'not_awaiting':
        machine.state = replace(machine.state, awaiting_confirm=False)
    elif case == 'outside':
        machine.state = replace(machine.state, zone=ZONE_OUTSIDE)
    else:
        machine.on_door(CLOSED, 10.)
        now = 20.
    before = machine.state
    assert machine.on_arrival_confirm(SOURCE_OPERATOR, floor, machine.state.candidate_id, True, now) == 'ignored'
    assert machine.state == before


@pytest.mark.parametrize('door', [OPEN, OPENING])
def test_confirmation_accepts_currently_open_or_opening_door(machine, door):
    candidate(machine)
    machine.on_door(door, 9.1)
    assert machine.on_arrival_confirm(SOURCE_OPERATOR, '3F', machine.state.candidate_id, True, 9.2) == 'accepted'


def test_unconfirmed_candidate_departs_again_even_when_target_matches(machine):
    candidate(machine)
    machine.on_door(CLOSED, 10.)
    machine.tick(10.9)
    assert machine.state.awaiting_confirm
    machine.tick(11.)
    s = machine.state
    assert (s.floor, s.transit_from, s.transit_to, s.confidence) == ('', '3F', '3F', 'TRANSIT')
    assert not s.awaiting_confirm


def test_exit_at_unconfirmed_floor_is_violation(machine):
    candidate(machine)
    machine.on_position(False, True, False, 9.1)
    assert not machine.state.awaiting_confirm
    assert machine.drain_events()[-1].detail == 'left the car at an unconfirmed floor'
    machine.on_localization(True, 10.)
    assert machine.state.confidence == 'ASSUMED'


def test_crossing_without_open_door_and_set_floor_rules(machine):
    machine.on_position(True, False, True, 1.)
    assert machine.state.zone == ZONE_UNKNOWN
    with pytest.raises(ValueError, match='stationary'):
        machine.set_floor('4F', stationary=False)
    machine.set_floor('4F', stationary=True)
    assert (machine.state.zone, machine.state.confidence) == (ZONE_OUTSIDE, 'CONFIRMED')
    board(machine)
    with pytest.raises(ValueError, match='inside'):
        machine.set_floor('3F', stationary=True)
    with pytest.raises(ValueError):
        machine.on_car_target('9F')


@pytest.mark.parametrize('transit', [True, False])
def test_exit_in_transit_or_long_closed_is_violation(machine, transit):
    if transit:
        depart(machine)
    else:
        candidate(machine)
        machine.on_arrival_confirm(SOURCE_SIM, '3F', machine.state.candidate_id, True, 9.)
        machine.on_door(CLOSED, 10.)
    machine.on_position(False, True, False, 30.)
    assert (machine.state.zone, machine.state.confidence) == (ZONE_UNKNOWN, 'UNKNOWN')
    assert not machine.state.awaiting_confirm


def test_position_needs_lidar_agreement(machine):
    machine.on_door(OPEN, 1.)
    machine.on_position(True, False, False, 2.)
    assert machine.state.zone == ZONE_OUTSIDE
    machine.on_position(True, False, True, 3.)
    assert machine.state.zone == ZONE_IN_ELEVATOR
    machine.on_position(False, True, True, 4.)
    assert machine.state.zone == ZONE_IN_ELEVATOR


@pytest.mark.parametrize('door', [CLOSED, CLOSING])
@pytest.mark.parametrize('accepted', [True, False])
def test_recent_open_door_does_not_allow_confirmation_after_closure(machine, door, accepted):
    candidate(machine)
    machine.on_door(door, 9.1)
    before = machine.state
    assert machine.on_arrival_confirm(SOURCE_OPERATOR, '3F', before.candidate_id, accepted, 9.2) == 'ignored'
    assert machine.state == before and machine.car_target == '3F'


@pytest.mark.parametrize('reopen_at', [11.5, 15.])
def test_verified_floor_needs_new_confirmation_after_door_closure(machine, reopen_at):
    candidate(machine)
    first_id = machine.state.candidate_id
    assert machine.on_arrival_confirm(SOURCE_OPERATOR, '3F', first_id, True, 9.) == 'accepted'
    assert machine.car_target == '3F'
    machine.on_door(CLOSED, 10.)
    machine.on_position(True, False, True, 10.5, fully_inside=True)
    machine.tick(10.9)
    assert machine.state.confidence == 'VERIFIED'
    machine.tick(11.)
    assert (machine.state.confidence, machine.state.transit_from, machine.state.transit_to) == (
        'TRANSIT', '3F', '3F')
    assert not machine.state.awaiting_confirm
    machine.on_door(OPEN, reopen_at)
    machine.tick(reopen_at + 1.)
    assert machine.state.confidence == 'ASSUMED' and machine.state.awaiting_confirm
    assert machine.state.candidate_id == first_id + 1
    # A delayed answer to the previous stop cannot accept or reject this one.
    for accepted in (True, False):
        assert machine.on_arrival_confirm(SOURCE_SIM, '3F', first_id, accepted, reopen_at + 1.) == 'ignored'
        assert machine.state.awaiting_confirm
    assert machine.on_arrival_confirm(
        SOURCE_SIM, '3F', machine.state.candidate_id, True, reopen_at + 1.) == 'accepted'
    assert machine.state.confidence == 'VERIFIED'


def test_verified_straddling_body_does_not_depart_and_can_exit(machine):
    candidate(machine)
    machine.on_arrival_confirm(SOURCE_SIM, '3F', machine.state.candidate_id, True, 9.)
    machine.on_door(CLOSED, 10.)  # a person near the door panel creates a false closure
    for now in (10., 11., 12.):
        machine.on_position(True, False, True, now, fully_inside=False)
        machine.tick(now)
        assert machine.state.zone == ZONE_IN_ELEVATOR and machine.state.confidence == 'VERIFIED'
    machine.on_door(OPEN, 12.1)
    machine.on_position(False, True, False, 12.2)
    assert machine.state.zone == ZONE_OUTSIDE and machine.state.confidence == 'VERIFIED'
    assert machine.car_target == ''
    assert 'RULE_VIOLATION' not in events(machine)


@pytest.mark.parametrize('in_car, past_hall, enclosed', [(True, False, True), (True, False, False),
                                                       (True, True, True), (False, False, True)])
def test_verified_departure_uses_fresh_whole_body_evidence(machine, in_car, past_hall, enclosed):
    candidate(machine)
    machine.on_arrival_confirm(SOURCE_SIM, '3F', machine.state.candidate_id, True, 9.)
    machine.on_door(CLOSED, 10.)
    machine.on_position(in_car, past_hall, enclosed, 10.5, fully_inside=True)
    machine.tick(10.9)
    assert machine.state.confidence == 'VERIFIED'
    machine.tick(11.)
    assert (machine.state.confidence, machine.state.transit_from, machine.state.transit_to) == (
        'TRANSIT', '3F', '3F')


def test_legacy_position_call_defaults_to_not_fully_inside(machine):
    candidate(machine)
    machine.on_arrival_confirm(SOURCE_SIM, '3F', machine.state.candidate_id, True, 9.)
    machine.on_door(CLOSED, 10.)
    machine.on_position(True, False, True, 10.5)
    machine.tick(11.)
    assert machine.state.confidence == 'VERIFIED'
    machine.on_position(True, False, True, 11.1, fully_inside=True)
    machine.tick(11.1)
    assert machine.state.confidence == 'TRANSIT'


@pytest.mark.parametrize('fully_inside', [False, True])
def test_verified_departure_when_position_reports_stop(machine, fully_inside):
    candidate(machine)
    machine.on_arrival_confirm(SOURCE_SIM, '3F', machine.state.candidate_id, True, 9.)
    machine.on_position(True, False, True, 9., fully_inside=fully_inside)
    machine.on_door(CLOSED, 10.)
    machine.tick(11.)  # no further position reports for two seconds
    assert machine.state.confidence == 'TRANSIT'


def test_stale_not_fully_inside_position_uses_configured_timeout():
    machine = FloorStateMachine(load_floor_config(), initial_floor='4F', position_stale_s=2.)
    candidate(machine)
    machine.on_arrival_confirm(SOURCE_SIM, '3F', machine.state.candidate_id, True, 9.)
    machine.on_door(CLOSED, 10.)
    machine.on_position(True, False, True, 10., fully_inside=False)
    machine.tick(12.)  # exactly the timeout is still fresh
    assert machine.state.confidence == 'VERIFIED'
    machine.tick(12.01)
    assert machine.state.confidence == 'TRANSIT'


def test_verified_state_without_any_position_report_can_depart(machine):
    machine.state = replace(machine.state, zone=ZONE_IN_ELEVATOR, confidence='VERIFIED')
    machine.on_car_target('4F')
    machine.on_door(CLOSED, 10.)
    machine.tick(12.)
    assert machine.state.confidence == 'TRANSIT'


def test_candidate_ids_include_reopened_origin_and_survive_operator_reset(machine):
    depart(machine)
    assert machine.state.candidate_id == 0
    machine.on_door(OPEN, 4.5)
    machine.tick(5.5)
    assert machine.state.floor == '4F' and not machine.state.awaiting_confirm
    assert machine.state.candidate_id == 1
    machine.on_door(CLOSED, 6.)
    machine.tick(7.)
    machine.on_door(OPEN, 10.)
    machine.tick(11.)
    assert machine.state.candidate_id == 2 and machine.state.awaiting_confirm
    machine.on_arrival_confirm(SOURCE_OPERATOR, '3F', 2, True, 11.)
    machine.on_position(False, True, False, 11.1)
    machine.set_floor('4F', stationary=True)
    assert machine.state.candidate_id == 2 and machine.car_target == ''
    candidate(machine)
    assert machine.state.candidate_id == 3


def test_violation_clears_car_target(machine):
    depart(machine)
    assert machine.car_target == '3F'
    machine.on_position(False, True, False, 4.1)
    assert machine.state.zone == ZONE_UNKNOWN
    assert machine.car_target == '' and not machine.state.awaiting_confirm


@pytest.fixture
def vision_machine():
    return FloorStateMachine(load_floor_config(), initial_floor='4F', vision_wait_s=1.5)


@pytest.mark.parametrize('floor', ['3F', '4F', '5F', '2F'])
def test_camera_candidate_asks_guessed_building_or_pass_floor(vision_machine, floor):
    m = vision_machine
    depart(m)
    m.on_door(OPEN, 8.)
    assert m.on_vision_guess(floor, 8.5) == 'stored'
    assert m.tick(8.9).confidence == 'TRANSIT'
    s = m.tick(9.)
    assert (s.floor, s.confidence, s.awaiting_confirm, s.candidate_source) == (
        floor, 'ASSUMED', True, 'VISION')
    record = next(e for e in m.drain_events() if e.event == 'ARRIVAL_CANDIDATE')
    assert record.to_floor == floor and 'VISION' in record.detail


def test_camera_can_arrive_after_settle_before_timeout(vision_machine):
    m = vision_machine
    candidate(m)  # OPEN at 8; settled at 9, but still waiting for camera
    assert m.state.confidence == 'TRANSIT'
    m.on_door(OPEN, 9.2)
    assert m.t.arrival_wait_since == 8.
    assert m.on_vision_guess('4F', 9.4) == 'stored'
    assert m.tick(9.4).floor == '4F'


@pytest.mark.parametrize('floor', ['', '9F'])
def test_unknown_camera_report_falls_back_without_waiting_for_timeout(vision_machine, floor):
    m = vision_machine
    candidate(m)
    assert m.on_vision_guess(floor, 9.2) == 'unknown'
    s = m.tick(9.2)
    assert (s.floor, s.awaiting_confirm, s.candidate_source) == ('3F', True, 'TARGET')


def test_missing_or_late_camera_report_falls_back_at_timeout(vision_machine):
    m = vision_machine
    candidate(m)
    assert m.tick(10.49).confidence == 'TRANSIT'
    s = m.tick(10.5)
    assert (s.floor, s.awaiting_confirm, s.candidate_source) == ('3F', True, 'TARGET')
    assert m.on_vision_guess('4F', 10.6) == 'ignored'
    assert m.tick(11.) == s


def test_camera_after_deadline_cannot_win_by_arriving_before_tick(vision_machine):
    m = vision_machine
    candidate(m)
    assert m.on_vision_guess('4F', 10.51) == 'ignored'
    assert m.tick(10.6).candidate_source == 'TARGET'


def test_camera_outside_car_or_before_current_open_edge_is_ignored(vision_machine):
    m = vision_machine
    assert m.on_vision_guess('4F', 0.) == 'ignored'
    board(m)
    assert m.on_vision_guess('4F', 2.) == 'ignored'
    m.on_car_target('3F')
    m.on_door(CLOSED, 3.)
    m.tick(4.)
    assert m.on_vision_guess('4F', 7.) == 'ignored'
    m.on_door(OPEN, 8.)
    assert m.on_vision_guess('4F', 7.9) == 'ignored'
    assert m.tick(10.5).candidate_source == 'TARGET'


def test_camera_report_does_not_survive_interrupted_opening(vision_machine):
    m = vision_machine
    depart(m)
    m.on_door(OPEN, 8.)
    assert m.on_vision_guess('4F', 8.5) == 'stored'
    m.on_door(CLOSING, 8.6)
    assert m.tick(11.).confidence == 'TRANSIT'
    assert m.on_vision_guess('4F', 11.) == 'ignored'
    m.on_door(OPEN, 12.)
    assert m.on_vision_guess('4F', 8.5) == 'ignored'
    assert m.tick(13.).confidence == 'TRANSIT'
    assert m.tick(14.5).candidate_source == 'TARGET'


def test_camera_ignored_when_door_only_reopened(vision_machine):
    m = vision_machine
    depart(m)
    m.on_door(OPEN, 4.5)
    assert m.on_vision_guess('5F', 5.) == 'ignored'
    s = m.tick(5.5)
    assert (s.floor, s.awaiting_confirm, s.candidate_source) == ('4F', False, '')


def test_disabled_camera_preserves_immediate_target_candidate(machine):
    depart(machine)
    machine.on_door(OPEN, 8.)
    assert machine.on_vision_guess('4F', 8.5) == 'ignored'
    s = machine.tick(9.)
    assert (s.floor, s.awaiting_confirm, s.candidate_source) == ('3F', True, 'TARGET')


@pytest.mark.parametrize('floor', ['4F', '2F'])
def test_accept_non_target_then_depart_and_ask_new_candidate(vision_machine, floor):
    m = vision_machine
    candidate(m)
    m.on_vision_guess(floor, 9.2)
    first = m.tick(9.2)
    assert m.on_arrival_confirm(SOURCE_OPERATOR, floor, first.candidate_id, True, 9.3) == 'accepted'
    assert (m.state.floor, m.state.confidence, m.state.candidate_source) == (floor, 'VERIFIED', '')
    assert not m.state.awaiting_confirm and m.state.floor != m.car_target == '3F'
    m.on_door(CLOSED, 10.)
    m.on_position(True, False, True, 10.5, fully_inside=True)
    assert m.tick(11.).confidence == 'TRANSIT'
    m.on_door(OPEN, 14.)
    # Last stop's guess must not be used at the new opening.
    assert m.tick(15.).confidence == 'TRANSIT'
    second = m.tick(16.5)
    assert second.floor == '3F' and second.candidate_source == 'TARGET'
    assert second.candidate_id == first.candidate_id + 1 and second.awaiting_confirm
    assert m.on_arrival_confirm(SOURCE_OPERATOR, '3F', first.candidate_id, False, 16.6,
                                actual_floor='4F') == 'ignored'


def test_verified_non_target_departs_without_whole_body_evidence(vision_machine):
    """Review 2026-11-02: the mission never exits at a non-target stop, so the car can leave."""
    m = vision_machine
    candidate(m)
    m.on_vision_guess('5F', 9.2)
    first = m.tick(9.2)
    assert m.on_arrival_confirm(SOURCE_OPERATOR, '5F', first.candidate_id, True, 9.3) == 'accepted'
    m.on_door(CLOSED, 10.)
    m.on_position(True, False, True, 10.5, fully_inside=False)   # fresh report, body not "fully inside"
    assert m.tick(11.).confidence == 'TRANSIT'


@pytest.mark.parametrize('actual_floor', ['3F', '5F', '2F'])
def test_no_with_known_actual_floor_verifies_correction(vision_machine, actual_floor):
    m = vision_machine
    candidate(m)
    m.on_vision_guess('4F', 9.2)
    s = m.tick(9.2)
    assert m.on_arrival_confirm(SOURCE_OPERATOR, s.floor, s.candidate_id, False, 9.3,
                                actual_floor=actual_floor) == 'accepted'
    assert (m.state.floor, m.state.confidence, m.state.candidate_source) == (actual_floor, 'VERIFIED', '')
    assert not m.state.awaiting_confirm and m.car_target == '3F'
    e = m.drain_events()[-1]
    assert (e.event, e.from_floor, e.to_floor, e.detail) == (
        'ARRIVED', '4F', actual_floor, f'OPERATOR corrected 4F->{actual_floor}')


@pytest.mark.parametrize('actual_floor', ['', '9F'])
def test_no_without_known_actual_floor_keeps_riding(vision_machine, actual_floor):
    m = vision_machine
    candidate(m)
    m.on_vision_guess('4F', 9.2)
    s = m.tick(9.2)
    assert m.on_arrival_confirm(SOURCE_OPERATOR, s.floor, s.candidate_id, False, 9.3,
                                actual_floor=actual_floor) == 'rejected'
    assert (m.state.floor, m.state.confidence, m.state.candidate_source) == ('', 'TRANSIT', '')
    assert not m.state.awaiting_confirm and m.car_target == '3F'
    assert m.tick(30.).confidence == 'TRANSIT'


@pytest.mark.parametrize('case', ['contradiction', 'closed', 'closing', 'wrong_id', 'wrong_floor', 'outside'])
def test_correction_preserves_confirmation_guards(machine, case):
    candidate(machine)
    floor, candidate_id, actual = '3F', machine.state.candidate_id, '4F'
    if case == 'contradiction':
        actual = floor
    elif case == 'closed':
        machine.on_door(CLOSED, 9.1)
    elif case == 'closing':
        machine.on_door(CLOSING, 9.1)
    elif case == 'wrong_id':
        candidate_id += 1
    elif case == 'wrong_floor':
        floor = '5F'
    else:
        machine.state = replace(machine.state, zone=ZONE_OUTSIDE)
    before = machine.state
    assert machine.on_arrival_confirm(SOURCE_OPERATOR, floor, candidate_id, False, 9.2,
                                      actual_floor=actual) == 'ignored'
    assert machine.state == before



@pytest.fixture
def auto_machine():
    return FloorStateMachine(load_floor_config(), initial_floor='4F', vision_wait_s=1.5, vision_auto=True)


def test_confident_guess_of_the_target_verifies_without_a_question(auto_machine):
    m = auto_machine
    candidate(m)
    assert m.on_vision_guess('3F', 9.2, .95, .07, ('3F', '4F', '5F')) == 'stored'
    s = m.tick(9.2)
    assert (s.floor, s.confidence, s.awaiting_confirm, s.candidate_source) == ('3F', 'VERIFIED', False, 'VISION_AUTO')
    assert s.floor == m.car_target                      # the mission alights on this
    assert any(e.event == 'ARRIVED' and e.detail == 'VISION auto' for e in m.drain_events())


def test_confident_guess_of_another_floor_keeps_riding_and_departs(auto_machine):
    m = auto_machine
    candidate(m)
    m.on_vision_guess('5F', 9.2, .95, .07)
    s = m.tick(9.2)
    assert (s.floor, s.confidence, s.awaiting_confirm) == ('5F', 'VERIFIED', False)
    assert s.floor != m.car_target == '3F'             # not the target: the mission stays in the car
    m.on_door(CLOSED, 10.)
    assert m.tick(11.).confidence == 'TRANSIT' and m.state.transit_to == '3F'


@pytest.mark.parametrize('score,margin', [(.89, .07), (.95, .02)])
def test_unsure_guess_falls_back_to_the_question(auto_machine, score, margin):
    m = auto_machine
    candidate(m)
    m.on_vision_guess('3F', 9.2, score, margin)
    s = m.tick(9.2)
    assert (s.floor, s.confidence, s.awaiting_confirm, s.candidate_source) == ('3F', 'ASSUMED', True, 'VISION')


def test_automatic_mode_needs_a_known_guess(auto_machine):
    m = auto_machine
    candidate(m)
    m.on_vision_guess('', 9.2, .99, .5)
    s = m.tick(9.2)
    assert (s.confidence, s.awaiting_confirm, s.candidate_source) == ('ASSUMED', True, 'TARGET')


def test_confident_guess_without_automatic_mode_still_asks(vision_machine):
    m = vision_machine
    candidate(m)
    m.on_vision_guess('3F', 9.2, .99, .5)
    assert m.tick(9.2).awaiting_confirm



@pytest.mark.parametrize('references', [(), ('4F', '5F')])
def test_target_without_references_is_asked_not_assumed(auto_machine, references):
    """Review 2026-11-02 P1: an unreferenced floor resembling the target must not pass as it."""
    m = auto_machine
    candidate(m)
    m.on_vision_guess('3F', 9.2, .97, .1, references)
    s = m.tick(9.2)
    assert (s.confidence, s.awaiting_confirm, s.candidate_source) == ('ASSUMED', True, 'VISION')


def test_target_needs_references_for_every_floor_on_the_way():
    m = FloorStateMachine(load_floor_config(), initial_floor='5F', vision_wait_s=1.5, vision_auto=True)
    m.set_floor('5F', stationary=True)
    m.on_door(OPEN, 1.)
    m.on_position(True, False, True, 2.)
    m.on_car_target('3F')
    m.on_door(CLOSED, 3.)
    m.tick(4.)
    m.on_door(OPEN, 8.)
    m.tick(9.)
    m.on_vision_guess('3F', 9.2, .97, .1, ('3F', '5F'))       # 4F lies between and has no references
    assert m.tick(9.2).awaiting_confirm
    m2 = FloorStateMachine(load_floor_config(), initial_floor='5F', vision_wait_s=1.5, vision_auto=True)
    m2.set_floor('5F', stationary=True)
    m2.on_door(OPEN, 1.)
    m2.on_position(True, False, True, 2.)
    m2.on_car_target('3F')
    m2.on_door(CLOSED, 3.)
    m2.tick(4.)
    m2.on_door(OPEN, 8.)
    m2.tick(9.)
    m2.on_vision_guess('3F', 9.2, .97, .1, ('3F', '4F', '5F'))
    assert m2.tick(9.2).confidence == 'VERIFIED'


def test_target_after_a_rejected_stop_is_asked(auto_machine):
    """After "no / don't know" the origin is unknown, so the stops on the way are unbounded."""
    m = auto_machine
    candidate(m)
    m.on_vision_guess('5F', 9.2, .5, .0, ('3F', '4F', '5F'))     # unsure -> question
    first = m.tick(9.2)
    assert m.on_arrival_confirm(SOURCE_OPERATOR, first.floor, first.candidate_id, False, 9.3) == 'rejected'
    m.on_door(CLOSED, 10.)
    m.on_door(OPEN, 14.)
    m.tick(14.6)
    m.on_vision_guess('3F', 14.7, .97, .1, ('3F', '4F', '5F'))
    s = m.tick(15.1)   # settled (1.0 s default) with the confident target guess
    assert (s.floor, s.confidence, s.awaiting_confirm) == ('3F', 'ASSUMED', True)


def test_non_target_decision_needs_no_references(auto_machine):
    m = auto_machine
    candidate(m)
    m.on_vision_guess('5F', 9.2, .97, .1, ())
    s = m.tick(9.2)
    assert (s.floor, s.confidence, s.awaiting_confirm) == ('5F', 'VERIFIED', False)
