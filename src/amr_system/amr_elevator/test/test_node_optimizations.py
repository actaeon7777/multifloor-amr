"""Exercise node methods with fake state/publishers; never initialize or spin ROS."""

from array import array
import json
import math
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException

from amr_elevator import mission_node, monitor_node
from amr_elevator.door_detector import CLOSED, CLOSING, OPEN, OPENING, UNKNOWN, DoorDetector
from amr_elevator.geometry import compose
from amr_elevator.maneuver import travel_clearances
from amr_elevator.mission_node import FloorMissionManager
from amr_elevator.monitor_node import ElevatorMonitor


def test_feedback_changes_immediately_and_otherwise_at_most_once_per_second():
    clock = [0.]
    publications = []
    mission = SimpleNamespace(
        phase='BOARDING', detail='', floor='4F', room='', start_floor='1F',
        counters={}, timings={}, started=0., finished=False,
        handle=SimpleNamespace(publish_feedback=lambda msg: publications.append((clock[0], msg))),
    )
    node = SimpleNamespace(
        mission=mission, floor_state=None, door=SimpleNamespace(state=OPEN),
        last_status=None, last_feedback_time=-math.inf, now=lambda: clock[0], status_pub=Mock(),
    )
    for tick in range(61):
        clock[0] = tick / 20
        FloorMissionManager.publish_status(node)
    assert [t for t, _ in publications] == [0., 1., 2., 3.]
    assert node.status_pub.publish.call_count == 1  # elapsed alone never republishes latched status
    assert json.loads(node.status_pub.publish.call_args.args[0].data)['phase'] == 'BOARDING'

    clock[0] = 3.05
    mission.phase = 'IN_CAR_ALIGN'
    FloorMissionManager.publish_status(node)
    assert publications[-1][0] == 3.05
    assert publications[-1][1].phase == 'IN_CAR_ALIGN'
    assert node.status_pub.publish.call_count == 2

    clock[0] = 3.1
    FloorMissionManager.publish_status(node, force=True)
    assert len(publications) == 5  # force affects the latched topic only
    assert node.status_pub.publish.call_count == 3
    clock[0] = 4.1
    FloorMissionManager.publish_status(node)
    assert len(publications) == 6
    clock[0] = 4.15
    node.door.state = UNKNOWN
    FloorMissionManager.publish_status(node)
    assert len(publications) == 7  # non-phase changes in key also publish immediately
    mission.finished = True
    clock[0] = 10.
    FloorMissionManager.publish_status(node, force=True)
    assert len(publications) == 7
    node.mission = None
    FloorMissionManager.publish_status(node)
    assert len(publications) == 7


def test_clearance_reuses_scan_and_invalidates_on_new_message(monkeypatch):
    compute = Mock(wraps=travel_clearances)
    monkeypatch.setattr(mission_node, 'travel_clearances', compute)
    node = SimpleNamespace(scan=None, laser_pose=None, _clearance_scan=None, _clearances=(math.inf, math.inf),
                           get_logger=lambda: Mock())
    assert FloorMissionManager.clearance(node, False) == math.inf
    first = SimpleNamespace(ranges=[1., 2.], angle_min=0., angle_increment=math.pi)
    node.scan = first
    assert FloorMissionManager.clearance(node, False) == 0.
    compute.assert_not_called()
    node.laser_pose = (0., 0., 0.)
    for _ in range(20):
        assert FloorMissionManager.clearance(node, False) == pytest.approx(.68)
    assert compute.call_count == 1
    for _ in range(20):
        assert FloorMissionManager.clearance(node, True) == pytest.approx(1.68)
    assert compute.call_count == 1  # reversal reuses the same projection, with reverse clearance
    assert FloorMissionManager.clearance(node, False) == pytest.approx(.68)
    assert compute.call_count == 1
    node.scan = SimpleNamespace(ranges=[.4, 3.], angle_min=0., angle_increment=math.pi)
    for _ in range(20):
        assert FloorMissionManager.clearance(node, False) == pytest.approx(.08)
    assert compute.call_count == 2


def test_only_dynamic_tf_subscription_is_destroyed_after_success():
    dynamic, static = object(), object()
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=.1, y=.2), rotation=SimpleNamespace(x=0., y=0., z=1., w=0.)))
    node = SimpleNamespace(
        scan=None, laser_pose=None, p={'base_frame': 'base_footprint'},
        tf=SimpleNamespace(lookup_transform=Mock(side_effect=[TransformException('not ready'), transform])),
        tf_listener=SimpleNamespace(tf_sub=dynamic, tf_static_sub=static), destroy_subscription=Mock(),
    )
    message = LaserScan()
    message.header.frame_id = 'laser'
    FloorMissionManager.on_scan(node, message)
    assert node.laser_pose is None
    node.destroy_subscription.assert_not_called()
    FloorMissionManager.on_scan(node, message)
    assert node.laser_pose == pytest.approx((.1, .2, math.pi))
    node.destroy_subscription.assert_called_once_with(dynamic)
    assert node.tf_listener.tf_static_sub is static
    assert not hasattr(node.tf_listener, 'tf_sub')
    FloorMissionManager.on_scan(node, LaserScan())
    assert node.tf.lookup_transform.call_count == 2
    node.destroy_subscription.assert_called_once()


@pytest.mark.parametrize('cls', [ElevatorMonitor, FloorMissionManager])
def test_geometry_cache_keys_include_floor_and_elevator(cls):
    lookup = Mock(side_effect=lambda *key: object())
    node = SimpleNamespace(building=SimpleNamespace(elevator_geometry=lookup), elevator_id='E1',
                           _geometry_cache={}, current_floor=lambda: '1F')
    first = cls.geometry(node, '1F')
    assert cls.geometry(node, '1F') is first
    second = cls.geometry(node, '4F')
    assert second is not first and cls.geometry(node, '4F') is second
    if cls is ElevatorMonitor:
        assert cls.geometry(node) is first
    node.elevator_id = 'E2'
    assert cls.geometry(node, '1F') is not first
    assert lookup.call_count == 3
    assert [call.args for call in lookup.call_args_list] == [('1F', 'E1'), ('4F', 'E1'), ('1F', 'E2')]


def test_monitor_publishes_unknown_on_every_far_scan():
    detector = DoorDetector()
    detector.state = OPEN
    geometry = SimpleNamespace(car_polygon=[(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)],
                               hall_door=((0., -.55), (0., .55)))
    node = SimpleNamespace(
        laser_offset=(0., 0., 0.), p={'enclosure_near': 3., 'enclosure_ratio': .75},
        estimated_pose=lambda: (10., 0., 0.), geometry=lambda: geometry,
        machine=SimpleNamespace(state=SimpleNamespace(zone=0), on_door=Mock()),
        detector=detector, now=lambda: 1., elevator_id='E1', door_pub=Mock(),
    )
    message = LaserScan(range_min=.05, range_max=30.)
    message.ranges = array('f', [math.inf] * 3240)
    for _ in range(3):
        ElevatorMonitor.on_scan(node, message)
    assert node.door_pub.publish.call_count == 3
    assert node.machine.on_door.call_count == 3
    for call in node.door_pub.publish.call_args_list:
        reading = call.args[0]
        assert (reading.state, reading.open_ratio, reading.valid_beams) == (UNKNOWN, 0., 0)
        assert reading.header == message.header


@pytest.mark.parametrize('ranges', [[], [math.nan, math.inf, -math.inf, 0., .049, .05, 4., 4.01, 1.3]])
def test_car_projection_preserves_point_set_and_order(monkeypatch, ranges):
    fit = Mock(return_value=None)
    monkeypatch.setattr(monitor_node, 'fit_car', fit)
    monkeypatch.setattr(monitor_node, 'car_wall_segments', lambda geometry: ['walls'])
    node = SimpleNamespace(laser_offset=(.13, -.04, .27))
    base = (1.2, -.9, .5)
    message = SimpleNamespace(range_min=.05, angle_min=-1.7, angle_increment=.123)
    assert ElevatorMonitor.correct_in_car(node, base, message, np.asarray(ranges), object()) == base
    laser = compose(base, node.laser_offset)
    expected = []
    for index, value in enumerate(ranges):
        if math.isfinite(value) and message.range_min <= value <= 4.:
            angle = laser[2] + message.angle_min + index * message.angle_increment
            expected.append((laser[0] + value * math.cos(angle), laser[1] + value * math.sin(angle)))
    points = fit.call_args.args[0]
    assert points.shape == (len(expected), 2)
    np.testing.assert_allclose(points, np.asarray(expected).reshape(-1, 2), rtol=1e-14, atol=1e-14)
    assert fit.call_args.args[1:] == (['walls'], base[:2])
    assert fit.call_args.kwargs == {'max_distance': .3}


def test_request_stamps_and_increments_sequence():
    from builtin_interfaces.msg import Time
    from amr_elevator_interfaces.msg import ElevatorRequest

    stamp = Time(sec=42)
    node = SimpleNamespace(sequence=0, request_pub=Mock(), get_logger=lambda: Mock(),
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp)))
    FloorMissionManager.request_elevator(node, ElevatorRequest.KIND_CALL, '4F', '3F', ElevatorRequest.DIR_DOWN)
    FloorMissionManager.request_elevator(node, ElevatorRequest.KIND_CAR_FLOOR, target_floor='3F')
    FloorMissionManager.request_elevator(node, ElevatorRequest.KIND_CANCEL)
    messages = [call.args[0] for call in node.request_pub.publish.call_args_list]
    assert [m.sequence for m in messages] == [1, 2, 3]
    assert all(m.header.stamp == stamp for m in messages)
    assert (messages[0].start_floor, messages[0].target_floor, messages[0].direction) == ('4F', '3F', 2)
    assert (messages[2].start_floor, messages[2].target_floor, messages[2].direction) == ('', '', 0)


def test_call_reentry_requests_and_announces_once_each_time():
    from amr_elevator_interfaces.msg import ElevatorRequest
    from amr_elevator.door_detector import CLOSED

    node = SimpleNamespace(
        mission=SimpleNamespace(floor='3F', start_floor='4F'),
        current_floor=lambda: '4F', door_state=lambda: CLOSED,
        building=SimpleNamespace(floor=lambda name: SimpleNamespace(name=name, level=int(name[0]))),
        request_elevator=Mock(), announce=Mock(), set_phase=Mock(),
        _announced={'call': 100., 'waiting': 100.},
    )
    for _ in range(2):
        FloorMissionManager.phase_call_elevator(node)
    assert node.request_elevator.call_count == 2
    assert node.request_elevator.call_args.args == (ElevatorRequest.KIND_CALL, '4F', '3F', ElevatorRequest.DIR_DOWN)
    assert [c.args[0] for c in node.announce.call_args_list] == ['call', 'waiting', 'call', 'waiting']
    node.door_state = lambda: OPEN
    FloorMissionManager.phase_call_elevator(node)
    assert node.request_elevator.call_count == 2
    node.set_phase.assert_called_with('WAIT_DOOR_OPEN', 'car already here')


@pytest.mark.parametrize('confidence, advances', [('ASSUMED', False), ('VERIFIED', True), ('CONFIRMED', True)])
def test_riding_waits_for_confirmation_but_keeps_overall_timeout(confidence, advances):
    node = SimpleNamespace(
        mission=SimpleNamespace(floor='3F', counters={}, phase_since=0.),
        floor_state=SimpleNamespace(floor='3F', confidence=confidence, awaiting_confirm=not advances),
        door_open_since=0., door_state=lambda: OPEN, now=lambda: 10., elapsed=lambda: 10.,
        p={'ride_timeout': 180.}, renew_lease=Mock(), publish_twist=Mock(), set_phase=Mock(), fail=Mock(),
    )
    node.at_confirmed_target = MethodType(FloorMissionManager.at_confirmed_target, node)
    FloorMissionManager.phase_riding(node)
    assert node.set_phase.called is advances
    node.fail.assert_not_called()
    if not advances:
        node.elapsed = lambda: 181.
        FloorMissionManager.phase_riding(node)
        node.fail.assert_called_once_with('RIDE_TIMEOUT', 'did not reach the target floor')


def test_select_floor_immediately_rides():
    from amr_elevator_interfaces.msg import ElevatorRequest

    node = SimpleNamespace(mission=SimpleNamespace(floor='3F'), renew_lease=Mock(), publish_twist=Mock(),
                           request_elevator=Mock(), set_phase=Mock())
    FloorMissionManager.phase_select_floor(node)
    node.request_elevator.assert_called_once_with(ElevatorRequest.KIND_CAR_FLOOR, target_floor='3F')
    node.set_phase.assert_called_once_with('RIDING')


@pytest.mark.parametrize('outcome', ['SUCCEEDED', 'CANCELED', 'FAILED'])
def test_finish_withdraws_request_exactly_once(outcome):
    from amr_elevator_interfaces.msg import ElevatorRequest

    mission = SimpleNamespace(finished=False, phase='RIDING', phase_since=0., timings={},
                              handle=Mock(is_cancel_requested=True), completion=Mock())
    node = SimpleNamespace(mission=mission, now=lambda: 1., request_elevator=Mock(),
                           cancel_nav=Mock(), publish_status=Mock())
    FloorMissionManager.finish(node, outcome)
    FloorMissionManager.finish(node, outcome)
    node.request_elevator.assert_called_once_with(ElevatorRequest.KIND_CANCEL)
    mission.completion.set_result.assert_called_once()


def test_monitor_request_validates_floor_and_confirmation_logs_verdict():
    from amr_elevator_interfaces.msg import ElevatorRequest, FloorArrivalConfirm
    from amr_elevator.floor_state import FloorStateMachine
    from amr_stack.floor_config import load_floor_config

    logger = Mock()
    node = SimpleNamespace(machine=FloorStateMachine(load_floor_config()), get_logger=lambda: logger,
                           now=lambda: 1.)
    ElevatorMonitor.on_request(node, ElevatorRequest(kind=ElevatorRequest.KIND_CALL, target_floor='3F'))
    assert node.machine.car_target == ''
    ElevatorMonitor.on_request(node, ElevatorRequest(kind=ElevatorRequest.KIND_CAR_FLOOR, target_floor='3F'))
    assert node.machine.car_target == '3F'
    ElevatorMonitor.on_request(node, ElevatorRequest(kind=ElevatorRequest.KIND_CAR_FLOOR, target_floor='9F'))
    assert node.machine.car_target == '3F'
    logger.warning.assert_called_once()
    ElevatorMonitor.on_arrival_confirm(node, FloorArrivalConfirm(source=1, floor='4F', accepted=True))
    logger.info.assert_called_once_with('Arrival confirmation 4F: ignored')


def test_cancel_request_transport_failure_does_not_block_cleanup():
    mission = SimpleNamespace(finished=False, phase='RIDING', phase_since=0., timings={},
                              handle=Mock(is_cancel_requested=True), completion=Mock())
    logger = Mock()
    node = SimpleNamespace(mission=mission, now=lambda: 1.,
                           request_elevator=Mock(side_effect=RuntimeError('publisher unavailable')),
                           cancel_nav=Mock(), publish_status=Mock(), get_logger=lambda: logger)
    FloorMissionManager.finish(node, 'CANCELED')
    assert mission.finished and node.maneuver is None
    node.cancel_nav.assert_called_once()
    mission.handle.canceled.assert_called_once()
    mission.completion.set_result.assert_called_once()
    logger.warning.assert_called_once()


@pytest.fixture
def ride_node():
    """Step mission phases with real transitions but no ROS node or motion transport."""
    from amr_elevator_interfaces.msg import ElevatorDoorState, FloorState

    clock = [0.]
    node = SimpleNamespace(
        mission=SimpleNamespace(phase='RIDING', phase_since=0., floor='3F', counters={}, timings={}),
        floor_state=FloorState(floor='', confidence='TRANSIT', zone=FloorState.ZONE_IN_ELEVATOR),
        door=ElevatorDoorState(), door_open_since=None, now=lambda: clock[0],
        maneuver=None, pose=None, renew_lease=Mock(), publish_twist=Mock(),
        request_elevator=Mock(), announce=Mock(), publish_status=Mock(), fail=Mock(),
        get_logger=lambda: Mock(), p={'ride_timeout': 180., 'boarding_timeout': 30.},
    )
    for name in ('at_confirmed_target', 'set_phase', 'start_alighting', 'count', 'elapsed', 'on_door', 'door_state'):
        setattr(node, name, MethodType(getattr(FloorMissionManager, name), node))
    return node, clock


@pytest.mark.parametrize('floor, confidence, expected', [
    ('3F', 'VERIFIED', True), ('3F', 'CONFIRMED', True), ('3F', 'ASSUMED', False),
    ('4F', 'VERIFIED', False), ('', 'TRANSIT', False),
])
def test_confirmed_target_checks_floor_and_confidence(ride_node, floor, confidence, expected):
    node, _ = ride_node
    node.floor_state.floor, node.floor_state.confidence = floor, confidence
    assert node.at_confirmed_target() is expected
    node.floor_state = None
    assert not node.at_confirmed_target()


@pytest.mark.parametrize('phase', ['SWITCH_MAP', 'ALIGHTING', 'RETURN_TO_CAR'])
@pytest.mark.parametrize('floor, confidence', [('', 'TRANSIT'), ('3F', 'ASSUMED'), ('4F', 'VERIFIED')])
def test_exit_preparation_rechecks_floor_and_resets_attempt_state(ride_node, phase, floor, confidence):
    from amr_elevator_interfaces.msg import ElevatorDoorState

    node, clock = ride_node
    node.set_phase(phase)
    node.floor_state.floor, node.floor_state.confidence = floor, confidence
    node.mission.counters.update(_loaded_map='3F', alighting_attempts=2, _clear_since=-10.,
                                 _blocked_since=-10., _return_arrived=True, _return_wait_since=-10.,
                                 _exit_started_at=-10., _alight_announced=True)
    node.maneuver = object()  # stale movement must not survive the phase transition
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    getattr(FloorMissionManager, 'phase_' + phase.lower())(node)
    assert node.mission.phase == 'RIDING' and node.maneuver is None
    node.publish_twist.assert_called_once_with()
    assert node.mission.counters == {'_loaded_map': '3F', 'alighting_attempts': 2}
    clock[0] = 1.
    node.floor_state.floor, node.floor_state.confidence = '3F', 'VERIFIED'
    FloorMissionManager.phase_riding(node)
    assert node.mission.phase == 'SWITCH_MAP'
    FloorMissionManager.phase_switch_map(node)  # cached map, without another map service call
    assert node.mission.phase == 'ALIGHTING' and node.maneuver is None
    assert node.mission.counters == {'_loaded_map': '3F', 'alighting_attempts': 2}


def test_return_after_started_exit_clears_maneuver_and_clearance_timers(ride_node):
    from amr_elevator_interfaces.msg import ElevatorDoorState

    node, _ = ride_node
    node.set_phase('ALIGHTING')
    node.mission.counters.update(_exit_started=True, _exit_started_at=-10., _clear_since=-10.,
                                 _blocked_since=-10., alighting_attempts=1, _loaded_map='3F')
    node.maneuver = object()
    node.geometry = Mock(return_value=SimpleNamespace(car_pose=(0., 0., 0.)))
    node.p['max_alighting_attempts'] = 3
    FloorMissionManager.return_to_car(node, 'door closing', already_in_car=True)
    assert node.mission.phase == 'RETURN_TO_CAR' and node.maneuver is None
    assert not any(k in node.mission.counters for k in (
        '_exit_started', '_exit_started_at', '_clear_since', '_blocked_since'))
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    FloorMissionManager.phase_return_to_car(node)
    assert node.mission.phase == 'RIDING'
    node.floor_state.floor, node.floor_state.confidence = '3F', 'VERIFIED'
    FloorMissionManager.phase_riding(node)
    FloorMissionManager.phase_switch_map(node)
    assert node.mission.phase == 'ALIGHTING' and node.maneuver is None
    assert node.mission.counters['alighting_attempts'] == 2


@pytest.mark.parametrize('confidence', ['VERIFIED', 'CONFIRMED'])
def test_return_with_confirmed_target_can_retry_exit(ride_node, confidence):
    from amr_elevator_interfaces.msg import ElevatorDoorState

    node, _ = ride_node
    node.set_phase('RETURN_TO_CAR')
    node.mission.counters['_return_arrived'] = True
    node.floor_state.floor, node.floor_state.confidence = '3F', confidence
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    FloorMissionManager.phase_return_to_car(node)
    assert node.mission.phase == 'ALIGHTING'


def test_rejected_stop_retries_with_spacing_and_fails_on_third_stall(ride_node):
    from amr_elevator_interfaces.msg import ElevatorDoorState, ElevatorRequest

    node, clock = ride_node
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    clock[0] = 20.
    FloorMissionManager.phase_riding(node)
    node.request_elevator.assert_not_called()
    for attempt, now in enumerate((20.1, 40.2), start=1):
        clock[0] = now
        FloorMissionManager.phase_riding(node)
        assert node.request_elevator.call_count == attempt
        node.request_elevator.assert_called_with(ElevatorRequest.KIND_CAR_FLOOR, target_floor='3F')
        FloorMissionManager.phase_riding(node)
        assert node.request_elevator.call_count == attempt
        node.fail.assert_not_called()
    clock[0] = 60.3
    FloorMissionManager.phase_riding(node)
    assert node.mission.counters['car_repress'] == 3 and node.request_elevator.call_count == 2
    assert node.fail.call_args.args[0] == 'RIDE_STALLED'


@pytest.mark.parametrize('interruption', [UNKNOWN, CLOSED, OPENING, CLOSING])
def test_stall_timer_requires_continuously_open_door(ride_node, interruption):
    from amr_elevator_interfaces.msg import ElevatorDoorState

    node, clock = ride_node
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    clock[0] = 19.
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert node.door_open_since == 0.  # repeated OPEN observations preserve the edge
    node.on_door(ElevatorDoorState(state=interruption))
    assert node.door_open_since is None
    clock[0] = 20.
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    clock[0] = 39.
    FloorMissionManager.phase_riding(node)
    node.request_elevator.assert_not_called()
    clock[0] = 41.
    FloorMissionManager.phase_riding(node)
    node.request_elevator.assert_called_once()


def test_waiting_for_answer_only_has_ride_timeout(ride_node):
    from amr_elevator_interfaces.msg import ElevatorDoorState

    node, clock = ride_node
    node.floor_state.floor, node.floor_state.confidence = '3F', 'ASSUMED'
    node.floor_state.awaiting_confirm = True
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    clock[0] = 100.
    FloorMissionManager.phase_riding(node)
    node.request_elevator.assert_not_called()
    node.fail.assert_not_called()
    clock[0] = 181.
    FloorMissionManager.phase_riding(node)
    node.request_elevator.assert_not_called()
    node.fail.assert_called_once_with('RIDE_TIMEOUT', 'did not reach the target floor')


def test_monitor_forwards_and_publishes_candidate_id():
    from builtin_interfaces.msg import Time
    from amr_elevator_interfaces.msg import FloorArrivalConfirm
    from amr_elevator.floor_state import FloorSnapshot

    state = FloorSnapshot(floor='3F', awaiting_confirm=True, candidate_id=42)
    machine = Mock()
    machine.tick.return_value = state
    machine.drain_events.return_value = []
    machine.on_arrival_confirm.return_value = 'accepted'
    node = SimpleNamespace(machine=machine, now=lambda: 10., get_logger=lambda: Mock(),
                           follow_car=Mock(), refresh_anchor=Mock(), estimated_pose=lambda: None,
                           last_published=None, next_publish=0., state_pub=Mock(),
                           p={'map_frame': 'map', 'publish_period': 1.},
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())))
    ElevatorMonitor.tick(node)
    message = node.state_pub.publish.call_args.args[0]
    assert message.candidate_id == 42 and message.awaiting_confirm
    ElevatorMonitor.on_arrival_confirm(node, FloorArrivalConfirm(source=1, floor='3F',
                                                                candidate_id=42, accepted=True))
    machine.on_arrival_confirm.assert_called_once_with(1, '3F', 42, True, 10., actual_floor='')


@pytest.mark.parametrize('margin, depth, expected', [
    (.05, .20, False), (.05, .377, False), (.05, .378, True),
    (.10, .40, False), (.10, .50, True),
])
def test_monitor_reports_whole_body_clear_of_car_door(margin, depth, expected):
    from builtin_interfaces.msg import Time
    from amr_elevator.floor_state import FloorSnapshot

    machine = Mock()
    machine.tick.return_value = FloorSnapshot(floor='3F')
    machine.drain_events.return_value = []
    geometry = SimpleNamespace(
        car_polygon=((0., -1.), (2., -1.), (2., 1.), (0., 1.)),
        hall_door=((-0.2, -1.), (-0.2, 1.)), car_door=((0., -1.), (0., 1.)))
    node = SimpleNamespace(
        machine=machine, now=lambda: 10., follow_car=Mock(), refresh_anchor=Mock(),
        estimated_pose=lambda: (depth, 0., math.pi), geometry=lambda: geometry,
        inward_yaw=lambda: 0., enclosed=True, pose_pub=Mock(), state_pub=Mock(),
        last_published=None, next_publish=0.,
        p={'map_frame': 'map', 'publish_period': 1., 'exit_margin': .37, 'fully_inside_margin': margin},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
    )
    ElevatorMonitor.tick(node)
    # All these centres are inside; only body clearance determines fully_inside.
    machine.on_position.assert_called_once_with(True, False, True, 10., fully_inside=expected)
