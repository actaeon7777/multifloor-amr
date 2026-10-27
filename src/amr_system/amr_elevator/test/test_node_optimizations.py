"""Exercise node methods with fake state/publishers; never initialize or spin ROS."""

from array import array
import json
import math
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException

from amr_elevator import mission_node, monitor_node
from amr_elevator.door_detector import DoorDetector, OPEN, UNKNOWN
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
