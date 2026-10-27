"""Pure geometry checks: no ROS initialization or sensor hardware."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from amr_elevator.clearance import (
    boarding_region, cloud_points_xy, exit_region, points_in_region, scan_points,
    strip_clearance, sweep_hits,
)
from amr_elevator.maneuver import travel_clearances


@pytest.mark.parametrize('laser_pose', [(0., 0., 0.), (.13, -.02, math.pi), (-.1, .04, .3)])
def test_scan_strips_match_existing_clearances(laser_pose):
    rng = np.random.default_rng(902)
    for _ in range(20):
        ranges = rng.uniform(.05, 10., 1080)
        ranges[rng.choice(1080, 100, replace=False)] = np.tile([np.nan, np.inf, 0., -np.inf], 25)
        amin, inc = -math.pi, 2 * math.pi / len(ranges)
        points = scan_points(ranges, amin, inc, .05, 10., laser_pose)
        expected = travel_clearances(ranges, amin, inc, .328, .307, laser_pose)
        assert tuple(strip_clearance(points, reverse) for reverse in (False, True)) == pytest.approx(expected)


def test_scan_filters_limits_and_retains_endpoint_returns():
    points = scan_points([0., .05, 1., 1.01, math.nan, math.inf], 0., 0., .05, 1., (1., 2., math.pi/2))
    np.testing.assert_allclose(points, [(1., 2.05), (1., 3.)])
    assert scan_points([], 0., 1., .05, 10., (0., 0., 0.)).shape == (0, 2)
    assert strip_clearance([], False) == math.inf
    assert strip_clearance([(.2, .307)], False) == pytest.approx(-.128)
    assert strip_clearance([(-.4, -.307)], True) == pytest.approx(.072)


def test_cloud_translation_rotation_and_invalid_points():
    rotation = np.array(((0., 0., 1.), (-1., 0., 0.), (0., -1., 0.)))
    points = cloud_points_xy([[1., .2, 2.], [np.nan, 1., 2.], [1., 2., np.inf]],
                             ((.1, .2, .3), rotation))
    np.testing.assert_allclose(points, [[2.1, -.8]])
    assert cloud_points_xy([], ((0., 0., 0.), np.eye(3))).shape == (0, 2)


@pytest.mark.parametrize('delta', [math.pi, -math.pi])
def test_rotation_sweeps_side_point_but_ignores_body_and_car_walls(delta):
    assert sweep_hits([(0., .4)], delta)
    assert not sweep_hits([(0., .4)], 0.)
    assert not sweep_hits([(0., .1), (.32, .28)], delta)
    walls = [(1.115, y) for y in np.linspace(-.935, .935, 30)]
    walls += [(-1.115, y) for y in np.linspace(-.935, .935, 30)]
    walls += [(x, .935) for x in np.linspace(-1.115, 1.115, 30)]
    walls += [(x, -.935) for x in np.linspace(-1.115, 1.115, 30)]
    assert not sweep_hits(walls, delta)
    assert not sweep_hits([], delta)


def test_sweep_direction_and_exact_end_angle():
    # Just outside the starting front edge; the positive endpoint brings it in.
    point = [(.349, -.1)]
    assert sweep_hits(point, math.radians(1))
    assert not sweep_hits(point, -math.radians(1))


@pytest.fixture
def geometry():
    return SimpleNamespace(hall_door=((-1.775, -.55), (-1.775, .55)),
                           car_door=((-1.115, -.55), (-1.115, .55)),
                           car_pose=(0., 0., math.pi),
                           car_polygon=((-1.115, -.935), (1.115, -.935),
                                        (1.115, .935), (-1.115, .935)))


def test_convex_region_strict_edges_and_empty():
    polygon = [(0., 0.), (1., 0.), (1., 1.), (0., 1.)]
    assert points_in_region([(0., 0.), (.5, .5), (1., .5), (2., 2.), (np.nan, .5)], polygon) == 1
    assert points_in_region([], polygon) == 0


def test_boarding_cluster_wall_and_jamb_exclusion(geometry):
    region = boarding_region(geometry, .10)
    person = np.array([(x, y) for x in [-.15, 0., .15] for y in [-.15, 0., .15]])
    assert region.contains(person).all()
    assert not region.contains([*geometry.car_polygon, (-1.115, 0.), (-1.08, 0.),
                                (-1.115, .55), (-1.115, -.55),
                                (-1.775, .55), (-1.775, -.55), (0., .935)]).any()
    assert region.contains([(-.9, 0.)])[0]  # corridor
    assert region.contains([(0., .5)])[0]   # turning disc outside corridor
    assert not region.contains([(0., .7)])[0]
    # Independently configurable wall inset.
    assert not boarding_region(geometry, .1, wall_margin=.3).contains([(-.9, 0.)])[0]


def test_boarding_from_the_wait_pose_covers_hall_approach_and_tunnel(geometry):
    # Wait pose 1.0 m in front of the hall door, facing the car.
    region = boarding_region(geometry, .10, approach_from=(-2.775, 0., 0.))
    # Someone still walking out of the car: in the hall, in the tunnel, in the car.
    assert region.contains([(-2.3, 0.), (-1.775, .1), (-1.4, -.1), (0., 0.)]).all()
    assert not region.contains([
        (-2.6, 0.),                              # behind the front bumper (the robot itself)
        (-2.1, .6), (-2.775, .6),                # beside the path in the hall
        (-1.775, .55), (-1.775, -.55),           # hall-door jambs
        (-1.4, .55), (-1.4, -.45),               # tunnel walls with the 0.15 m inset
        (-1.775, .8), (-1.775, -.8),             # hall wall beside the door
        *geometry.car_polygon, (0., .935),       # car walls
    ]).any()
    # A robot offset to one side gets its own slanted path, still clipped at the jambs.
    offset = boarding_region(geometry, .10, approach_from=(-2.775, .2, -.15))
    assert offset.contains([(-2.3, .13)])[0] and not offset.contains([(-1.775, .55)])[0]


def test_exit_includes_hall_and_excludes_car_walls(geometry):
    region = exit_region(geometry, (-2.5, 0., math.pi), .1)
    assert region.contains([(0., 0.), (-.8, 0.), (-1.5, 0.), (-2.4, .3)]).all()
    assert not region.contains([(-1.115, 0.), (-1.08, 0.), (-2.4, .5), (-2.6, 0.)]).any()


def test_rotated_map_region_is_equivalent(geometry):
    rotation = np.array(((0., -1.), (1., 0.)))
    def transform(points):
        return np.asarray(points) @ rotation.T + (12., -4.)
    rotated = SimpleNamespace(hall_door=transform(geometry.hall_door),
                              car_polygon=transform(geometry.car_polygon),
                              car_pose=(12., -4., -math.pi/2))
    points = [(0., 0.), (0., .6), (-.8, 0.), (-1.115, .55)]
    np.testing.assert_array_equal(boarding_region(rotated, .1).contains(transform(points)),
                                  boarding_region(geometry, .1).contains(points))


def test_exit_wall_exclusion_survives_arbitrary_map_rotation(geometry):
    angle = .37
    rotation = np.array(((math.cos(angle), -math.sin(angle)), (math.sin(angle), math.cos(angle))))
    def transform(points):
        return np.asarray(points) @ rotation.T + (12., -4.)
    rotated = SimpleNamespace(car_polygon=transform(geometry.car_polygon), car_pose=(12., -4., angle),
                              hall_door=transform(geometry.hall_door), car_door=transform(geometry.car_door))
    exit_pose = (*transform([(-2.5, 0.)])[0], angle)
    assert not exit_region(rotated, exit_pose, .1).contains(transform([(-1.115, 0.)]))[0]


@pytest.mark.parametrize('big_endian', [False, True])
def test_depth_message_cache_padding_transform_and_expiry_without_ros_node(big_endian):
    """Call the callback on fake state; never construct, initialize or spin a node."""
    from types import MethodType
    from unittest.mock import Mock
    from sensor_msgs.msg import PointCloud2, PointField
    from amr_elevator.mission_node import FloorMissionManager

    from time import monotonic
    clock = [1.]
    scan = object()
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0., y=0., z=.3),
        rotation=SimpleNamespace(x=-.5, y=.5, z=-.5, w=.5)))
    node = SimpleNamespace(p={'use_depth': True, 'depth_max_age': .5, 'base_frame': 'base_footprint',
                              'depth_min_height': .06, 'depth_max_height': 1., 'depth_max_range': 3.,
                              'scan_max_age': .5},
                           depth_transform=None, depth_received=-math.inf, scan_received=monotonic(),
                           mission=SimpleNamespace(phase='WAIT_DOOR_OPEN'),
                           now=lambda: clock[0], tf=SimpleNamespace(lookup_transform=Mock(return_value=transform)),
                           _scan_points=np.empty((0, 2)), scan=scan, _clearance_scan=scan,
                           laser_pose=(0., 0., 0.), _clearances=(math.inf, math.inf))
    for name in ('depth_fresh', 'scan_fresh', 'obstacle_points', 'clearance'):
        setattr(node, name, MethodType(getattr(FloorMissionManager, name), node))
    cloud = PointCloud2(height=2, width=1, point_step=16, row_step=20, is_bigendian=big_endian,
                        fields=[PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                for i, name in enumerate(('x', 'y', 'z'))])
    cloud.header.frame_id = 'camera_depth_optical_frame'
    cloud.data = np.array([[0., 0., .45, 99., 99.], [0., 0., .8, 99., 99.]],
                          dtype=('>' if big_endian else '<') + 'f4').tobytes()
    FloorMissionManager.on_depth(node, cloud)
    assert node.tf.lookup_transform.call_count == 1
    np.testing.assert_allclose(node.obstacle_points(), [[.45, 0.], [.8, 0.]])
    assert node.clearance(False) == pytest.approx(.13)
    assert node.clearance(True) == math.inf
    assert node.depth_received == 1.
    clock[0] = 1.5
    assert node.clearance(False) == pytest.approx(.13)  # inclusive expiry boundary
    clock[0] = 1.501
    assert node.clearance(False) == math.inf
    assert node.obstacle_points().shape == (0, 2)
    FloorMissionManager.on_depth(node, cloud)
    assert node.tf.lookup_transform.call_count == 1  # static transform cached
    assert node.depth_received == 1.501
    node.p['use_depth'] = False
    assert node.clearance(False) == math.inf


def test_space_confirmation_resets_on_occupancy_or_unknown_pose(geometry):
    from amr_elevator.mission_node import FloorMissionManager

    clock, points = [0.], [np.empty((0, 2))]
    node = SimpleNamespace(p={'clear_confirm_s': .5}, pose=(0., 0., 0.), scan=object(),
                           scan_fresh=lambda: True,
                           laser_pose=(0., 0., 0.), obstacle_points=lambda: points[0],
                           mission=SimpleNamespace(counters={}), now=lambda: clock[0])
    region = boarding_region(geometry, .1)
    assert not FloorMissionManager.space_clear(node, region)
    clock[0] = .4
    points[0] = np.array([[0., 0.]])
    assert not FloorMissionManager.space_clear(node, region)
    points[0] = np.empty((0, 2))
    clock[0] = .5
    assert not FloorMissionManager.space_clear(node, region)
    clock[0] = 1.
    assert FloorMissionManager.space_clear(node, region)
    node.pose = None
    assert not FloorMissionManager.space_clear(node, region)
    assert '_clear_since' not in node.mission.counters


@pytest.mark.parametrize('delta', [math.pi/2, math.pi, -math.pi/2, -math.pi])
def test_near_body_person_is_not_discarded_by_inflation(delta):
    assert sweep_hits([(0., .33)], delta, half_length=.398, half_width=.357)
    assert sweep_hits([(0., .45), (0., .33)], delta, half_length=.398, half_width=.357)
    assert not sweep_hits([(.328, .287), (0., .297)], delta, half_length=.398, half_width=.357)
    # An explicitly smaller real body changes only the residue exclusion.
    assert sweep_hits([(0., .28)], delta, body_half_length=.2, body_half_width=.2)


@pytest.mark.parametrize('delta', [1e-6, -1e-6, .01, -.01, math.pi/2, math.pi])
def test_front_person_blocks_nonzero_turn_with_node_safety_envelope(delta):
    # drive() rounds its .398/.357 safety half-extents outward to centimetres.
    assert sweep_hits([(.40, 0.)], delta, half_length=.40, half_width=.36)
    assert not sweep_hits([(.40, 0.)], 0., half_length=.40, half_width=.36)


def test_optical_floor_and_ceiling_are_removed_before_all_xy_checks(geometry):
    rotation = np.array(((0., 0., 1.), (-1., 0., 0.), (0., -1., 0.)))
    translation = np.array((.1, 0., .6))
    floor = np.array([(x, y, 0.) for x in np.linspace(.35, 3., 40)
                      for y in np.linspace(-.8, .8, 25)])
    box = np.array([[.45, .05, .10], [.50, -.05, .10]])
    rejected = np.array([[.6, 0., 1.5], [3.01, 0., .1], [0., 3.01, .1],
                         [.6, 0., -.1], [np.nan, 0., .1]])
    def optical(base):
        return (base - translation) @ rotation
    transform = (translation, rotation)
    floor_xy = cloud_points_xy(optical(floor), transform)
    assert floor_xy.shape == (0, 2)
    assert strip_clearance(floor_xy, False) == math.inf
    assert not sweep_hits(floor_xy, math.pi, half_length=.40, half_width=.36)
    assert not boarding_region(geometry, .1).contains(floor_xy).any()
    xy = cloud_points_xy(optical(np.concatenate((floor, box, rejected))), transform)
    np.testing.assert_allclose(xy, box[:, :2])
    assert strip_clearance(xy, False) == pytest.approx(.45 - .328)


def test_depth_height_and_horizontal_range_boundaries_and_overrides():
    points = [[3., 0., .06], [0., 3., 1.], [3., .01, .1], [0., 0., .059], [0., 0., 1.001]]
    np.testing.assert_allclose(cloud_points_xy(points, ((0., 0., 0.), np.eye(3))),
                               [[3., 0.], [0., 3.]])
    assert cloud_points_xy(points, ((0., 0., 0.), np.eye(3)), .1, .5, 2.).shape == (0, 2)


@pytest.mark.parametrize('point', [(0., .33), (.40, 0.)])
def test_node_drive_holds_turn_for_person_within_safety_envelope(point):
    from amr_elevator.maneuver import LineManeuver, TURN
    from amr_elevator.mission_node import FloorMissionManager

    commands = []
    maneuver = LineManeuver((0., 0.), final_yaw=.1)
    maneuver.phase = TURN
    node = SimpleNamespace(maneuver=maneuver, pose=(0., 0., 0.), mode='AUTO', estop_ready=lambda: True,
                           renew_lease=lambda: None, laser_pose=(0., 0., 0.), scan_fresh=lambda: True,
                           obstacle_points=lambda: np.asarray([point]), clearance=lambda reverse: math.inf,
                           p={'turn_margin': .05, 'tick_period': .05},
                           mission=SimpleNamespace(phase='IN_CAR_ALIGN'),
                           publish_twist=lambda v=0., w=0.: commands.append((v, w)))
    assert not FloorMissionManager.drive(node)
    assert commands == [(0., 0.)] and maneuver.blocked


@pytest.mark.parametrize('shift', [-.15, 0., .15])
@pytest.mark.parametrize('angle', [0., .37, math.pi])
def test_exit_tunnel_excludes_jambs_with_pose_error_but_keeps_people(geometry, shift, angle):
    rotation = np.array(((math.cos(angle), -math.sin(angle)), (math.sin(angle), math.cos(angle))))
    def transform(points):
        return np.asarray(points) @ rotation.T + (12., -4.)
    rotated = SimpleNamespace(car_polygon=transform(geometry.car_polygon), car_pose=(12., -4., angle),
                              hall_door=transform(geometry.hall_door), car_door=transform(geometry.car_door))
    region = exit_region(rotated, (*transform([(-2.5, 0.)])[0], angle), .1)
    # Scan points projected with a laterally shifted robot pose.
    jambs = [(x, side + shift) for x in np.linspace(-1.775, -1.116, 20) for side in [-.55, .55]]
    person = [(x, side + shift) for x in [-1.6, -1.5, -1.4] for side in [-.15, 0., .15]]
    assert not region.contains(transform(jambs)).any()
    assert region.contains(transform(person)).all()
    # Boarding remains clipped to the car interior: it has no tunnel component.
    assert not boarding_region(rotated, .1).contains(transform(jambs)).any()


def test_exit_tunnel_margin_does_not_shrink_hall_corridor(geometry):
    region = exit_region(geometry, (-2.5, 0., 0.), .3, wall_margin=.2)
    assert not region.contains([(-1.5, .36)])[0]
    assert region.contains([(-2., .5)])[0]


def test_scan_expiry_holds_space_drive_and_turn_even_with_fresh_depth(geometry, monkeypatch):
    from types import MethodType
    from unittest.mock import Mock
    from sensor_msgs.msg import LaserScan
    from amr_elevator import mission_node
    from amr_elevator.maneuver import LineManeuver, TURN
    from amr_elevator.mission_node import FloorMissionManager

    clock = [0.]
    monkeypatch.setattr(mission_node, 'monotonic', lambda: clock[0])
    node = SimpleNamespace(
        p={'scan_max_age': .5, 'clear_confirm_s': .1, 'use_depth': True, 'depth_max_age': .5,
           'turn_margin': .05, 'tick_period': .05}, now=lambda: clock[0],
        pose=(0., 0., 0.), scan=None, scan_received=-math.inf, laser_pose=(0., 0., 0.),
        _scan_points=np.empty((0, 2)), _clearance_scan=None, _clearances=(math.inf, math.inf),
        depth_transform=((0., 0., 0.), np.eye(3)), depth_received=0.,
        _depth_points=np.empty((0, 2)), _depth_clearances=(math.inf, math.inf),
        mission=SimpleNamespace(phase='IN_CAR_ALIGN', counters={}), mode='AUTO', estop_ready=lambda: True,
        publish_twist=Mock(), renew_lease=lambda: None)
    for name in ('scan_fresh', 'depth_fresh', 'obstacle_points', 'clearance', 'space_clear'):
        setattr(node, name, MethodType(getattr(FloorMissionManager, name), node))
    assert node.clearance(False) == 0.  # no scan received yet
    scan = LaserScan(range_min=.05, range_max=30., angle_increment=.1)
    scan.ranges = [math.inf]
    FloorMissionManager.on_scan(node, scan)
    assert node.scan_received == 0.
    region = boarding_region(geometry, .1)
    assert not node.space_clear(region)
    clock[0] = .5
    assert node.space_clear(region) and node.clearance(False) == math.inf
    clock[0] = .5001
    node.depth_received = clock[0]
    assert not node.space_clear(region) and '_clear_since' not in node.mission.counters
    assert node.clearance(False) == node.clearance(True) == 0.
    node.maneuver = LineManeuver((0., 0.), final_yaw=math.pi)
    node.maneuver.phase = TURN
    assert not FloorMissionManager.drive(node)
    node.publish_twist.assert_called_with(0., 0.)
    assert node.maneuver.blocked
    FloorMissionManager.on_scan(node, scan)
    assert not FloorMissionManager.drive(node)
    assert node.publish_twist.call_args.args[1] != 0. and not node.maneuver.blocked
    assert not node.space_clear(region)  # confirmation must restart after stale data


@pytest.mark.parametrize('phase', [None, 'IDLE', 'NAV_TO_HALL', 'CALL_ELEVATOR', 'SELECT_FLOOR',
                                  'RIDING', 'SWITCH_MAP', 'RELOCALIZE', 'NAV_TO_GOAL', 'FAILED_SAFE'])
def test_depth_is_dropped_before_accessing_cloud_outside_elevator_legs(phase):
    from unittest.mock import Mock
    from amr_elevator.mission_node import FloorMissionManager

    node = SimpleNamespace(p={'use_depth': True}, mission=None if phase is None else SimpleNamespace(phase=phase),
                           depth_transform=None, tf=Mock(), depth_received=-math.inf)
    # This object has no header/fields/data: touching it would raise immediately.
    FloorMissionManager.on_depth(node, object())
    node.tf.lookup_transform.assert_not_called()
    assert node.depth_received == -math.inf
