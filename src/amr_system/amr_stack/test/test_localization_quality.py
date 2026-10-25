"""Evidence gates must reject plausible but unsupported localization poses."""

import ast
from array import array
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

import amr_stack.localization_quality as localization_quality
from amr_stack.localization_quality import (
    MapQuality, ScanQuality, covariance_is_bounded, particle_concentration,
)


class TestTime:
    """Only the timestamp interface needed by the pure startup calculations."""

    __test__ = False

    def __init__(self, *, nanoseconds):
        self.nanoseconds = nanoseconds
        self.clock_type = 'ROS_TIME'

    @classmethod
    def from_msg(cls, stamp):
        return cls(nanoseconds=stamp.sec * 1_000_000_000 + stamp.nanosec)


@pytest.fixture
def startup_unit():
    """Execute actual methods without importing ROS or constructing a Node."""
    path = Path(localization_quality.__file__).with_name('navigation_startup.py')
    tree = ast.parse(path.read_text())
    functions = {'compose_se2', 'inverse_se2', 'stamp_ns'}
    methods = {'tracking_ray_poses', 'scan_args', 'quality_range',
               'pose_covariance_is_bounded', 'transform_available',
               'tick', 'set_state', 'publish_status'}
    body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in functions]
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'NavigationStartup')
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    body.append(cls)
    namespace = dict(np=np, math=math, json=json, Time=TestTime, String=SimpleNamespace,
                     time=SimpleNamespace(monotonic=lambda: 100., time=lambda: 200.),
                     covariance_is_bounded=Mock(wraps=covariance_is_bounded))
    exec(compile(ast.Module(body=body, type_ignores=[]), str(path), 'exec'), namespace)
    node = namespace['NavigationStartup']()
    node._in_tick = False
    node._status_dirty = False
    node._covariance_cache = node._tf_availability_cache = None
    node.cfg = dict(odom_frame='odom', quality_max_beams=60, quality_max_range=30.,
                    tracking_quality_max_range=12., global_localization_backend='tim')
    node.metrics = {}
    node.prior_verified = True
    node.get_logger = lambda: SimpleNamespace(info=Mock(), error=Mock())
    return node, namespace


@pytest.mark.parametrize('duration', [.0001, .01, .1, .215])
@pytest.mark.parametrize('returns', ['mixed', 'none'])
@pytest.mark.parametrize('heading', [0., math.pi - .03, -math.pi + .03])
def test_vectorized_tracking_ray_poses_equal_original_loop(startup_unit, duration, returns, heading):
    node, namespace = startup_unit
    ranges = np.linspace(.1, 15., 121).astype(np.float32)
    ranges[::7] = np.nan
    ranges[::11] = np.inf
    if returns == 'none':
        ranges[:] = np.inf
    node.scan = SimpleNamespace(
        ranges=array('f', ranges), angle_min=-1., angle_increment=.03,
        range_min=.01, range_max=30., time_increment=duration / 120,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=123), frame_id='laser'))
    node.map_quality = MapQuality
    node.valid_scan = lambda scan: True
    lookups = []

    def transform(target, source, when):
        lookups.append((target, source, when.nanoseconds))
        t = (when.nanoseconds - 10_000_000_123) / 1e9
        theta = heading + 4. * t
        return (1. + .7*t + .3*math.cos(theta), -.5 + t*t + .3*math.sin(theta),
                math.atan2(math.sin(theta), math.cos(theta)))

    node.checked_transform = transform
    laser_pose = (2., -3., heading)
    expected = loop_tracking_ray_poses_reference(node, laser_pose, namespace)
    expected_metrics, expected_lookups = node.metrics.copy(), lookups.copy()
    lookups.clear()
    actual = node.tracking_ray_poses(laser_pose)
    assert actual.shape == expected.shape
    np.testing.assert_allclose(actual, expected, rtol=0., atol=1e-9)
    assert node.metrics == expected_metrics
    assert lookups == expected_lookups


@pytest.mark.parametrize('scene', ['corridor', 'far_wall', 'leaving_map'])
@pytest.mark.parametrize('ray_step', [.1, .2, .3, 20.])
@pytest.mark.parametrize('moving', [False, True])
def test_ragged_score_equals_dense_reference(scene, ray_step, moving):
    if scene == 'corridor':
        grid = np.zeros((20, 140), dtype=int)
        grid[[0, -1], :] = 100
        grid[:, [0, -1]] = 100
        grid[1:8, 50:55] = -1
        grid[12:19, 70:72] = 100
        pose, max_range = (2.05, 1.05, .1), 12.
    elif scene == 'far_wall':
        grid = np.zeros((80, 450), dtype=int)
        grid[[0, -1], :] = 100
        grid[:, [0, -1]] = 100
        pose, max_range = (2.05, 4.05, 0.), 8.
    else:
        grid = np.zeros((30, 40), dtype=int)
        grid[5:10, 10:15] = -1
        grid[10:25, 25] = 100
        pose, max_range = (1.05, 1.55, .3), 12.
    scan = synthetic_scan(grid, pose, count=96)
    # Include paths with no interior samples, exact/adjacent step boundaries,
    # and long measured returns through unknown, walls and map boundaries.
    scan['ranges'][::9] = [0.1, .3, np.nextafter(.3, 0.), np.nextafter(.3, 1.),
                           1., 3., 6., 9., 11., 2., 5.]
    if scene == 'far_wall':
        # A real endpoint at the room's far wall must be excluded by max_range.
        scan['range_max'] = 50.
        scan['ranges'][48] = 42.85
    indices, _, _ = MapQuality.sampled_rays(**scan, max_beams=73, max_range=max_range)
    if scene == 'far_wall':
        assert 48 not in indices
    poses = None
    if moving:
        poses = np.tile(pose, (len(indices), 1))
        poses[:, 0] += np.linspace(-.1, .2, len(indices))
        poses[:, 2] += np.linspace(-.2, .2, len(indices))
    quality = map_quality(grid)
    args = dict(**scan, laser_pose=pose, max_beams=73, max_range=max_range,
                ray_step=ray_step, ray_poses=poses)
    expected = dense_score_reference(quality, **args)
    actual = quality.score(**args)
    # Dataclass equality checks every count, ratio and origin_free exactly.
    assert actual == expected
    assert actual.valid_beams > 0
    if scene == 'leaving_map':
        assert actual.known_beams < actual.valid_beams


@pytest.mark.parametrize('ray_step', [.1, .2, .3])
def test_ragged_score_preserves_strict_step_boundaries_and_empty_paths(ray_step):
    grid = np.zeros((30, 80), dtype=int)
    grid[:, 10] = -1
    grid[:, 20] = 100
    quality = map_quality(grid, tolerance=.2)
    stops = np.arange(ray_step, 5., ray_step) + .2
    ranges = np.column_stack((np.nextafter(stops, 0.), stops,
                              np.nextafter(stops, np.inf))).ravel()
    for values in (ranges, [.1, .2, .3], [], [np.inf, np.nan]):
        args = dict(ranges=values, angle_min=0., angle_increment=.0001,
                    range_min=.01, range_max=12., laser_pose=(.55, 1.55, 0.),
                    max_beams=180, ray_step=ray_step)
        assert quality.score(**args) == dense_score_reference(quality, **args)


def prepare_status(node):
    node.state, node.detail, node.source = 'READY', 'ready', 'manual'
    node.last_logged_detail, node.last_state_log = '', -math.inf
    node.verification_requirements = lambda: (1., 4)
    node.status_pub = SimpleNamespace(publish=Mock())
    node.generation, node.session = 1, 'test'
    node.beam_phase, node.applied_beams = 'tracking', 60
    node.saved_poses, node.map_id = {}, 'map'
    node.tracking_warning, node.tracking_bad_count = '', 0
    node.tracking_bad_since = node.good_since = None
    node.good_count = 0
    node.tim_metrics, node.refinement_metrics = {}, {}
    node.remember_pose = Mock()


@pytest.mark.parametrize('raises', [False, True])
def test_tick_coalesces_status_but_outside_callbacks_publish_immediately(startup_unit, raises):
    node, _ = startup_unit
    prepare_status(node)

    def advance():
        node.set_state('VERIFYING', 'checking')
        node.set_state('READY', 'ready')
        assert node.status_pub.publish.call_count == 0
        if raises:
            raise ValueError('test fault')

    node.advance = advance
    node.remember_pose = lambda: node.metrics.update(remembered=True)
    node.tick()
    assert node.status_pub.publish.call_count == 1
    payload = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert payload['state'] == ('FAILED' if raises else 'READY')
    assert payload['remembered']
    assert payload['quality_max_range'] == 12.
    assert not node._in_tick and not node._status_dirty
    node.set_state('SUSPENDED', 'service callback')
    assert node.status_pub.publish.call_count == 2
    assert json.loads(node.status_pub.publish.call_args.args[0].data)['detail'] == 'service callback'
    # The periodic heartbeat remains even when advance changes nothing.
    node.advance = lambda: None
    node.tick()
    assert node.status_pub.publish.call_count == 3


def test_covariance_and_tf_caches_are_scoped_to_tick_and_exact_keys(startup_unit):
    node, namespace = startup_unit
    prepare_status(node)
    checker = namespace['covariance_is_bounded']
    poses = [SimpleNamespace(pose=SimpleNamespace(covariance=covariance().ravel())),
             SimpleNamespace(pose=SimpleNamespace(covariance=np.zeros(36)))]
    node.tf_buffer = SimpleNamespace(can_transform=Mock(return_value=True))

    def advance():
        for _ in range(2):
            assert node.pose_covariance_is_bounded(poses[0], math.inf, math.inf)
            assert not node.pose_covariance_is_bounded(poses[0], .01, .01)
            assert not node.pose_covariance_is_bounded(poses[1], math.inf, math.inf)
            for stamp, source in [(1, 'laser'), (2, 'laser'), (1, 'base')]:
                actual = node.transform_available('odom', source, TestTime(nanoseconds=stamp))
                assert actual is node.tf_buffer.can_transform.return_value

    node.advance = advance
    node.tick()
    assert checker.call_count == node.tf_buffer.can_transform.call_count == 3
    # TF loss must be observed again on the next tick, never persistently cached.
    node.tf_buffer.can_transform.return_value = False
    node.tick()
    assert checker.call_count == node.tf_buffer.can_transform.call_count == 6
    assert node._covariance_cache is node._tf_availability_cache is None
    # Calls from outside a tick never reuse results.
    for _ in range(2):
        node.pose_covariance_is_bounded(poses[0], math.inf, math.inf)
        assert not node.transform_available('odom', 'laser', TestTime(nanoseconds=1))
    assert checker.call_count == node.tf_buffer.can_transform.call_count == 8


def test_readonly_scan_array_survives_new_message_and_threaded_consumers(startup_unit):
    node, _ = startup_unit
    grid, pose = room_grid(), (2.05, 2.55, .1)
    scan = synthetic_scan(grid, pose, count=24)
    scan['ranges'] = array('f', scan['ranges'])
    original = scan['ranges'].tobytes()
    node.scan = SimpleNamespace(**scan)
    args = node.scan_args()
    assert isinstance(args['ranges'], np.ndarray)
    assert not args['ranges'].flags.writeable
    assert np.shares_memory(args['ranges'], np.asarray(scan['ranges']))
    with pytest.raises(ValueError):
        args['ranges'][0] = 0.
    node.scan = SimpleNamespace(ranges=array('f', [0.]))
    quality = map_quality(grid)
    list_args = dict(args, ranges=list(scan['ranges']))
    consumers = [
        (quality.score, {}),
        (quality.find_rival, dict(max_candidates=10)),
        (quality.refine_pose, dict(position_window=.1, yaw_window=.05, seeds=1)),
    ]
    with ThreadPoolExecutor(max_workers=1) as worker:
        for consumer, extra in consumers:
            expected = consumer(**list_args, laser_pose=pose, **extra)
            actual = worker.submit(consumer, **args, laser_pose=pose, **extra).result()
            assert actual == expected
    assert scan['ranges'].tobytes() == original


def map_quality(grid, resolution=.1, origin=(0., 0., 0.), tolerance=.15):
    height, width = grid.shape
    return MapQuality(width, height, resolution, *origin, grid.ravel(), match_tolerance=tolerance)


def synthetic_scan(grid, pose, resolution=.1, count=120):
    """Raycast independently of MapQuality, returning first occupied endpoints."""
    ranges = []
    angle_min, increment = -math.pi, 2 * math.pi / count
    for angle in angle_min + np.arange(count) * increment + pose[2]:
        result = float('inf')
        for distance in np.arange(.02, 12., .02):
            gx = math.floor((pose[0] + distance * math.cos(angle)) / resolution)
            gy = math.floor((pose[1] + distance * math.sin(angle)) / resolution)
            if not (0 <= gx < grid.shape[1] and 0 <= gy < grid.shape[0]):
                break
            if grid[gy, gx] >= 65:
                result = distance
                break
        ranges.append(result)
    return dict(ranges=ranges, angle_min=angle_min, angle_increment=increment,
                range_min=.01, range_max=12.)


def room_grid():
    grid = np.zeros((50, 60), dtype=int)
    grid[[0, -1], :] = 100
    grid[:, [0, -1]] = 100
    # An asymmetric interior feature breaks the rotational symmetry of a room.
    grid[12:20, 35:40] = 100
    return grid


def test_correct_scan_matches_and_wrong_pose_does_not():
    grid = room_grid()
    quality = map_quality(grid)
    pose = (2.05, 2.55, .1)
    scan = synthetic_scan(grid, pose)
    correct = quality.score(**scan, laser_pose=pose)
    wrong = quality.score(**scan, laser_pose=(3.05, 2.55, .1))
    assert correct.origin_free
    assert correct.valid_beams == 120
    assert correct.match_ratio > .95
    assert wrong.match_ratio < .6


def test_unknown_and_outside_endpoints_are_not_dropped_from_denominator():
    grid = np.zeros((10, 10), dtype=int)
    grid[5, 6] = 100
    grid[6, 4] = -1
    quality = map_quality(grid, tolerance=0.)
    # Four cardinal rays: occupied, unknown, outside the map, and known free.
    result = quality.score([.2, .1, 2., .2], 0., math.pi / 2, .01, 10.,
                           (.45, .55, 0.), ray_step=1.)
    assert result.valid_beams == 4
    assert result.matched_beams == 1
    assert result.known_beams == 2
    assert result.match_ratio == .25
    assert result.known_ratio == .5


def test_unknown_near_occupied_still_does_not_match():
    grid = np.zeros((10, 10), dtype=int)
    grid[5, 7] = 100
    grid[5, 6] = -1
    result = map_quality(grid, tolerance=.2).score(
        [.2], 0., .1, .01, 10., (.45, .55, 0.), ray_step=1.)
    assert result.match_ratio == 0.
    assert result.known_ratio == 0.


def test_laser_pose_and_rotated_occupancy_grid_origin_are_respected():
    grid = room_grid()
    local_pose = (2.05, 2.55, .1)
    scan = synthetic_scan(grid, local_pose)
    plain = map_quality(grid).score(**scan, laser_pose=local_pose)
    origin = (12., -7., math.pi / 2)
    world_pose = (origin[0] - local_pose[1], origin[1] + local_pose[0],
                  local_pose[2] + origin[2])
    transformed = map_quality(grid, origin=origin).score(**scan, laser_pose=world_pose)
    assert transformed == plain


def test_endpoint_match_through_an_intervening_wall_is_rejected():
    grid = np.zeros((10, 40), dtype=int)
    grid[:, 15:18] = 100
    grid[:, 30:33] = 100
    result = map_quality(grid).score([2.55], 0., .1, .01, 10., (.55, .55, 0.))
    assert result.obstructed_beams == 1
    assert result.match_ratio == 0.


def test_empty_or_no_return_scan_has_no_evidence():
    quality = map_quality(room_grid())
    result = quality.score([float('inf'), float('nan'), 12., 0.], 0., .1, .01, 12.,
                           (2., 2., 0.))
    assert result.valid_beams == 0
    assert result.match_ratio == 0.


def covariance(x_var=.02, y_var=.02, yaw_var=.02):
    matrix = np.zeros((6, 6))
    matrix[0, 0], matrix[1, 1], matrix[5, 5] = x_var, y_var, yaw_var
    return matrix


def test_covariance_checks_yaw_correlation_and_invalid_estimates():
    assert covariance_is_bounded(covariance().ravel())
    assert not covariance_is_bounded(covariance(yaw_var=.2).ravel())
    assert not covariance_is_bounded(np.zeros(36))
    correlated = covariance(.08, .08)
    correlated[0, 1] = correlated[1, 0] = .07
    assert not covariance_is_bounded(correlated.ravel(), max_xy_std=.35)
    impossible = covariance()
    impossible[0, 5] = impossible[5, 0] = .2
    assert not covariance_is_bounded(impossible.ravel())
    invalid = covariance()
    invalid[5, 5] = float('nan')
    assert not covariance_is_bounded(invalid.ravel())


def test_particle_mass_checks_heading_modes_wraparound_and_weights():
    poses = [[1., 2., math.pi - .02], [1., 2., -math.pi + .02], [1., 2., 0.]]
    assert particle_concentration(poses, [1., 2., math.pi]) == pytest.approx(2 / 3)
    assert particle_concentration(poses, [1., 2., math.pi], weights=[.45, .45, .1]) == pytest.approx(.9)
    assert particle_concentration([], [1., 2., 0.]) == 0.
    assert particle_concentration(poses, [1., 2., 0.], weights=[1., -1., 1.]) == 0.
    assert particle_concentration([[2., 2., 0.]], [1., 2., 0.]) == 0.


def test_repeated_corridor_has_a_rival_despite_concentrated_particles():
    grid = np.full((40, 100), -1, dtype=int)
    # Two identical long, narrow enclosed corridors separated by unknown space.
    for x0 in (0, 50):
        grid[5:35, x0 + 5:x0 + 45] = 0
        grid[[5, 34], x0 + 5:x0 + 45] = 100
        grid[5:35, [x0 + 5, x0 + 44]] = 100
    pose = (2.05, 1.55, .13)
    scan = synthetic_scan(grid, pose, count=60)
    quality = map_quality(grid)
    assert quality.score(**scan, laser_pose=pose).match_ratio > .95
    assert particle_concentration([pose] * 100, pose) == 1.
    rival = quality.find_rival(**scan, laser_pose=pose, max_beams=60)
    assert rival.completed
    assert rival.best_score > .95
    assert rival.best_pose is not None
    assert (math.hypot(rival.best_pose[0] - pose[0], rival.best_pose[1] - pose[1]) > 1.
            or abs((rival.best_pose[2] - pose[2] + math.pi) % (2 * math.pi) - math.pi) > .5)


def test_rival_budget_exhaustion_never_means_unique():
    grid = room_grid()
    pose = (2.05, 2.55, .1)
    result = map_quality(grid).find_rival(**synthetic_scan(grid, pose, count=30),
                                        laser_pose=pose, max_candidates=10)
    assert not result.completed
    assert result.candidates_evaluated <= 10


@pytest.mark.parametrize('hint_offset', [(.6, -.4, .35), (-.8, .3, -.4), (0., 0., 0.)])
def test_refinement_recovers_a_rough_operator_click(hint_offset):
    grid = room_grid()
    pose = (2.05, 2.55, .1)
    scan = synthetic_scan(grid, pose, count=180)
    hint = tuple(p + o for p, o in zip(pose, hint_offset))
    result = map_quality(grid).refine_pose(**scan, laser_pose=hint)
    assert math.hypot(result.pose[0] - pose[0], result.pose[1] - pose[1]) < .06
    assert abs((result.pose[2] - pose[2] + math.pi) % (2 * math.pi) - math.pi) < .03
    assert result.quality.match_ratio > .95
    assert result.quality.match_ratio >= result.hint_quality.match_ratio


def test_refinement_stays_within_its_window_and_rotated_map_origin():
    grid = room_grid()
    origin = (1., -2., .3)
    quality = map_quality(grid, origin=origin)
    local = (2.05, 2.55, .1)
    c, s = math.cos(origin[2]), math.sin(origin[2])
    pose = (origin[0] + c*local[0] - s*local[1], origin[1] + s*local[0] + c*local[1], local[2] + origin[2])
    scan = synthetic_scan(grid, local, count=180)
    near = quality.refine_pose(**scan, laser_pose=(pose[0] + .3, pose[1], pose[2] - .2))
    assert math.hypot(near.pose[0] - pose[0], near.pose[1] - pose[1]) < .06
    # Too far for the window: the result never leaves it to reach the true pose.
    far = (pose[0] + 2., pose[1], pose[2])
    result = quality.refine_pose(**scan, laser_pose=far, position_window=.5)
    assert math.hypot(result.pose[0] - pose[0], result.pose[1] - pose[1]) > 1.


def test_invalid_map_geometry_and_scan_metadata_fail_explicitly():
    with pytest.raises(ValueError):
        MapQuality(2, 2, .1, 0., 0., 0., [0])
    with pytest.raises(ValueError):
        map_quality(room_grid()).score([1.], 0., 0., .01, 12., (2., 2., 0.))


def test_sampled_ray_indices_preserve_scan_timing_through_filtering_and_thinning():
    ranges = [float('nan'), 1., float('inf'), 2., 3., .005, 4., 10.]
    arguments = (ranges, .5, -.25, .01, 10., 2, 3.5)
    indices, distances, angles = MapQuality.sampled_rays(*arguments)
    assert indices.tolist() == [1, 4]
    assert distances.tolist() == [1., 3.]
    assert angles.tolist() == [.25, -.5]
    old_distances, old_angles = MapQuality._rays(*arguments)
    np.testing.assert_array_equal(old_distances, distances)
    np.testing.assert_array_equal(old_angles, angles)


def moving_scan(grid, poses, resolution=.1):
    """Trace one physical ray at each acquisition pose, independently of scoring."""
    ranges = []
    increment = 2 * math.pi / len(poses)
    for index, (x, y, heading) in enumerate(poses):
        direction = -math.pi + index * increment + heading
        distance = float('inf')
        for sample in np.arange(.005, 12., .005):
            column = math.floor((x + sample * math.cos(direction)) / resolution)
            row = math.floor((y + sample * math.sin(direction)) / resolution)
            if not (0 <= column < grid.shape[1] and 0 <= row < grid.shape[0]):
                break
            if grid[row, column] >= 65:
                distance = float(sample)
                break
        ranges.append(distance)
    return dict(ranges=ranges, angle_min=-math.pi, angle_increment=increment,
                range_min=.001, range_max=12.)


def moving_laser_poses(count=180):
    fractions = np.linspace(0., 1., count)
    bases = np.column_stack((1.05 + .7 * fractions, 2.15 - .2 * fractions,
                             .4 + .45 * fractions))
    # A rotating base moves its offset laser along an arc as well as translating.
    lasers = bases.copy()
    lasers[:, 0] += .35 * np.cos(bases[:, 2]) + .15 * np.sin(bases[:, 2])
    lasers[:, 1] += .35 * np.sin(bases[:, 2]) - .15 * np.cos(bases[:, 2])
    return bases, lasers


def test_moving_scan_uses_acquisition_laser_poses_instead_of_a_rigid_scan_pose():
    grid = room_grid()
    _, poses = moving_laser_poses()
    scan = moving_scan(grid, poses)
    quality = map_quality(grid, tolerance=.05)
    corrected = quality.score(**scan, laser_pose=poses[0], ray_poses=poses)
    rigid = quality.score(**scan, laser_pose=poses[0])
    assert corrected.origin_free
    assert corrected.valid_beams == len(poses)
    assert corrected.match_ratio > .98
    assert rigid.match_ratio < .6


def test_motion_correction_requires_laser_extrinsic_and_handles_rotated_map_origin():
    grid = room_grid()
    bases, lasers = moving_laser_poses()
    scan = moving_scan(grid, lasers)
    # Also verify selected-ray alignment when scoring only a subset of the scan.
    indices, _, _ = MapQuality.sampled_rays(
        scan['ranges'], scan['angle_min'], scan['angle_increment'],
        scan['range_min'], scan['range_max'], 60, 30.,
    )
    selected = lasers[indices]
    plain = map_quality(grid, tolerance=.05).score(
        **scan, laser_pose=lasers[0], max_beams=60, ray_poses=selected,
    )
    wrong_extrinsic = map_quality(grid, tolerance=.05).score(
        **scan, laser_pose=bases[0], max_beams=60, ray_poses=bases[indices],
    )
    origin = (12., -7., math.pi / 2)
    world_poses = np.column_stack((origin[0] - selected[:, 1],
                                  origin[1] + selected[:, 0], selected[:, 2] + origin[2]))
    world_reference = (origin[0] - lasers[0, 1], origin[1] + lasers[0, 0],
                       lasers[0, 2] + origin[2])
    rotated = map_quality(grid, origin=origin, tolerance=.05).score(
        **scan, laser_pose=world_reference, max_beams=60, ray_poses=world_poses,
    )
    assert plain.match_ratio > .98
    assert wrong_extrinsic.match_ratio < .6
    assert rotated == plain


@pytest.mark.parametrize('interior,expected_known,expected_obstructed', [(-1, 1, 0), (100, 2, 1)])
def test_each_ray_path_keeps_its_own_unknown_space_and_wall_obstruction(
    interior, expected_known, expected_obstructed,
):
    grid = np.zeros((20, 50), dtype=int)
    grid[:, 40] = 100
    grid[:10, 20:23] = interior
    # Both endpoints hit x=4.05. Only the lower ray crosses the intermediate
    # wall/unknown region; using the upper reference origin for LOS is incorrect.
    poses = [(.55, .55, 0.), (.55, 1.55, -.1)]
    result = map_quality(grid, tolerance=0.).score(
        [3.5, 3.5], 0., .1, .01, 10., (.55, 1.55, 0.),
        ray_step=.1, ray_poses=poses,
    )
    assert result.origin_free
    assert result.valid_beams == 2
    assert result.known_beams == expected_known
    assert result.obstructed_beams == expected_obstructed
    assert result.matched_beams == 1
    assert result.match_ratio == .5


def test_moving_ray_unknown_and_outside_endpoints_stay_in_the_denominator():
    grid = np.zeros((10, 10), dtype=int)
    grid[5, 6] = 100
    grid[6, 4] = -1
    poses = [(.45, .55, 0.), (.45, .55, 0.), (.55, .45, 0.), (.55, .55, 0.)]
    result = map_quality(grid, tolerance=0.).score(
        [.2, .1, 2., .2], 0., math.pi / 2, .01, 10., poses[0],
        ray_step=1., ray_poses=poses,
    )
    assert result.origin_free
    assert result.valid_beams == 4
    assert result.matched_beams == 1
    assert result.known_beams == 2
    assert result.match_ratio == .25
    assert result.known_ratio == .5


@pytest.mark.parametrize('invalid_origin', [(0.05, .05, 0.), (.55, .55, 0.), (-.05, 2., 0.)])
@pytest.mark.parametrize('which_origin', ['reference', 'sampled_ray'])
def test_reference_and_every_acquisition_origin_must_be_free(invalid_origin, which_origin):
    grid = room_grid()
    grid[5, 5] = -1
    reference = (2.05, 2.55, 0.)
    poses = [reference, reference]
    if which_origin == 'reference':
        reference = invalid_origin
    else:
        poses[1] = invalid_origin
    result = map_quality(grid).score([1., 1.], 0., .1, .01, 12., reference, ray_poses=poses)
    assert not result.origin_free
    assert result.valid_beams == 2


def test_stationary_ray_poses_preserve_existing_score_exactly():
    grid = room_grid()
    pose = (2.05, 2.55, .1)
    scan = synthetic_scan(grid, pose)
    quality = map_quality(grid)
    plain = quality.score(**scan, laser_pose=pose)
    corrected = quality.score(**scan, laser_pose=pose, ray_poses=[pose] * 120)
    assert corrected == plain


@pytest.mark.parametrize('poses', [
    [], [(2., 2., 0.)], [(2., 2., 0.)] * 3,
    [(2., 2.), (2., 2.)], np.zeros((2, 3, 1)),
    [(2., 2., 0.), (float('nan'), 2., 0.)],
    [(2., 2., 0.), (2., float('inf'), 0.)],
    [(2., 2., 0.), (2., 2., -float('inf'))],
])
def test_malformed_ray_poses_are_rejected_without_dropping_the_bad_ray(poses):
    with pytest.raises(ValueError):
        map_quality(room_grid()).score(
            [1., 1.], 0., .1, .01, 12., (2., 2., 0.), ray_poses=poses,
        )


def test_empty_scan_requires_an_empty_pose_array_when_acquisition_poses_are_supplied():
    quality = map_quality(room_grid())
    args = ([float('inf')], 0., .1, .01, 12., (2., 2., 0.))
    result = quality.score(*args, ray_poses=np.empty((0, 3)))
    assert result.origin_free
    assert result.valid_beams == 0
    assert result.match_ratio == 0.
    with pytest.raises(ValueError):
        quality.score(*args, ray_poses=[(2., 2., 0.)])


# Frozen pre-optimization reference; do not share the optimized algorithm.
def dense_score_reference(
    self, ranges, angle_min, angle_increment, range_min, range_max, laser_pose,
    max_beams=180, max_range=30.0, ray_step=0.2, ray_poses=None,
):
    pose = np.asarray(laser_pose, dtype=float)
    if pose.shape != (3,) or not np.isfinite(pose).all() or not math.isfinite(ray_step) or ray_step <= 0:
        raise ValueError('Invalid laser pose or ray sampling step')
    distances, angles = self._rays(
        ranges, angle_min, angle_increment, range_min, range_max, max_beams, max_range)
    ox, oy, in_map = self._cells(pose[0], pose[1])
    origin_free = bool(in_map & self.free[oy, ox])
    count = len(distances)
    if ray_poses is None:
        poses = np.broadcast_to(pose, (count, 3))
    else:
        poses = np.asarray(ray_poses, dtype=float)
        if poses.shape != (count, 3) or not np.isfinite(poses).all():
            raise ValueError('Ray poses must contain one finite map-frame laser pose per selected ray')
    if not count:
        return ScanQuality(0, 0, 0, 0, 0.0, 0.0, 0.0, origin_free)
    ox, oy, in_map = self._cells(poses[:, 0], poses[:, 1])
    origin_free = origin_free and bool(np.all(in_map & self.free[oy, ox]))
    cosine, sine = np.cos(angles + poses[:, 2]), np.sin(angles + poses[:, 2])
    gx, gy, inside = self._cells(poses[:, 0] + distances * cosine,
                               poses[:, 1] + distances * sine)
    known = inside & self.known[gy, gx]
    matched = known & self.near_occupied[gy, gx]
    obstructed = np.zeros(count, dtype=bool)
    # Count all measured returns in denominators, including points outside
    # the saved map.  Discarding them can make the wrong map pose score 100%.
    clearance = max(self.match_tolerance, self.resolution)
    steps = np.arange(ray_step, max(0.0, distances.max() - clearance), ray_step)
    if len(steps):
        active = steps[None, :] < (distances[:, None] - clearance)
        rx, ry, rinside = self._cells(
            poses[:, 0, None] + cosine[:, None] * steps,
            poses[:, 1, None] + sine[:, None] * steps)
        obstructed = np.any(active & rinside & self.occupied[ry, rx], axis=1)
        traversable = rinside & self.known[ry, rx]
        known &= np.all(~active | traversable, axis=1)
        matched &= known & ~obstructed
    return ScanQuality(
        count, int(matched.sum()), int(known.sum()), int(obstructed.sum()),
        float(matched.mean()), float(known.mean()), float(obstructed.mean()), origin_free)


# Frozen pre-optimization reference; do not share the optimized algorithm.
def loop_tracking_ray_poses_reference(self, laser_pose, helpers):
    compose_se2, inverse_se2 = helpers['compose_se2'], helpers['inverse_se2']
    stamp_ns, Time = helpers['stamp_ns'], helpers['Time']
    self.metrics.update(scan_motion_compensated=False, scan_motion_translation=0.0,
                        scan_motion_yaw=0.0, scan_duration=0.0)
    scan = self.scan
    if not self.prior_verified or scan.time_increment == 0:
        return None
    if not self.valid_scan(scan):
        raise ValueError('Invalid rolling scan metadata')
    args = self.scan_args()
    indices, _, _ = self.map_quality.sampled_rays(**args)
    start_ns = stamp_ns(scan.header.stamp)
    start = self.checked_transform(self.cfg['odom_frame'], scan.header.frame_id,
                                   Time.from_msg(scan.header.stamp))
    correction = compose_se2(laser_pose, inverse_se2(start))
    poses = []
    duration = (len(scan.ranges)-1)*scan.time_increment
    # Check even an invalid/no-return final beam's time. Completeness must
    # not depend on which ranges happened to survive subsampling.
    end = self.checked_transform(self.cfg['odom_frame'], scan.header.frame_id,
                                 Time(nanoseconds=start_ns + round(duration*1e9)))
    inverse_start = inverse_se2(start)
    delta = compose_se2(inverse_start, end)
    translation, rotation = math.hypot(delta[0], delta[1]), abs(delta[2])
    # One lookup per ray (360 per tick) was 15 % of this node's CPU on the
    # Jetson. Odometry TF arrives at 30 Hz and tf2 interpolates linearly
    # between samples, so 10 ms knots lose nothing; interpolate the rest.
    knot_times = np.arange(0.0, duration, 0.01)
    if not len(knot_times) or knot_times[-1] < duration:
        knot_times = np.append(knot_times, duration)
    knots = []
    for elapsed in knot_times:
        odom_laser = (end if elapsed >= duration else self.checked_transform(
            self.cfg['odom_frame'], scan.header.frame_id, Time(nanoseconds=start_ns + round(elapsed*1e9))))
        delta = compose_se2(inverse_start, odom_laser)
        translation = max(translation, math.hypot(delta[0], delta[1]))
        rotation = max(rotation, abs(delta[2]))
        knots.append(odom_laser)
    knots = np.asarray(knots, dtype=float)
    unwrapped = np.unwrap(knots[:, 2])
    ray_times = np.asarray(indices, dtype=float)*scan.time_increment
    for ray_time in ray_times:
        odom_laser = (float(np.interp(ray_time, knot_times, knots[:, 0])),
                      float(np.interp(ray_time, knot_times, knots[:, 1])),
                      float(np.interp(ray_time, knot_times, unwrapped)))
        poses.append(compose_se2(correction, odom_laser))
    self.metrics.update(scan_motion_compensated=True, scan_duration=duration,
                        scan_motion_translation=translation, scan_motion_yaw=rotation)
    return np.asarray(poses, dtype=float).reshape(-1, 3)
