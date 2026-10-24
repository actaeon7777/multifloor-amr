"""Evidence gates must reject plausible but unsupported localization poses."""

import math

import numpy as np
import pytest

from amr_stack.localization_quality import (
    MapQuality, covariance_is_bounded, particle_concentration,
)


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
