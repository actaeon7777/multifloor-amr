import math
import statistics
import time

import numpy as np
import pytest

from amr_patrol.detection import Grid, MotionDetector, Settings, apply_update
from amr_patrol.routes import Exclusion


def blank():
    return Grid(np.zeros((200, 200), dtype=np.int8), .05)


def blob(x=60, y=60, size=10):
    grid = blank()
    grid.data[y:y + size, x:x + size] = 100
    return grid


def detector(static=None, exclusions=(), **settings):
    settings.setdefault('warmup_s', 0.)
    result = MotionDetector(Settings(**settings), exclusions)
    result.set_static(static if static is not None else blank())
    return result


def frame(subject, grid, stamp, **kwargs):
    return subject.process(grid, (1., 1., 0.), '4F', stamp, **kwargs)


def test_wall_margin_and_unknown_and_inflation():
    static = blank()
    static.data[:, 59] = 100
    grid = blank()
    grid.data[60:70, 60:62] = 100
    assert detector(static).clusters(grid, (1, 1, 0), '4F') == []
    assert len(detector(static, static_margin_cells=0).clusters(grid, (1, 1, 0), '4F')) == 1
    static.data[60:70, 60:70] = -1
    assert detector(static).clusters(blob(), (1, 1, 0), '4F') == []
    for value in (99, 80, -1):
        grid.data[:] = value
        assert detector().clusters(grid, (1, 1, 0), '4F') == []


def test_blob_and_motion_two_frames_later():
    subject = detector()
    clusters = subject.clusters(blob(), (1, 1, 0), '4F')
    assert len(clusters) == 1
    assert clusters[0].x == pytest.approx(3.25)
    assert clusters[0].diameter_m == pytest.approx(math.sqrt(.5))
    assert clusters[0].cells == 100
    assert frame(subject, blob(), 0) == []
    assert frame(subject, blob(), 1) == []
    result, = frame(subject, blob(70), 2)
    assert (result.x, result.y) == pytest.approx((3.75, 3.25))
    assert result.displacement_m == pytest.approx(.5)
    assert result.track_id == 1


def test_stationary_blob_and_duplicate_time():
    subject = detector()
    for stamp in range(10):
        assert frame(subject, blob(), stamp) == []
    assert frame(subject, blob(70), 9) == []
    assert subject.tracks[1].frames == 10


def test_exclusion_floor_and_size_and_range():
    exclusion = Exclusion('4F', 2.9, 4, 2.9, 4)
    subject = detector(exclusions=(exclusion,))
    assert subject.clusters(blob(), (1, 1, 0), '4F') == []
    assert len(subject.clusters(blob(), (1, 1, 0), '3F')) == 1
    assert detector().clusters(blob(size=40), (1, 1, 0), '4F') == []
    assert detector().clusters(blob(size=1), (1, 1, 0), '4F') == []
    assert detector().clusters(blob(18, 18, 4), (1, 1, 0), '4F') == []
    assert detector().clusters(blob(180, 180), (1, 1, 0), '4F') == []


def test_repeat_debounce_and_expiry():
    subject = detector(global_repeat_s=0., move_window_s=3.)
    assert frame(subject, blob(), 0) == []
    assert len(frame(subject, blob(70), 1)) == 1
    for stamp in range(2, 31):
        assert frame(subject, blob(60 if stamp % 2 == 0 else 70), stamp) == []
    assert len(frame(subject, blob(70), 31)) == 1


def test_global_debounce_across_tracks():
    subject = detector()
    grid = blob()
    grid.data[100:110, 60:70] = 100
    assert frame(subject, grid, 0) == []
    moved = blob(70)
    moved.data[100:110, 70:80] = 100
    assert len(frame(subject, moved, 1)) == 1
    assert frame(subject, grid, 2) == []
    assert len(subject.tracks) == 2


def test_turning_skips_and_breaks_track_history():
    subject = detector()
    frame(subject, blob(), 0)
    assert frame(subject, blob(70), 1, yaw_rate=.5) == []
    assert not subject.tracks
    assert frame(subject, blob(70), 2) == []
    assert len(frame(subject, blob(80), 3)) == 1


def test_tracking_window_timeout_and_time_rewind():
    subject = detector()
    frame(subject, blob(), 0)
    assert frame(subject, blob(70), 4.5) == []  # Old position left movement window.
    assert frame(subject, blob(80), 10) == []  # Old track expired.
    assert len(subject.tracks) == 1
    assert frame(subject, blob(90), 1) == []  # Clock rewind resets tracking.
    assert len(subject.tracks) == 1


def test_one_to_one_match_and_min_frames():
    subject = detector(min_frames=3, global_repeat_s=0.)
    frame(subject, blob(), 0)
    assert frame(subject, blob(70), 1) == []
    grid = blob(80)
    grid.data[72:82, 70:80] = 100
    assert len(frame(subject, grid, 2)) == 1
    assert sorted(track.frames for track in subject.tracks.values()) == [1, 3]


def test_eight_connected_component():
    grid = blank()
    for index in range(60, 65):
        grid.data[index, index] = 100
    cluster, = detector().clusters(grid, (1, 1, 0), '4F')
    assert cluster.cells == 5


def test_grid_origin_resolution_rotation_and_outside_map():
    static = blank()
    static.data[60:70, 60:70] = 100
    # Same world blob, shifted origin and double resolution.
    costmap = Grid(np.zeros((100, 100), dtype=np.int8), .1, (1., 1., 0.))
    costmap.data[20:25, 20:25] = 100
    assert detector(static).clusters(costmap, (1, 1, 0), '4F') == []
    cluster, = detector().clusters(costmap, (1, 1, 0), '4F')
    assert (cluster.x, cluster.y) == pytest.approx((3.25, 3.25))
    rotated = Grid(blob().data, .05, (10., 0., math.pi / 2))
    cluster, = detector().clusters(rotated, (5, 1, 0), '4F')
    assert (cluster.x, cluster.y) == pytest.approx((6.75, 3.25))
    outside = Grid(blob().data, .05, (-20., 0., 0.))
    assert detector().clusters(outside, (-15, 1, 0), '4F') == []


def test_patch_is_atomic_and_updates_only_rectangle():
    grid = blank()
    apply_update(grid, 10, 20, 2, 3, [100, 99, 0, -1, 50, 100])
    np.testing.assert_array_equal(grid.data[20:23, 10:12], [[100, 99], [0, -1], [50, 100]])
    assert np.count_nonzero(grid.data) == 5
    before = grid.data.copy()
    for args in ((199, 0, 2, 1, [1, 1]), (-1, 0, 1, 1, [1]), (0, 0, 2, 2, [1])):
        with pytest.raises(ValueError):
            apply_update(grid, *args)
        np.testing.assert_array_equal(grid.data, before)


@pytest.mark.parametrize('settings', [
    {'static_margin_cells': -1}, {'static_margin_cells': True}, {'min_cells': 0},
    {'min_frames': 1}, {'match_radius': float('nan')}, {'max_yaw_rate': -1},
    {'repeat_s': -1}, {'max_range': .5}, {'min_diameter_m': 2},
    {'move_window_s': 0}, {'track_timeout_s': float('inf')},
    {'max_person_speed': 0}, {'max_person_speed': float('nan')},
    {'vacated_fraction': 0}, {'vacated_fraction': 1.01},
    {'repeat_radius_m': -1}, {'warmup_s': -1},
])
def test_invalid_settings(settings):
    with pytest.raises(ValueError):
        Settings(**settings)


def test_cached_frame_budget_at_robot_costmap_size():
    """Small synthetic CPU check; no ROS, sensors, or long-running benchmark."""
    static = Grid(np.zeros((462, 1014), dtype=np.int8), .05)
    subject = detector(static)
    costmap = Grid(static.data, .05)
    for y in (160, 200, 240, 280):
        for x in (440, 480, 520, 560):
            costmap.data[y:y + 10, x:x + 10] = 100
    subject.process(costmap, (25., 11., 0.), '4F', 0.)
    samples = []
    for stamp in range(1, 6):
        costmap.data = np.roll(costmap.data, 2, axis=1)
        start = time.perf_counter()
        subject.process(costmap, (25., 11., 0.), '4F', float(stamp))
        samples.append(time.perf_counter() - start)
    assert statistics.median(samples) < .05, f'frame times (ms): {[t * 1000 for t in samples]}'


@pytest.mark.parametrize('x,robot_xs', [(160, (.2, .6, 1.3)), (60, (2.9, 2.5, 2.))])
def test_static_object_at_range_boundary_does_not_move_with_robot(x, robot_xs):
    grid = blank()
    grid.data[63:67, x:x + 20] = 100
    subject = detector(move_threshold_m=.1)
    for stamp, robot_x in enumerate(robot_xs):
        pose = (robot_x, 3.25, 0.)
        assert subject.process(grid, pose, '4F', stamp) == []
        clusters = subject.clusters(grid, pose, '4F')
        if stamp == 0:
            assert not clusters  # Centroid may be in range, but the bounding box intersects it.
        for cluster in clusters:
            assert cluster.x == pytest.approx((x + 10) * .05)
            assert cluster.cells == 80  # Never a clipped fragment.
    assert len(subject.clusters(grid, pose, '4F')) == 1


def test_range_boundary_tangent_bounding_box_is_dropped():
    grid = blob(60, 60)
    subject = detector(ignore_radius=.5)
    assert subject.clusters(grid, (2.5, 3.25, 0.), '4F') == []


@pytest.mark.parametrize('missing', [blank(), blob(size=40)])
def test_occlusion_or_size_dropout_breaks_correspondence(missing):
    subject = detector()
    frame(subject, blob(), 0)
    assert frame(subject, missing, 1) == []
    assert frame(subject, blob(70), 2) == []
    assert len(subject.tracks) == 2  # Old track is alive but cannot be matched across the gap.
    assert sorted(track.frames for track in subject.tracks.values()) == [1, 1]


def test_match_radius_bounded_by_elapsed_time_and_person_speed():
    subject = detector()
    frame(subject, blob(), 0)
    assert frame(subject, blob(70), .1) == []  # 5 m/s is implausible.
    assert len(subject.tracks) == 2
    slow = detector(max_person_speed=.4)
    frame(slow, blob(), 0)
    assert frame(slow, blob(70), 1) == []


def test_growth_does_not_vacate_origin_but_walk_does():
    subject = detector()
    original = blob(size=4)
    frame(subject, original, 0)
    grown = blank()
    grown.data[60:64, 60:80] = 100
    assert frame(subject, grown, 1) == []  # Centre moved 0.4 m, all original cells remain lethal.
    assert subject.tracks[1].frames == 2
    walked = blob(80, size=4)
    detection, = frame(subject, walked, 2)
    assert detection.displacement_m == pytest.approx(1.)
    assert max(len(sample.footprint) for _, sample in subject.tracks[1].history) <= 64


def test_vacated_fraction_is_required_even_after_centroid_motion():
    subject = detector(move_threshold_m=.15, vacated_fraction=1.)
    frame(subject, blob(), 0)
    assert frame(subject, blob(64), 1) == []  # Partial overlap is insufficient at fraction=1.
    assert len(frame(subject, blob(70), 2)) == 1


def test_uncovered_lethal_cells_are_not_candidates():
    subject = detector()
    coverage = np.zeros((200, 200), dtype=bool)
    assert frame(subject, blob(), 0, coverage=coverage) == []
    assert not subject.tracks
    coverage[60:70, 60:70] = True
    assert frame(subject, blob(), 1, coverage=coverage) == []
    assert len(subject.tracks) == 1
    # Stale lethal data outside the updated rectangle must not create a new object.
    assert frame(subject, blob(70), 2, coverage=coverage) == []
    assert all(t.frames == 1 for t in subject.tracks.values())
    coverage[60:70, 70:80] = True
    assert frame(subject, blob(70), 3, coverage=coverage) == []  # No matching across missed frame.


def test_warmup_tracks_motion_without_consuming_repeat_suppression():
    assert Settings().warmup_s == 5.
    subject = detector(warmup_s=5.)
    subject.reset(stamp=100.)  # Activation, before the first map/frame.
    subject.set_static(blank())
    assert frame(subject, blob(), 103) == []
    assert frame(subject, blob(70), 104) == []
    assert subject.tracks[1].frames == 2
    detection, = frame(subject, blob(80), 105)
    assert detection.track_id == 1 and detection.displacement_m == pytest.approx(1.)


def test_spatial_repeat_survives_turn_clear_and_track_expiry():
    subject = detector(global_repeat_s=0.)
    frame(subject, blob(), 0)
    assert len(frame(subject, blob(70), 1)) == 1
    assert frame(subject, blob(70), 2, yaw_rate=.5) == []
    assert not subject.tracks
    frame(subject, blob(), 3)
    assert frame(subject, blob(70), 4) == []
    subject.tracks.clear()  # Independent of track identity and its last_detection field.
    frame(subject, blob(), 5)
    assert frame(subject, blob(70), 6) == []
    frame(subject, blank(), 12)
    assert not subject.tracks
    frame(subject, blob(), 13)
    assert frame(subject, blob(70), 14) == []
    frame(subject, blob(), 32)
    assert len(frame(subject, blob(70), 33)) == 1
