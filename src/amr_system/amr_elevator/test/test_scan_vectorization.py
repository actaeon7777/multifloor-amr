"""Compare optimized scan processing with the original scalar predicates."""

import math

import numpy as np
import pytest

from amr_elevator.door_detector import DoorDetector, OPEN, UNKNOWN, enclosure_ratio
from amr_elevator.geometry import ray_segment_distance
from amr_elevator.maneuver import travel_clearance


def reference_measure(detector, laser_pose, door, ranges, angle_min, angle_increment,
                      range_min=.05, range_max=30.):
    x, y, yaw = laser_pose
    passed = valid = 0
    for index, value in enumerate(ranges):
        distance = ray_segment_distance((x, y), yaw + angle_min + index * angle_increment,
                                        door[0], door[1], detector.jamb_margin)
        if distance is None or distance > detector.max_view_distance:
            continue
        if math.isnan(value) or value < range_min:
            continue
        valid += 1
        if math.isinf(value) or value >= range_max or value > distance + detector.pass_margin:
            passed += 1
    return (passed / valid if valid else 0.), valid


def reference_enclosure(ranges, range_min=.05, near=3.):
    valid = [r for r in ranges if not math.isnan(r) and r >= range_min]
    if not valid:
        return 0.
    return sum(1 for r in valid if r < near) / len(valid)


def reference_clearance(ranges, angle_min, angle_increment, reverse, half_length=.32,
                        half_width=.3, laser_pose=(0., 0., 0.)):
    lx, ly, lyaw = laser_pose
    best = math.inf
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value <= 0.:
            continue
        angle = lyaw + angle_min + index * angle_increment
        x, y = lx + value * math.cos(angle), ly + value * math.sin(angle)
        if reverse:
            x = -x
        if x > 0 and abs(y) <= half_width:
            best = min(best, x - half_length)
    return best


def random_scan(seed, count=3240):
    ranges = np.random.default_rng(seed).uniform(-.1, 8., count)
    ranges[::31] = np.nan
    ranges[1::31] = np.inf
    ranges[2::31] = -np.inf
    ranges[3::31] = 0.
    ranges[4::31] = .05
    ranges[5::31] = 30.
    ranges[6::31] = 3.
    return ranges


@pytest.mark.parametrize('as_list', [False, True])
@pytest.mark.parametrize('laser,door', [
    ((.6, 0., math.pi), ((0., .55), (0., -.55))),
    ((-.6, .1, .27), ((0., .55), (0., -.55))),
    ((6., 0., math.pi), ((0., .55), (0., -.55))),
    ((1., 1., -.73), ((2., 1.5), (1.5, 2.5))),
    ((0., 0., 0.), ((1., 0.), (1., 0.))),
    ((0., 0., 0.), ((4., -.55), (4., .55))),
])
def test_measure_equals_loop(laser, door, as_list):
    detector = DoorDetector()
    for seed in range(5):
        ranges = random_scan(seed)
        if as_list:
            ranges = ranges.tolist()
        args = laser, door, ranges, -math.pi, 2 * math.pi / len(ranges)
        assert detector.measure(*args) == reference_measure(detector, *args)


@pytest.mark.parametrize('y', [-.55, -.47, 0., .47, .55])
@pytest.mark.parametrize('delta', [-1e-14, 0., 1e-14])
def test_measure_jamb_and_pass_boundaries(y, delta):
    detector = DoorDetector()
    door = ((1., -.55), (1., .55))
    angle = math.atan2(y + delta, 1.)
    distance = ray_segment_distance((0., 0.), angle, *door, detector.jamb_margin)
    value = (distance if distance is not None else 1.) + detector.pass_margin
    for beam in [np.nextafter(value, -np.inf), value, np.nextafter(value, np.inf),
                 .05, .049, 30., np.nan, np.inf, -np.inf]:
        args = (0., 0., 0.), door, [beam], angle, 0.
        assert detector.measure(*args) == reference_measure(detector, *args)


def test_far_door_does_not_read_beams_and_resets_unknown():
    class UnreadableScan:
        def __len__(self):
            raise AssertionError('far door must not touch the beams')

        def __iter__(self):
            raise AssertionError('far door must not touch the beams')

        def __array__(self, *args, **kwargs):
            raise AssertionError('far door must not touch the beams')

    detector = DoorDetector()
    detector.state, detector.previous_ratio, detector.streak = OPEN, 1., (OPEN, 3)
    reading = detector.update((0., 0., 0.), ((1., 5.), (1., 6.)), UnreadableScan(), 0., .1)
    assert (reading.state, reading.open_ratio, reading.valid_beams) == (UNKNOWN, 0., 0)
    assert detector.previous_ratio is None and detector.streak == (None, 0)


def test_beam_cache_reuses_layout_but_respects_yaw_and_layout_changes():
    detector = DoorDetector()
    door = ((1., -.55), (1., .55))
    previous_angles = None
    for count, amin, inc, yaw in [(100, -.5, .01, 0.), (100, -.5, .01, .37),
                                  (101, -.5, .01, 0.), (101, -.4, .01, 0.),
                                  (101, -.4, .02, 0.)]:
        args = (0., 0., yaw), door, random_scan(1, count), amin, inc
        assert detector.measure(*args) == reference_measure(detector, *args)
        if yaw == .37:
            assert detector._beam_angles is previous_angles
        else:
            assert detector._beam_angles is not previous_angles
        previous_angles = detector._beam_angles


@pytest.mark.parametrize('as_list', [False, True])
@pytest.mark.parametrize('seed', range(10))
def test_enclosure_and_clearance_equal_random_loops(seed, as_list):
    ranges = random_scan(seed)
    if as_list:
        ranges = ranges.tolist()
    assert enclosure_ratio(ranges) == reference_enclosure(ranges)
    for pose in [(0., 0., 0.), (.12, -.03, math.pi), (-.07, .11, .813)]:
        for reverse in [False, True]:
            args = ranges, -math.pi, 2 * math.pi / len(ranges), reverse
            expected = reference_clearance(*args, laser_pose=pose)
            assert travel_clearance(*args, laser_pose=pose) == expected


@pytest.mark.parametrize('ranges', [[], [np.nan, np.inf, -np.inf, 0., -.1]])
def test_no_qualifying_beams(ranges):
    assert enclosure_ratio(ranges) == reference_enclosure(ranges)
    for reverse in [False, True]:
        assert travel_clearance(ranges, 0., .1, reverse) == math.inf


@pytest.mark.parametrize('reverse', [False, True])
def test_clearance_body_width_boundary(reverse):
    for angle in [.3, -.3, 2.8, -2.8]:
        value = .3 / abs(math.sin(angle))
        for beam in [np.nextafter(value, -np.inf), value, np.nextafter(value, np.inf)]:
            args = [beam], angle, 0., reverse
            assert travel_clearance(*args) == reference_clearance(*args)
