"""LiDAR door sensing on synthetic scans."""

import math

import pytest

from amr_elevator.door_detector import (
    CLOSED, CLOSING, OPEN, OPENING, UNKNOWN, DoorDetector, enclosure_ratio,
)
from amr_elevator.geometry import (
    compose, depth_beyond, inverse, point_in_polygon, ray_segment_distance, relative,
)

DOOR = ((0., .55), (0., -.55))  # hall door plane x = 0, car to the west (-x)
COUNT = 720
ANGLE_MIN, INCREMENT = -math.pi, 2 * math.pi / COUNT


def scan(laser, opening, beyond=2.5, wall=1.2):
    """Ranges with the door panels open for |y| < opening/2 on the door plane."""
    x, y, yaw = laser
    ranges = []
    for i in range(COUNT):
        angle = yaw + ANGLE_MIN + i * INCREMENT
        if math.cos(angle) < -1e-6:
            t = -x / math.cos(angle)
            hit_y = y + t * math.sin(angle)
            if abs(hit_y) <= .55:
                ranges.append(t + beyond if abs(hit_y) < opening / 2 else t)
                continue
            if abs(hit_y) <= 1.5:
                ranges.append(t)  # hall wall beside the door
                continue
        ranges.append(wall)
    return ranges


LASER = (.6, 0., math.pi)


def feed(detector, opening, laser=LASER, **kwargs):
    return detector.update(laser, DOOR, scan(laser, opening, **kwargs), ANGLE_MIN, INCREMENT)


def test_closed_and_open_need_two_consistent_scans():
    detector = DoorDetector()
    assert feed(detector, 0.).state == UNKNOWN
    reading = feed(detector, 0.)
    assert reading.state == CLOSED and reading.open_ratio == 0. and reading.valid_beams > 100
    assert feed(detector, 1.1).state == CLOSED
    assert feed(detector, 1.1).state == OPEN


def test_partial_openings_report_motion():
    detector = DoorDetector()
    feed(detector, 0.)
    feed(detector, 0.)
    assert feed(detector, .4).state == OPENING
    assert feed(detector, .7).state == OPENING
    feed(detector, 1.1)
    assert feed(detector, 1.1).state == OPEN
    assert feed(detector, .45).state == CLOSING
    feed(detector, 0.)
    assert feed(detector, 0.).state == CLOSED


def test_jamb_margin_ignores_the_door_edges():
    detector = DoorDetector(jamb_margin=.1)
    ratio, _ = detector.measure(LASER, DOOR, scan(LASER, .9), ANGLE_MIN, INCREMENT)
    assert ratio > .95  # the 0.1 m strips at each end are closed panel edges


def test_out_of_view_and_missing_returns():
    detector = DoorDetector()
    far = (6., 0., math.pi)
    assert feed(detector, 1.1, laser=far).state == UNKNOWN  # beyond max_view_distance
    behind = (-.6, 0., 0.)  # door segment behind... still visible from the car side
    assert feed(detector, 0., laser=behind).valid_beams >= 10
    nan_scan = [float('nan')] * COUNT
    assert detector.update(LASER, DOOR, nan_scan, ANGLE_MIN, INCREMENT).state == UNKNOWN
    inf_scan = [float('inf')] * COUNT
    assert detector.measure(LASER, DOOR, inf_scan, ANGLE_MIN, INCREMENT)[0] == 1.


def test_invalid_thresholds():
    with pytest.raises(ValueError):
        DoorDetector(open_ratio=.2, closed_ratio=.5)


def test_enclosure_ratio():
    assert enclosure_ratio([1.] * 90 + [10.] * 10) == pytest.approx(.9)
    assert enclosure_ratio([float('nan'), .01]) == 0.


def test_geometry_helpers():
    a, b = (1., 2., .5), (-.3, .4, -1.2)
    assert compose(a, inverse(a)) == pytest.approx((0., 0., 0.), abs=1e-12)
    assert compose(a, relative(a, b)) == pytest.approx(b)
    square = [(0, 0), (1, 0), (1, 1), (0, 1)]
    assert point_in_polygon(.5, .5, square) and not point_in_polygon(1.5, .5, square)
    assert depth_beyond((-1., 0.), DOOR, math.pi) == pytest.approx(1.)
    assert ray_segment_distance((1., 0.), math.pi, *DOOR) == pytest.approx(1.)
    assert ray_segment_distance((1., 0.), 0., *DOOR) is None
    assert ray_segment_distance((1., .5), math.pi, *DOOR, end_margin=.1) is None
