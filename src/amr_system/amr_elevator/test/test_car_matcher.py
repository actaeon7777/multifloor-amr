"""Car-wall pose correction recovers odometry drift inside the car."""

import math

import numpy as np
import pytest

from amr_elevator.car_matcher import apply_fit, car_wall_segments, fit_car
from amr_elevator.geometry import ray_segment_distance
from amr_stack.floor_config import load_floor_config


@pytest.fixture(scope='module')
def geometry():
    return load_floor_config().elevator_geometry('4F', 'E1')


def world(geometry, door_open):
    near_right, far_right, far_left, near_left = geometry.car_polygon
    door_left, door_right = geometry.car_door
    walls = [(near_right, far_right), (far_right, far_left), (far_left, near_left),
             (near_left, door_left), (door_right, near_right)]
    if not door_open:
        walls.append((door_left, door_right))
    else:
        walls += [((9.975, 3.86), (9.975, 8.)), ((9.975, 2.76), (9.975, -2.))]  # hall wall
    return walls


def scan_points(true_pose, believed_pose, walls, count=720):
    """Scan from the true pose, projected with the believed pose (as the monitor does)."""
    points = []
    for i in range(count):
        angle = -math.pi + i * 2 * math.pi / count
        hits = [d for a, b in walls
                if (d := ray_segment_distance(true_pose[:2], true_pose[2] + angle, a, b)) is not None]
        if not hits or min(hits) > 8.:
            continue
        r = min(hits)
        heading = believed_pose[2] + angle
        points.append((believed_pose[0] + r * math.cos(heading), believed_pose[1] + r * math.sin(heading)))
    return np.array(points)


@pytest.mark.parametrize('door_open', [False, True])
@pytest.mark.parametrize('error', [(.2, -.15, .1), (-.25, .3, -.15), (0., 0., 0.)])
def test_recovers_drift(geometry, door_open, error):
    truth = (geometry.car_pose[0] + .1, geometry.car_pose[1] - .05, .3)
    believed = (truth[0] + error[0], truth[1] + error[1], truth[2] + error[2])
    points = scan_points(truth, believed, world(geometry, door_open))
    fit = fit_car(points, car_wall_segments(geometry), believed[:2], max_distance=.5)
    assert fit is not None and fit.rms < .01
    corrected = apply_fit(believed, fit)
    assert math.dist(corrected[:2], truth[:2]) < .01
    assert abs(corrected[2] - truth[2]) < .005


def test_rejects_unconstrained_or_wrong_geometry(geometry):
    segments = car_wall_segments(geometry)
    truth = geometry.car_pose
    # Only one wall visible: not enough constraint.
    back = [segments[1]]
    points = scan_points(truth, truth, back)
    assert fit_car(points, segments, truth[:2]) is None
    # Robot in the hall: hall walls are not the car.
    hall = (11.5, 3.31, math.pi)
    points = scan_points(hall, hall, [((9.975, 3.86), (9.975, 8.)), ((9.975, 2.76), (9.975, -2.)),
                                      ((14., -2.), (14., 8.))])
    assert fit_car(points, segments, hall[:2]) is None
    assert fit_car(np.zeros((3, 2)), segments, (0., 0.)) is None
