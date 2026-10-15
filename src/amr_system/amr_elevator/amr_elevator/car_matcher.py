"""Correct the odometry-carried pose inside the car from the car's own walls (no ROS).

Inside the car the floor map is useless but the car interior is known exactly
(floors.yaml dimensions).  Scan points near the car walls are fitted to the wall
lines with a few Gauss-Newton point-to-line iterations; the result is accepted
only with enough points on walls of both directions and a small residual.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CarFit:
    dx: float
    dy: float
    dyaw: float
    pivot: tuple
    rms: float
    inliers: int


def car_wall_segments(geometry):
    """Car interior wall segments on the map; the front wall leaves the door span out."""
    near_right, far_right, far_left, near_left = geometry.car_polygon
    door_left, door_right = geometry.car_door
    return [(near_right, far_right), (far_right, far_left), (far_left, near_left),
            (near_left, door_left), (door_right, near_right)]


def _nearest(points, segments, end_margin=.05):
    """Signed perpendicular distance and normal to the nearest wall (inf if beside no wall).

    Corners are ambiguous, so points within ``end_margin`` of a wall end are not associated.
    """
    best = np.full(len(points), np.inf)
    signed = np.full(len(points), np.inf)
    normals = np.zeros((len(points), 2))
    for a, b in segments:
        a, b = np.asarray(a, float), np.asarray(b, float)
        length = float(np.hypot(*(b - a)))
        unit = (b - a) / length
        normal = np.array([-unit[1], unit[0]])
        rel = points - a
        along = rel @ unit
        perp = rel @ normal
        distance = np.where((along >= end_margin) & (along <= length - end_margin), np.abs(perp), np.inf)
        better = distance < best
        best[better] = distance[better]
        signed[better] = perp[better]
        normals[better] = normal
    return signed, normals


def fit_car(points, segments, pivot, max_distance=.25, iterations=8, min_points=40, min_per_axis=15,
            max_rms=.04):
    """Correction (about ``pivot``) that moves ``points`` onto the walls, or None."""
    points = np.asarray(points, float)
    if len(points) < min_points:
        return None
    pivot = np.asarray(pivot, float)
    total = np.zeros(3)
    current = points.copy()
    for iteration in range(iterations):
        distance, normal = _nearest(current, segments)
        # Coarse association first, then tighten to reject door edges and clutter.
        mask = np.abs(distance) < (max_distance if iteration < 2 else max(.06, max_distance / 3))
        if mask.sum() < min_points:
            return None
        n, d = normal[mask], distance[mask]
        rel = current[mask] - pivot
        perp = np.column_stack((-rel[:, 1], rel[:, 0]))
        jacobian = np.column_stack((n, np.sum(n * perp, axis=1)))
        step, *_ = np.linalg.lstsq(jacobian, -d, rcond=None)
        c, s = math.cos(step[2]), math.sin(step[2])
        rotation = np.array([[c, -s], [s, c]])
        current = (current - pivot) @ rotation.T + pivot + step[:2]
        total = _compose(total, step)
        if np.hypot(step[0], step[1]) < 1e-4 and abs(step[2]) < 1e-4:
            break
    distance, normal = _nearest(current, segments)
    mask = np.abs(distance) < max_distance / 2
    n = normal[mask]
    along_x = int(np.sum(np.abs(n[:, 0]) > .9))
    along_y = int(np.sum(np.abs(n[:, 1]) > .9))
    if mask.sum() < min_points or min(along_x, along_y) < min_per_axis:
        return None
    rms = float(np.sqrt(np.mean(distance[mask] ** 2)))
    if rms > max_rms:
        return None
    return CarFit(float(total[0]), float(total[1]), float(total[2]), tuple(pivot), rms, int(mask.sum()))


def _compose(total, step):
    """Accumulate small corrections about the same pivot."""
    c, s = math.cos(step[2]), math.sin(step[2])
    x = c * total[0] - s * total[1] + step[0]
    y = s * total[0] + c * total[1] + step[1]
    return np.array([x, y, total[2] + step[2]])


def apply_fit(pose, fit, gain=1.):
    """Pose corrected by ``gain`` x the fit (rotation about the fit pivot)."""
    angle = gain * fit.dyaw
    c, s = math.cos(angle), math.sin(angle)
    px, py = fit.pivot
    x, y = pose[0] - px, pose[1] - py
    return (c * x - s * y + px + gain * fit.dx, s * x + c * y + py + gain * fit.dy,
            math.atan2(math.sin(pose[2] + angle), math.cos(pose[2] + angle)))
