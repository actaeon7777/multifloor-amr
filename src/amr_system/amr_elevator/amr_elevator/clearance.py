"""Sensor projections and elevator free-space checks (metres, radians; no ROS).

Regions expose ``contains(points_map)`` returning a boolean mask. Car polygons
must be convex and counter-clockwise. ``wall_margin`` is an inward offset of
each car edge, independent of the extra space requested around the robot.
"""

from dataclasses import dataclass
import math

import numpy as np


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, laser_pose):
    """Project finite, in-range LaserScan returns into the base frame."""
    values = np.asarray(ranges, dtype=float)
    indices = np.flatnonzero(np.isfinite(values) & (values >= range_min) & (values <= range_max))
    angles = laser_pose[2] + angle_min + indices * angle_increment
    values = values[indices]
    return np.column_stack((laser_pose[0] + values * np.cos(angles),
                            laser_pose[1] + values * np.sin(angles)))


def cloud_points_xy(xyz, transform, min_height=.06, max_height=1., max_range=3.):
    """Base-frame obstacle XY; transform is (translation[3], rotation[3,3]).

    Filter height and horizontal range *after* transformation. The upstream
    persistence filter also passes floor, ceiling and distant points unchanged.
    """
    points = np.asarray(xyz, dtype=float).reshape(-1, 3)
    points = points[np.all(np.isfinite(points), axis=1)]
    translation, rotation = transform
    points = points @ np.asarray(rotation).T + np.asarray(translation)
    valid = (np.all(np.isfinite(points), axis=1)
             & (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
             & (np.hypot(points[:, 0], points[:, 1]) <= max_range))
    return points[valid, :2]


def strip_clearance(points, reverse, half_length=.328, half_width=.307):
    """Bumper clearance in the forward/rear strip, including negative distances."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    along = points[:, 0] * (-1. if reverse else 1.)
    selected = (along > 0.) & (np.abs(points[:, 1]) <= half_width)
    return float(np.min(along[selected], initial=np.inf) - half_length)


def sweep_hits(points, delta_yaw, half_length=.348, half_width=.307, step=math.radians(2),
               body_half_length=.328, body_half_width=.287):
    """Check the signed rotation, sampling 0, step, ... and its exact endpoint.

    Ignore only the real body plus 0.01 m filter tolerance at angle zero,
    independently of the inflated collision rectangle. Near-body people must
    remain obstacles even when they step into the inflated starting rectangle.
    Radial pruning bounds the work for dense depth clouds.
    """
    if not math.isfinite(delta_yaw) or not math.isfinite(step) or step <= 0.:
        raise ValueError('rotation must be finite and step positive and finite')
    if delta_yaw == 0.:
        return False
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    initial = ((np.abs(points[:, 0]) <= body_half_length + .01)
               & (np.abs(points[:, 1]) <= body_half_width + .01))
    points = points[~initial & (np.sum(points * points, axis=1) <= half_length**2 + half_width**2)]
    if not len(points):
        return False
    angles = np.append(np.arange(0., abs(delta_yaw), step), abs(delta_yaw))
    for angle in np.copysign(angles, delta_yaw):
        c, s = math.cos(angle), math.sin(angle)
        x = points[:, 0] * c + points[:, 1] * s
        y = -points[:, 0] * s + points[:, 1] * c
        if np.any((np.abs(x) <= half_length) & (np.abs(y) <= half_width)):
            return True
    return False


def _inside(points, polygon, margin=0., strict=True):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    polygon = np.asarray(polygon, dtype=float)
    edges = np.roll(polygon, -1, axis=0) - polygon
    normals = np.column_stack((-edges[:, 1], edges[:, 0]))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    distances = (points[:, None, :] - polygon) * normals
    distances = distances.sum(axis=2)
    return np.all(distances > margin if strict else distances >= margin, axis=1)


def points_in_region(points_map, polygon):
    """Count points strictly inside a convex CCW polygon, excluding its boundary."""
    return int(np.count_nonzero(_inside(points_map, polygon)))


@dataclass(frozen=True)
class SpaceRegion:
    start: object
    end: object
    polygon: object
    half_width: float
    wall_margin: float
    radius: float = 0.
    include_hall: bool = False
    tunnel: object = None  # (hall-door centre, car-door centre, door half-width)

    def contains(self, points_map):
        points = np.asarray(points_map, dtype=float).reshape(-1, 2)
        start, end = np.asarray(self.start), np.asarray(self.end)
        axis = end - start
        length = np.linalg.norm(axis)
        axis = axis / length
        offset = points - start
        along = offset @ axis
        across = offset[:, 0] * axis[1] - offset[:, 1] * axis[0]
        corridor = (along >= 0.) & (along <= length) & (np.abs(across) < self.half_width)
        if self.radius:
            corridor |= np.linalg.norm(points - end, axis=1) < self.radius
        allowed = _inside(points, self.polygon, self.wall_margin)
        if self.include_hall:
            # Roundoff after a map rotation must not classify a wall as hall.
            allowed |= ~_inside(points, self.polygon, margin=-1e-9, strict=False)
        if self.tunnel is not None:
            hall, car, door_half_width = self.tunnel
            inward = np.asarray(car) - hall
            depth = np.linalg.norm(inward)
            inward /= depth
            offset = points - hall
            along = offset @ inward
            lateral = np.abs(offset[:, 0] * inward[1] - offset[:, 1] * inward[0])
            in_tunnel = (along >= -1e-9) & (along <= depth + 1e-9)
            # The specified 0.15 m pose error exceeds the default 0.12 m
            # wall inset. Retain at least 0.15 m at jambs so a shifted .55 m
            # jamb cannot enter the default .407 m corridor at lateral .40 m.
            half_width = max(0., min(self.half_width, door_half_width - max(self.wall_margin, .15)))
            corridor &= ~in_tunnel | (lateral < half_width - 1e-9)
        return corridor & allowed


def boarding_region(geometry, margin, wall_margin=.12, approach_from=None):
    """Corridor to the car centre plus turning disc.

    Without ``approach_from`` the corridor starts at the hall door and is
    clipped inside the car. With the robot's (x, y, yaw) it starts at the front
    bumper and also covers the hall and door tunnel, so a person still walking
    out of the car blocks the start (the wait pose is 1.0 m from the door).
    """
    turn = math.hypot(.348, .307) + margin
    if approach_from is None:
        return SpaceRegion(np.mean(geometry.hall_door, axis=0), geometry.car_pose[:2],
                           geometry.car_polygon, .307 + margin, wall_margin, turn)
    x, y, yaw = approach_from
    bumper = (x + .348 * math.cos(yaw), y + .348 * math.sin(yaw))
    hall, car = np.mean(geometry.hall_door, axis=0), np.mean(geometry.car_door, axis=0)
    half_door = min(np.linalg.norm(np.subtract(*geometry.hall_door)),
                    np.linalg.norm(np.subtract(*geometry.car_door))) / 2
    return SpaceRegion(bumper, geometry.car_pose[:2], geometry.car_polygon, .307 + margin,
                       wall_margin, turn, include_hall=True, tunnel=(hall, car, half_door))


def exit_region(geometry, hall_exit_pose, margin, wall_margin=.12):
    """Car to hall corridor, with wall exclusion and a narrower door tunnel.

    Tunnel jamb inset is at least wall_margin and 0.15 m (pose-error allowance).
    The same tunnel clip applies to the boarding region's hall approach.
    """
    hall, car = np.mean(geometry.hall_door, axis=0), np.mean(geometry.car_door, axis=0)
    half_door = min(np.linalg.norm(np.subtract(*geometry.hall_door)),
                    np.linalg.norm(np.subtract(*geometry.car_door))) / 2
    return SpaceRegion(geometry.car_pose[:2], hall_exit_pose[:2], geometry.car_polygon,
                       .307 + margin, wall_margin, include_hall=True, tunnel=(hall, car, half_door))
