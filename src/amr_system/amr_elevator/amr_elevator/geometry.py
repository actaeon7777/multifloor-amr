"""Planar pose helpers shared by the elevator nodes (no ROS)."""

import math


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    """a ∘ b for (x, y, yaw) poses."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


def relative(a, b):
    """Pose b expressed in frame a."""
    return compose(inverse(a), b)


def point_in_polygon(x, y, polygon):
    inside = False
    points = list(polygon)
    for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def depth_beyond(point, line, inward_yaw):
    """Signed distance of ``point`` past ``line`` along ``inward_yaw`` (positive = car side)."""
    mid = ((line[0][0] + line[1][0]) / 2, (line[0][1] + line[1][1]) / 2)
    return (point[0] - mid[0]) * math.cos(inward_yaw) + (point[1] - mid[1]) * math.sin(inward_yaw)


def ray_segment_distance(origin, angle, a, b, end_margin=0.):
    """Distance along the ray to segment a-b, ignoring ``end_margin`` at both ends; None if missed."""
    dx, dy = math.cos(angle), math.sin(angle)
    sx, sy = b[0] - a[0], b[1] - a[1]
    denominator = dx * sy - dy * sx
    if abs(denominator) < 1e-12:
        return None
    ox, oy = a[0] - origin[0], a[1] - origin[1]
    t = (ox * sy - oy * sx) / denominator
    u = (ox * dy - oy * dx) / denominator
    length = math.hypot(sx, sy)
    margin = end_margin / length if length > 0 else 0.
    if t <= 0 or not margin <= u <= 1 - margin:
        return None
    return t
