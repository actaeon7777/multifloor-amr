"""Elevator door state from a planar LiDAR scan (no ROS).

The RF devices never report the door; the AMR decides from its own scan.
Beams that cross the door opening (away from the jambs) are open when their
range passes the door plane by ``pass_margin``; a closed panel returns at the
plane.  Out-of-range returns count as passing.  The state uses hysteresis and
consecutive scans so a person walking by does not flip it.
"""

from dataclasses import dataclass
import math

import numpy as np

from amr_elevator.geometry import ray_segment_distance

UNKNOWN, CLOSED, OPENING, OPEN, CLOSING = 0, 1, 2, 3, 4
NAMES = {UNKNOWN: 'UNKNOWN', CLOSED: 'CLOSED', OPENING: 'OPENING', OPEN: 'OPEN', CLOSING: 'CLOSING'}


@dataclass(frozen=True)
class DoorReading:
    state: int
    open_ratio: float
    valid_beams: int


class DoorDetector:
    def __init__(self, open_ratio=.7, closed_ratio=.2, confirm_scans=2, min_beams=10,
                 pass_margin=.3, jamb_margin=.08, max_view_distance=4.):
        if not 0 <= closed_ratio < open_ratio <= 1 or confirm_scans < 1 or min_beams < 1:
            raise ValueError('invalid door detector thresholds')
        self.open_ratio, self.closed_ratio = open_ratio, closed_ratio
        self.confirm_scans, self.min_beams = confirm_scans, min_beams
        self.pass_margin, self.jamb_margin = pass_margin, jamb_margin
        self.max_view_distance = max_view_distance
        self.state = UNKNOWN
        self.previous_ratio = None
        self.streak = (None, 0)
        self._beam_key = None
        self._beam_offsets = None
        self._beam_angles = None

    def reset(self):
        self.state, self.previous_ratio, self.streak = UNKNOWN, None, (None, 0)

    def measure(self, laser_pose, door, ranges, angle_min, angle_increment,
                range_min=.05, range_max=30.):
        """(open_ratio, valid_beams) of the beams crossing the door segment."""
        x, y, yaw = laser_pose
        a, b = door
        sx, sy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(sx, sy)
        if length == 0.:
            return 0., 0
        ox, oy = a[0] - x, a[1] - y
        along = max(0., min(1., -(ox * sx + oy * sy) / (length * length)))
        nearest = math.hypot(ox + along * sx, oy + along * sy)
        if nearest > self.max_view_distance:
            return 0., 0

        key = (angle_min, angle_increment, len(ranges))
        if key != self._beam_key:
            self._beam_offsets = np.arange(len(ranges), dtype=float) * angle_increment
            self._beam_angles = angle_min + self._beam_offsets
            self._beam_key = key
        # Keep the scalar loop's addition order, including for nonzero laser yaw.
        angles = self._beam_angles if yaw == 0. else (yaw + angle_min) + self._beam_offsets
        dx, dy = np.cos(angles), np.sin(angles)
        denominator = dx * sy - dy * sx
        intersects = np.abs(denominator) >= 1e-12
        distance = np.full(len(ranges), np.inf)
        u = np.full(len(ranges), np.nan)
        np.divide(ox * sy - oy * sx, denominator, out=distance, where=intersects)
        np.divide(ox * dy - oy * dx, denominator, out=u, where=intersects)
        margin = self.jamb_margin / length
        values = np.asarray(ranges, dtype=float)
        valid_mask = (intersects & (distance > 0.) & (u >= margin) & (u <= 1 - margin)
                      & (distance <= self.max_view_distance)
                      & ~np.isnan(values) & (values >= range_min))
        passed_mask = valid_mask & (np.isinf(values) | (values >= range_max)
                                    | (values > distance + self.pass_margin))

        # NumPy/libm trig may differ by an ulp. Resolve boundary beams with the
        # scalar predicate so inclusive jambs and strict pass thresholds stay exact.
        boundary = (np.isclose(np.abs(denominator), 1e-12, rtol=1e-12, atol=1e-15)
                    | np.isclose(u, margin, rtol=1e-12, atol=1e-15)
                    | np.isclose(u, 1 - margin, rtol=1e-12, atol=1e-15)
                    | np.isclose(distance, self.max_view_distance, rtol=1e-12, atol=1e-15)
                    | (valid_mask & np.isfinite(values)
                       & np.isclose(values, distance + self.pass_margin, rtol=1e-12, atol=1e-15)))
        for index in np.flatnonzero(boundary):
            value = values[index]
            scalar = ray_segment_distance((x, y), yaw + angle_min + int(index) * angle_increment,
                                          a, b, self.jamb_margin)
            valid_mask[index] = (scalar is not None and scalar <= self.max_view_distance
                                 and not math.isnan(value) and value >= range_min)
            passed_mask[index] = valid_mask[index] and (
                math.isinf(value) or value >= range_max or value > scalar + self.pass_margin)
        valid = int(np.count_nonzero(valid_mask))
        passed = int(np.count_nonzero(passed_mask))
        return (passed / valid if valid else 0.), valid

    def update(self, laser_pose, door, ranges, angle_min, angle_increment,
               range_min=.05, range_max=30.):
        ratio, valid = self.measure(laser_pose, door, ranges, angle_min, angle_increment,
                                    range_min, range_max)
        if valid < self.min_beams:
            self.reset()
            return DoorReading(UNKNOWN, ratio, valid)
        if ratio >= self.open_ratio:
            target = OPEN
        elif ratio <= self.closed_ratio:
            target = CLOSED
        elif self.previous_ratio is not None and ratio > self.previous_ratio + .02:
            target = OPENING
        elif self.previous_ratio is not None and ratio < self.previous_ratio - .02:
            target = CLOSING
        else:
            target = self.state if self.state in (OPENING, CLOSING) else None
        self.previous_ratio = ratio
        if target is None:
            return DoorReading(self.state, ratio, valid)
        count = self.streak[1] + 1 if self.streak[0] == target else 1
        self.streak = (target, count)
        if target in (OPENING, CLOSING) or count >= self.confirm_scans:
            self.state = target
        return DoorReading(self.state, ratio, valid)


def enclosure_ratio(ranges, range_min=.05, near=3.):
    """Share of valid beams closer than ``near`` metres (≈1 inside a car)."""
    values = np.asarray(ranges, dtype=float)
    valid = ~np.isnan(values) & (values >= range_min)
    count = np.count_nonzero(valid)
    if not count:
        return 0.
    return int(np.count_nonzero(valid & (values < near))) / int(count)
