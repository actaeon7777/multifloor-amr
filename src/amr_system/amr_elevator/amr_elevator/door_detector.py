"""Elevator door state from a planar LiDAR scan (no ROS).

The RF devices never report the door; the AMR decides from its own scan.
Beams that cross the door opening (away from the jambs) are open when their
range passes the door plane by ``pass_margin``; a closed panel returns at the
plane.  Out-of-range returns count as passing.  The state uses hysteresis and
consecutive scans so a person walking by does not flip it.
"""

from dataclasses import dataclass
import math

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

    def reset(self):
        self.state, self.previous_ratio, self.streak = UNKNOWN, None, (None, 0)

    def measure(self, laser_pose, door, ranges, angle_min, angle_increment,
                range_min=.05, range_max=30.):
        """(open_ratio, valid_beams) of the beams crossing the door segment."""
        x, y, yaw = laser_pose
        passed = valid = 0
        for index, value in enumerate(ranges):
            distance = ray_segment_distance((x, y), yaw + angle_min + index * angle_increment,
                                            door[0], door[1], self.jamb_margin)
            if distance is None or distance > self.max_view_distance:
                continue
            if math.isnan(value) or value < range_min:
                continue
            valid += 1
            if math.isinf(value) or value >= range_max or value > distance + self.pass_margin:
                passed += 1
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
    valid = [r for r in ranges if not math.isnan(r) and r >= range_min]
    if not valid:
        return 0.
    return sum(1 for r in valid if r < near) / len(valid)
