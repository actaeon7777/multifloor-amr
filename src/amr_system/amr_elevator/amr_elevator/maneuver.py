"""Slow straight-line maneuvers through the elevator door (no ROS).

Nav2 cannot plan through the 1.1 m opening with the shared costmap settings, so
boarding and alighting drive a straight line on the odometry-carried pose:
turn toward the target, drive with heading correction and a braking profile,
then turn to the final heading.  ``reverse`` backs along the line keeping the
current heading (used to retreat from a closing door).
"""

from dataclasses import dataclass
import math

from amr_elevator.geometry import wrap

ALIGN, DRIVE, TURN, DONE = 'ALIGN', 'DRIVE', 'TURN', 'DONE'


@dataclass
class ManeuverLimits:
    max_speed: float = .35
    max_turn: float = .5
    accel: float = .4
    decel: float = .3
    position_tolerance: float = .05
    yaw_tolerance: float = .05
    heading_tolerance: float = .12
    heading_gain: float = 1.5
    min_clearance: float = .15


class LineManeuver:
    def __init__(self, target, final_yaw=None, reverse=False, limits=None):
        self.target = target[:2]
        self.final_yaw = final_yaw
        self.reverse = reverse
        self.limits = limits or ManeuverLimits()
        self.phase = ALIGN
        self.speed = 0.
        self.blocked = False

    def remaining(self, pose):
        return math.hypot(self.target[0] - pose[0], self.target[1] - pose[1])

    def command(self, pose, dt, clearance=math.inf):
        """(linear, angular) for the current pose; ``clearance`` is free space ahead of travel."""
        lim = self.limits
        distance = self.remaining(pose)
        bearing = math.atan2(self.target[1] - pose[1], self.target[0] - pose[0])
        heading = wrap(bearing + math.pi) if self.reverse else bearing
        if self.phase in (ALIGN, DRIVE) and distance <= lim.position_tolerance:
            self.phase = TURN if self.final_yaw is not None else DONE
            self.speed = 0.
        if self.phase == ALIGN:
            error = wrap(heading - pose[2])
            if abs(error) <= lim.heading_tolerance:
                self.phase = DRIVE
            else:
                return 0., self._turn(error)
        if self.phase == DRIVE:
            # Project onto the travel direction: past the target means stop, not turn back.
            along = (self.target[0] - pose[0]) * math.cos(pose[2]) + (self.target[1] - pose[1]) * math.sin(pose[2])
            if self.reverse:
                along = -along
            if along <= lim.position_tolerance:
                self.phase = TURN if self.final_yaw is not None else DONE
                self.speed = 0.
                return 0., 0.
            error = wrap(heading - pose[2])
            if abs(error) > .6:  # badly off the line: re-align in place
                self.phase = ALIGN
                self.speed = 0.
                return 0., self._turn(error)
            self.blocked = clearance < lim.min_clearance
            limit = 0. if self.blocked else min(lim.max_speed, math.sqrt(2 * lim.decel * max(0., along)))
            if self.speed < limit:
                self.speed = min(limit, self.speed + lim.accel * dt)
            else:
                self.speed = max(limit, self.speed - 2 * lim.decel * dt) if not self.blocked else 0.
            turn = max(-lim.max_turn, min(lim.max_turn, lim.heading_gain * error))
            return (-self.speed if self.reverse else self.speed), turn
        if self.phase == TURN:
            error = wrap(self.final_yaw - pose[2])
            if abs(error) <= lim.yaw_tolerance:
                self.phase = DONE
            else:
                return 0., self._turn(error)
        return 0., 0.

    def _turn(self, error):
        lim = self.limits
        rate = max(.15, min(lim.max_turn, 1.2 * abs(error)))
        return math.copysign(rate, error)


def travel_clearance(ranges, angle_min, angle_increment, reverse, half_length=.32, half_width=.3):
    """Free distance ahead of the bumper along the travel direction from a scan in the base frame."""
    best = math.inf
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value <= 0.:
            continue
        angle = angle_min + index * angle_increment
        x, y = value * math.cos(angle), value * math.sin(angle)
        if reverse:
            x = -x
        if x > 0 and abs(y) <= half_width:
            best = min(best, x - half_length)
    return best
