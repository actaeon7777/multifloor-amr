"""Slow straight-line maneuvers through the elevator door (no ROS).

Nav2 cannot plan through the 1.1 m opening with the shared costmap settings, so
boarding and alighting drive a straight line on the odometry-carried pose:
turn toward the target, drive with heading correction and a braking profile,
then turn to the final heading.  ``reverse`` backs along the line keeping the
current heading (used to retreat from a closing door).
"""

from dataclasses import dataclass
import math

import numpy as np

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

    def pending_turn(self, pose):
        """Signed remaining in-place rotation, or None outside ALIGN/TURN.

        ALIGN can reach the target and transition straight to the final turn
        within command(), so expose that turn before issuing any angular speed.
        """
        if self.phase not in (ALIGN, TURN):
            return None
        if self.phase == TURN or self.remaining(pose) <= self.limits.position_tolerance:
            return None if self.final_yaw is None else wrap(self.final_yaw - pose[2])
        bearing = math.atan2(self.target[1] - pose[1], self.target[0] - pose[0])
        return wrap(bearing + (math.pi if self.reverse else 0.) - pose[2])

    def command(self, pose, dt, clearance=math.inf, turn_blocked=False):
        """(linear, angular) for the current pose; ``clearance`` is free space ahead of travel."""
        lim = self.limits
        self.blocked = False
        distance = self.remaining(pose)
        bearing = math.atan2(self.target[1] - pose[1], self.target[0] - pose[0])
        heading = wrap(bearing + math.pi) if self.reverse else bearing
        if self.phase in (ALIGN, DRIVE) and distance <= lim.position_tolerance:
            was_driving = self.phase == DRIVE
            self.phase = TURN if self.final_yaw is not None else DONE
            self.speed = 0.
            if was_driving:
                return 0., 0.  # final turn must also wait for its sweep check
        if self.phase == ALIGN:
            error = wrap(heading - pose[2])
            if abs(error) <= lim.heading_tolerance:
                self.phase = DRIVE
            else:
                if turn_blocked:
                    self.blocked = True
                    self.speed = 0.
                    return 0., 0.
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
                return 0., 0.  # next tick checks the new in-place sweep first
            creep = .05**2 / (2 * lim.decel)
            self.blocked = clearance < lim.min_clearance + creep
            obstacle_limit = math.sqrt(2 * lim.decel * max(0., clearance - lim.min_clearance))
            limit = 0. if self.blocked else min(lim.max_speed, obstacle_limit,
                                               math.sqrt(2 * lim.decel * max(0., along)))
            if self.speed < limit:
                self.speed = min(limit, self.speed + lim.accel * dt)
            else:
                self.speed = max(limit, self.speed - 2 * lim.decel * dt) if not self.blocked else 0.
            # A newly seen obstacle may require a lower speed than the ramp permits.
            self.speed = min(self.speed, obstacle_limit)
            if self.speed == 0.:
                return 0., 0.  # do not turn in place using DRIVE heading correction
            # Limit curvature at low speed instead of almost spinning in DRIVE.
            turn_limit = min(lim.max_turn, max(.1, self.speed / .2))
            turn = max(-turn_limit, min(turn_limit, lim.heading_gain * error))
            return (-self.speed if self.reverse else self.speed), turn
        if self.phase == TURN:
            error = wrap(self.final_yaw - pose[2])
            if abs(error) <= lim.yaw_tolerance:
                self.phase = DONE
            else:
                if turn_blocked:
                    self.blocked = True
                    self.speed = 0.
                    return 0., 0.
                return 0., self._turn(error)
        return 0., 0.

    def _turn(self, error):
        lim = self.limits
        rate = max(.15, min(lim.max_turn, 1.2 * abs(error)))
        return math.copysign(rate, error)


def travel_clearance(ranges, angle_min, angle_increment, reverse, half_length=.32, half_width=.3,
                     laser_pose=(0., 0., 0.)):
    """Free distance ahead of the bumper along the travel direction.

    The scan is in the laser frame; ``laser_pose`` is the laser's (x, y, yaw) in
    the base frame (the real S2 is mounted turned 180 deg, yaw = pi).
    """
    return travel_clearances(ranges, angle_min, angle_increment, half_length, half_width,
                             laser_pose)[bool(reverse)]


def travel_clearances(ranges, angle_min, angle_increment, half_length=.32, half_width=.3,
                      laser_pose=(0., 0., 0.)):
    """(forward, reverse) bumper clearances from a single scan projection."""
    lx, ly, lyaw = laser_pose
    values = np.asarray(ranges, dtype=float)
    indices = np.flatnonzero(np.isfinite(values) & (values > 0.))
    values = values[indices]
    angles = lyaw + angle_min + indices * angle_increment
    x, y = lx + values * np.cos(angles), ly + values * np.sin(angles)
    # Preserve scalar inclusion at the bumper centreline and body-width edges.
    boundary = (np.isclose(x, 0., rtol=0., atol=1e-15)
                | np.isclose(np.abs(y), half_width, rtol=1e-12, atol=1e-15))
    for index in np.flatnonzero(boundary):
        angle = lyaw + angle_min + int(indices[index]) * angle_increment
        x[index] = lx + values[index] * math.cos(angle)
        y[index] = ly + values[index] * math.sin(angle)
    within_width = np.abs(y) <= half_width
    result = []
    for direction in (1., -1.):
        along = direction * x
        qualifies = (along > 0.) & within_width
        best = np.min(along[qualifies], initial=np.inf)
        if not math.isfinite(best):
            result.append(math.inf)
            continue
        # Re-evaluate only the minimum (including ties) with libm, preserving
        # the scalar return value even at the maneuver's strict stop threshold.
        candidates = np.flatnonzero(qualifies & np.isclose(along, best, rtol=1e-12, atol=1e-15))
        result.append(min(
            direction * (lx + float(values[i]) * math.cos(
                lyaw + angle_min + int(indices[i]) * angle_increment)) - half_length
            for i in candidates))
    return tuple(result)
