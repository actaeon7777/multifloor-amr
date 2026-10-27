"""travel_clearance works on the scan in the laser frame (no ROS)."""
import math

from amr_elevator.maneuver import travel_clearance

LASER_TURNED = (0., 0., math.pi)   # real S2: laser +X points backwards


def scan_with_wall(distance, bearing_deg, n=360):
    """Scan in the laser frame with a wall perpendicular to the given bearing."""
    inc = 2 * math.pi / n
    ranges = [math.inf] * n
    for i in range(n):
        a = -math.pi + i * inc
        d = abs(math.atan2(math.sin(a - math.radians(bearing_deg)), math.cos(a - math.radians(bearing_deg))))
        if d < math.radians(15):
            ranges[i] = distance / math.cos(d)
    return ranges, -math.pi, inc


def test_wall_in_front_of_the_robot_limits_forward_travel_only():
    # A wall 1.0 m ahead of the robot is at laser bearing 180 deg on the turned S2.
    ranges, amin, inc = scan_with_wall(1.0, 180.)
    forward = travel_clearance(ranges, amin, inc, False, .32, laser_pose=LASER_TURNED)
    backward = travel_clearance(ranges, amin, inc, True, .32, laser_pose=LASER_TURNED)
    assert forward == 0.68 or abs(forward - .68) < 1e-6
    assert backward == math.inf


def test_wall_behind_the_robot_limits_reverse_only():
    ranges, amin, inc = scan_with_wall(.5, 0.)
    assert travel_clearance(ranges, amin, inc, False, .32, laser_pose=LASER_TURNED) == math.inf
    assert abs(travel_clearance(ranges, amin, inc, True, .32, laser_pose=LASER_TURNED) - .18) < 1e-6


def test_untransformed_scan_would_swap_directions():
    # Documents the defect the laser pose fixes: without it the same scan reads reversed.
    ranges, amin, inc = scan_with_wall(1.0, 180.)
    assert travel_clearance(ranges, amin, inc, False, .32) == math.inf
    assert abs(travel_clearance(ranges, amin, inc, True, .32) - .68) < 1e-6


def test_laser_offset_is_added_to_the_range():
    ranges, amin, inc = scan_with_wall(1.0, 0.)
    forward = travel_clearance(ranges, amin, inc, False, .32, laser_pose=(.1, 0., 0.))
    assert abs(forward - (1.1 - .32)) < 1e-6


def test_points_outside_the_body_width_are_ignored():
    n, inc = 360, 2 * math.pi / 360
    ranges = [math.inf] * n
    ranges[n // 2 + 60] = 1.0     # 60 deg off axis: y = 0.87 m, beyond half_width
    assert travel_clearance(ranges, -math.pi, inc, False, .32, laser_pose=LASER_TURNED) == math.inf


from amr_elevator.maneuver import ALIGN, DRIVE, TURN, DONE, LineManeuver
import pytest


def test_drive_brakes_before_minimum_clearance():
    maneuver = LineManeuver((10., 0.))
    maneuver.phase, maneuver.speed = DRIVE, .35
    speed, _ = maneuver.command((0., 0., 0.), .05, clearance=.25)
    assert 0. < speed <= math.sqrt(2 * .3 * (.25 - .15))
    assert not maneuver.blocked
    speed, _ = maneuver.command((0., 0., 0.), .05, clearance=.15)
    assert speed == 0. and maneuver.blocked  # a sub-0.05 m/s obstacle limit is a hold
    assert maneuver.command((0., 0., 0.), .05, clearance=.14)[0] == 0.
    assert maneuver.blocked
    assert maneuver.command((0., 0., 0.), .05, clearance=math.inf)[0] > 0.
    assert not maneuver.blocked


@pytest.mark.parametrize('phase', [ALIGN, TURN])
def test_turn_holds_and_resumes_without_stale_blocked_flag(phase):
    maneuver = LineManeuver((0., 2.), final_yaw=math.pi/2)
    maneuver.phase = phase
    assert maneuver.pending_turn((0., 0., 0.)) == pytest.approx(math.pi/2)
    assert maneuver.command((0., 0., 0.), .05, turn_blocked=True) == (0., 0.)
    assert maneuver.blocked
    assert maneuver.command((0., 0., 0.), .05, turn_blocked=False)[1] > 0.
    assert not maneuver.blocked
    maneuver.phase = DONE
    maneuver.blocked = True
    assert maneuver.pending_turn((0., 0., 0.)) is None
    assert maneuver.command((0., 0., 0.), .05, turn_blocked=True) == (0., 0.)
    assert not maneuver.blocked


def test_pending_turn_handles_align_target_arrival_and_reverse():
    maneuver = LineManeuver((0., 0.), final_yaw=-math.pi/2)
    assert maneuver.pending_turn((0., 0., 0.)) == pytest.approx(-math.pi/2)
    assert maneuver.command((0., 0., 0.), .05, turn_blocked=True) == (0., 0.)
    assert maneuver.phase == TURN and maneuver.blocked
    reverse = LineManeuver((-2., 0.), reverse=True)
    assert reverse.pending_turn((0., 0., 0.)) == pytest.approx(0.)


def test_drive_realign_waits_for_next_sweep_check():
    maneuver = LineManeuver((0., 2.))
    maneuver.phase = DRIVE
    assert maneuver.pending_turn((0., 0., .5)) is None
    assert maneuver.command((0., 0., .5), .05) == (0., 0.)
    assert maneuver.phase == ALIGN
    assert maneuver.pending_turn((0., 0., .5)) > .6
    assert maneuver.command((0., 0., .5), .05, turn_blocked=True) == (0., 0.)
    assert maneuver.blocked


def test_drive_arrival_waits_before_final_turn_and_stopped_drive_does_not_rotate():
    maneuver = LineManeuver((0., 0.), final_yaw=math.pi)
    maneuver.phase = DRIVE
    assert maneuver.command((0., 0., 0.), .05) == (0., 0.)
    assert maneuver.phase == TURN
    assert maneuver.command((0., 0., 0.), .05, turn_blocked=True) == (0., 0.)
    assert maneuver.blocked
    maneuver = LineManeuver((2., .1))
    maneuver.phase = DRIVE
    assert maneuver.command((0., 0., 0.), .05, clearance=.14) == (0., 0.)
    assert maneuver.blocked


@pytest.mark.parametrize('reverse', [False, True])
def test_obstacle_crawl_band_is_blocked_and_releases_above_threshold(reverse):
    maneuver = LineManeuver((-2. if reverse else 2., 0.), reverse=reverse)
    maneuver.phase, maneuver.speed = DRIVE, .35
    creep = .05**2 / (2 * maneuver.limits.decel)
    for clearance in (.15, .15 + creep/2, .15 + creep - 1e-8):
        assert maneuver.command((0., 0., 0.), .05, clearance=clearance) == (0., 0.)
        assert maneuver.blocked
    speed, _ = maneuver.command((0., 0., 0.), .2, clearance=.15 + creep + 1e-8)
    assert abs(speed) >= .05 and not maneuver.blocked
    assert (speed < 0.) == reverse


@pytest.mark.parametrize('error', [-.5, .5])
@pytest.mark.parametrize('speed', [.02, .06, .35])
def test_drive_heading_correction_scales_with_speed(error, speed):
    maneuver = LineManeuver((2 * math.cos(error), 2 * math.sin(error)))
    maneuver.phase, maneuver.speed = DRIVE, speed
    linear, angular = maneuver.command((0., 0., 0.), 0.)
    assert linear == pytest.approx(speed)
    assert abs(angular) == pytest.approx(min(.5, 1.5 * abs(error), max(.1, speed/.2)))
    assert (angular < 0.) == (error < 0.)
