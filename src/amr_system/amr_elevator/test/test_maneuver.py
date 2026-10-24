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
