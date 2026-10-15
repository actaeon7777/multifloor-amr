"""Real tf2 acquisition-time checks, without controllers or motion publishers."""
import importlib.util
import math
import os
from pathlib import Path

import numpy as np
import pytest

if os.environ.get('AMR_RUN_STARTUP_TESTS') != '1':
    pytest.skip('Opt in with AMR_RUN_STARTUP_TESTS=1, isolated domain 194', allow_module_level=True)

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.time import Time
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan

from amr_stack.navigation_startup import NavigationStartup

ROOT = Path(__file__).resolve().parents[4]
spec = importlib.util.spec_from_file_location(
    'rolling_geometry_fixture', ROOT / 'scripts/benchmark_rolling_scan_quality.py')
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


def transform(parent, child, pose, stamp):
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id = parent, child
    msg.header.stamp = Time(nanoseconds=stamp).to_msg()
    msg.transform.translation.x, msg.transform.translation.y = map(float, pose[:2])
    msg.transform.rotation.z = math.sin(float(pose[2])/2)
    msg.transform.rotation.w = math.cos(float(pose[2])/2)
    return msg


@pytest.fixture
def turning_scan():
    # Truth generates only synthetic measurements and expected test values.
    # The node receives a fixed global prior plus local odometry, never truth.
    assert os.environ.get('ROS_DOMAIN_ID') == '194'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init(args=[])
    # Match the production AMCL/verification range explicitly in this fixture.
    node = NavigationStartup(parameter_overrides=[
        Parameter('tracking_quality_max_range', value=30.0)])
    quality, polygons, _ = geometry.load_floor4()
    span = 1/geometry.HZ
    start = node.get_clock().now().nanoseconds - round(.25e9)
    poses = geometry.trajectory((25., 10., 0.), .4, .6, geometry.TIMES)
    scan = LaserScan()
    scan.header.frame_id = 'laser'
    scan.header.stamp = Time(nanoseconds=start).to_msg()
    scan.angle_min, scan.angle_max = -math.pi, math.pi
    scan.angle_increment = geometry.INCREMENT
    scan.range_min, scan.range_max = geometry.RANGE_MIN, geometry.RANGE_MAX
    scan.time_increment = span/(geometry.POINTS-1)
    scan.scan_time = span
    scan.ranges = geometry.raycast(polygons, poses).tolist()
    node.map_quality = quality
    node.prior_verified = node.initialized = True
    node.source, node.state = 'auto', 'READY'
    node.attempt_stamp = start - 1
    node.receive('scan', scan)
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id, pose.header.stamp = 'map', scan.header.stamp
    pose.pose.pose.position.x, pose.pose.pose.position.y = 25., 10.
    pose.pose.pose.orientation.w = 1.
    pose.pose.covariance[0] = pose.pose.covariance[7] = pose.pose.covariance[35] = .01
    node.receive('pose', pose)
    node.tf_buffer.set_transform_static(
        transform('base_footprint', 'laser', (0., 0., math.pi), start), 'synthetic_sensor_mount')

    def fill_odometry(complete=True):
        # 200 Hz local TF; tf2 must interpolate actual ray timestamps. Include
        # a final sample beyond the final ray's floating-point ROS duration.
        times = np.linspace(0., span+.001, 25)
        local = geometry.trajectory((0., 0., 0.), .4, .6, times)
        for elapsed, row in zip(times, local):
            if not complete and elapsed > span/2:
                break
            node.tf_buffer.set_transform(
                transform('odom', 'base_footprint', (row[0], row[1], row[2]-math.pi),
                          start + round(elapsed*1e9)), 'synthetic_wheel_odom')

    def fill_map(correction_yaw=0., end_yaw=0.):
        node.tf_buffer.set_transform(
            transform('map', 'odom', (25., 10., correction_yaw), start), 'synthetic_amcl_prior')
        node.tf_buffer.set_transform(
            transform('map', 'odom', (25., 10., end_yaw), start + round((span+.001)*1e9)),
            'synthetic_amcl_prior')
        pose.pose.pose.orientation.z = math.sin(correction_yaw/2)
        pose.pose.pose.orientation.w = math.cos(correction_yaw/2)

    fill_odometry()
    fill_map()
    try:
        yield node, poses, fill_odometry, fill_map
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_exact_odometry_ray_times_recover_true_turning_scan_without_weakening_gate(turning_scan):
    node, truth, _, _ = turning_scan
    raw = node.map_quality.score(**node.scan_args(), laser_pose=truth[0])
    assert raw.match_ratio < node.cfg['min_match_ratio']
    evidence, problem = node.pose_evidence()
    assert problem is None and evidence is not None
    assert node.metrics['scan_motion_compensated']
    assert node.metrics['match_ratio'] > .98
    assert node.metrics['scan_motion_yaw'] == pytest.approx(.6/geometry.HZ, abs=.001)
    assert node.metrics['scan_motion_translation'] > .04


def test_amcl_correction_is_held_fixed_within_scan(turning_scan):
    node, truth, fill_odometry, fill_map = turning_scan
    # A later global correction is not measured intra-scan robot motion.
    node.tf_buffer.clear()
    fill_odometry()
    fill_map(end_yaw=.5)
    indices, _, _ = node.map_quality.sampled_rays(**node.scan_args())
    poses = node.tracking_ray_poses(truth[0])
    assert np.max(np.abs(poses[:, :2] - truth[indices, :2])) < .001
    angle_error = (poses[:, 2] - truth[indices, 2] + math.pi) % (2*math.pi) - math.pi
    assert np.max(np.abs(angle_error)) < .001
    assert node.pose_evidence()[1] is None


def test_motion_compensation_does_not_accept_wrong_global_heading(turning_scan):
    node, _, fill_odometry, fill_map = turning_scan
    node.tf_buffer.clear()
    fill_odometry()
    fill_map(correction_yaw=.2, end_yaw=.2)
    _, problem = node.pose_evidence()
    assert problem is not None and node.evidence_issue == 'scan_alignment'
    assert node.metrics['match_ratio'] < node.cfg['min_match_ratio']
    assert '일치율' in problem and '지도 내 관측' in problem and '벽 관통' in problem


def test_missing_final_beam_transform_is_hard_failure_not_raw_scan_fallback(turning_scan):
    node, _, fill_odometry, fill_map = turning_scan
    # Keep the sensor mount static while removing all dynamic TF history.
    node.tf_buffer.clear()
    fill_odometry(complete=False)
    fill_map()
    assert node.tf_buffer.can_transform('map', 'laser', Time.from_msg(node.scan.header.stamp))
    evidence, problem = node.pose_evidence()
    assert evidence is None and problem is not None and node.evidence_issue == 'hard'
    assert not node.metrics['scan_motion_compensated']


@pytest.mark.parametrize('increment', [-.001, float('nan'), float('inf'), .01])
def test_invalid_acquisition_timing_cannot_be_motion_compensated(turning_scan, increment):
    node, truth, _, _ = turning_scan
    node.scan.time_increment = increment
    assert not node.valid_scan(node.scan)
    with pytest.raises(ValueError):
        node.tracking_ray_poses(truth[0])


def test_instantaneous_scan_keeps_legacy_geometry(turning_scan):
    node, truth, _, _ = turning_scan
    node.scan.time_increment = 0.
    assert node.tracking_ray_poses(truth[0]) is None
    # Removing valid timing must not fabricate a correction that passes this
    # known distorted scan. Drivers without timing retain static scoring.
    assert node.pose_evidence()[1] is not None
    assert not node.metrics['scan_motion_compensated']


def test_stationary_initialization_does_not_use_tracking_deskew(turning_scan):
    node, truth, _, _ = turning_scan
    node.prior_verified = False
    assert node.tracking_ray_poses(truth[0]) is None
