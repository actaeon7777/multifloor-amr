"""Real EKF + installed bringup, synthetic sensors, no CAN or physical joystick."""
import math
import os
import signal
import subprocess
import time

import pytest

if os.environ.get('AMR_RUN_EKF_TESTS') != '1':
    pytest.skip('Opt in with AMR_RUN_EKF_TESTS=1, domain 187 localhost only', allow_module_level=True)

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener


def test_real_filter_uses_velocity_and_gyro_without_pose_or_acceleration(tmp_path):
    assert os.environ['ROS_DOMAIN_ID'] == '187'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rclpy.init()
    node = rclpy.create_node('ekf_test_probe')
    wheel = node.create_publisher(Odometry, '/wheel/odometry', 10)
    imu = node.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
    output = []
    node.create_subscription(Odometry, '/odometry/filtered', output.append, 100)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    log = (tmp_path/'launch.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'amr_stack', 'bringup.launch.py',
        'start_can_bridge:=false', 'start_can_telemetry:=false',
        'start_joy:=false', 'start_lidar:=false',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def run(duration, gyro):
        deadline = time.monotonic()+duration
        next_wheel = next_imu = 0.
        while time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path/'launch.log').read_text()
            now = time.monotonic()
            if now >= next_wheel:
                msg = Odometry()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'odom'
                msg.child_frame_id = 'base_footprint'
                # Contradictory pose/yaw rate must NOT influence selected fusion.
                msg.pose.pose.position.x = 1000.
                msg.pose.pose.position.y = -1000.
                msg.pose.pose.orientation.z = 1.
                msg.twist.twist.linear.x = .2
                msg.twist.twist.angular.z = -1.
                for index in (0,7,14,21,28,35):
                    msg.pose.covariance[index] = .0001
                    msg.twist.covariance[index] = .0025
                wheel.publish(msg)
                next_wheel = now+.02
            if now >= next_imu:
                msg = Imu()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'imu_link'
                msg.orientation.w = 1.
                msg.orientation_covariance[0] = -1.
                msg.angular_velocity.z = gyro
                msg.linear_acceleration.x = 10.
                msg.linear_acceleration.z = 9.81
                for index in (0,4,8):
                    msg.angular_velocity_covariance[index] = .0009
                    msg.linear_acceleration_covariance[index] = .01
                imu.publish(msg)
                next_imu = now+.1
            rclpy.spin_once(node, timeout_sec=.005)

    try:
        run(7., .2)
        assert len(output) > 70, (tmp_path/'launch.log').read_text()
        latest = output[-1]
        assert latest.header.frame_id == 'odom'
        assert latest.child_frame_id == 'base_footprint'
        assert latest.twist.twist.linear.x == pytest.approx(.2, abs=.015)
        assert latest.twist.twist.angular.z == pytest.approx(.2, abs=.015)
        assert latest.twist.twist.linear.y == pytest.approx(0., abs=.005)
        assert 0. < latest.pose.pose.position.x < 3.
        assert abs(latest.pose.pose.position.y) < 3.
        assert abs(latest.pose.pose.position.z) < 1e-6
        assert abs(latest.pose.pose.orientation.x) < 1e-6
        assert abs(latest.pose.pose.orientation.y) < 1e-6
        assert latest.pose.pose.orientation.z > .05
        assert all(math.isfinite(v) for v in (*latest.pose.covariance, *latest.twist.covariance))
        transform = buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        assert abs(transform.transform.translation.x-latest.pose.pose.position.x) < .05
        assert buffer.can_transform('odom', 'imu_link', rclpy.time.Time())
        assert buffer.can_transform('odom', 'laser', rclpy.time.Time())
        assert node.count_publishers('/odometry/filtered') == 1
        assert not any(name == 'cmd_vel_to_can' for name,_ in node.get_node_names_and_namespaces())
        run(3., -.2)
        assert output[-1].twist.twist.angular.z == pytest.approx(-.2, abs=.015)
        assert output[-1].twist.twist.linear.x == pytest.approx(.2, abs=.015)
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
