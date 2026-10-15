"""ROS publishers driven by measured PhysX state; never owns TF."""
import heapq
import json
import math
import os
import time

import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from amr_can_bridge.telemetry import WheelOdometry
from .model import CommandTransport, PhysXWheelAngle


def stamp(seconds):
    ns = round(seconds * 1_000_000_000)
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def rotation(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class SimBridge(Node):
    def __init__(self, physics, model, joint_names, seed=0):
        if os.environ.get('ROS_DOMAIN_ID') != '107':
            raise RuntimeError('The simulated actuator is restricted to ROS_DOMAIN_ID=107')
        super().__init__('amr_sim_bridge')
        self.model, self.joint_names = model, joint_names
        self.sim_time = 0.
        self.pending, self.sequence = [], 0
        self.transport = CommandTransport(model['command_period_s'], model['command_timeout_s'],
                                          model['command_delay_s'])
        geometry = physics['kinematics']
        # PhysX joint axes both point +Y: forward rotation has the same sign.
        # CAN's inverted left encoder has already been removed by this adapter.
        self.odometry = WheelOdometry(diameter=2*geometry['wheel_radius_m'],
                                      track=geometry['wheel_distance_m'], left_sign=1., right_sign=1.)
        self.wheel_angles = {side: PhysXWheelAngle(self.odometry.max_rev_s, self.odometry.timeout)
                             for side in ('left', 'right')}
        self.rng = np.random.default_rng(seed)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.command, 1)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.wheel_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', sensor_qos)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self.truth_pub = self.create_publisher(Odometry, '/sim/ground_truth', 10)
        self.status_pub = self.create_publisher(String, '/sim/status', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.counts = {'wheel': 0, 'imu': 0, 'scan': 0, 'truth': 0}
        self.contacts = 0
        self.started = time.monotonic()
        self.last_status = 0.
        self.lidar_backend = 'initializing'
        self.render_loading = 'initializing'
        self.status(0.)

    def command(self, msg):
        if any(v != 0. for v in (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)):
            self.transport.reset()
            return
        self.transport.receive(msg.linear.x, msg.angular.z, self.sim_time, time.monotonic())

    def delayed(self, publisher, message):
        self.sequence += 1
        heapq.heappush(self.pending, (self.sim_time + self.model['sensor_delay_s'],
                                      self.sequence, publisher, message))

    def tick(self, seconds):
        self.sim_time = seconds
        self.clock_pub.publish(Clock(clock=stamp(seconds)))
        while self.pending and self.pending[0][0] <= seconds:
            _, _, publisher, message = heapq.heappop(self.pending)
            publisher.publish(message)
        if seconds - self.last_status >= 1.:
            self.status(seconds)

    def status(self, seconds):
        self.last_status = seconds
        self.status_pub.publish(String(data=json.dumps({
            'lidar_backend': self.lidar_backend,
            'render_loading': self.render_loading,
            'state': 'running', 'calibration_status': self.model['calibration_status'],
            'sim_time': seconds, 'real_time_factor': seconds/max(.001, time.monotonic()-self.started),
            'sensor_counts': self.counts, 'sensor_model': self.model['model_version'],
            'obstacle_contact_events': self.contacts,
            'wheel_angle_wraps': sum(angle.wraps for angle in self.wheel_angles.values()),
            'wheel_odometry_rebases': self.odometry.rebases,
            'wheel_odometry_dropped': self.odometry.dropped,
        })))

    def wheels(self, positions, velocities):
        msg = JointState()
        msg.header.stamp = stamp(self.sim_time)
        msg.header.frame_id = 'base_footprint'
        msg.name = self.joint_names
        msg.position, msg.velocity = list(map(float, positions)), list(map(float, velocities))
        self.delayed(self.joint_pub, msg)
        sample = None
        for side in ('left', 'right'):
            index = self.joint_names.index(side + '_drive_wheel_joint')
            position = self.wheel_angles[side].update(float(positions[index]), self.sim_time)
            sample = self.odometry.update(side, position/(2*math.pi),
                                           float(velocities[index])/(2*math.pi), self.sim_time)
        if sample is None:
            return
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp(sample.stamp), 'odom', 'base_footprint'
        msg.pose.pose.position.x, msg.pose.pose.position.y = sample.x, sample.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(sample.yaw/2), math.cos(sample.yaw/2)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = sample.v, sample.w
        msg.pose.covariance = [0.] * 36
        msg.twist.covariance = [0.] * 36
        position_var = .01 + .02*self.odometry.travel + self.odometry.rebases
        yaw_var = .01 + .02*(self.odometry.travel+self.odometry.rotation) + self.odometry.rebases
        for i, value in zip((0, 7, 14, 21, 28, 35), (position_var, position_var, 1e6, 1e6, 1e6, yaw_var)):
            msg.pose.covariance[i] = value
        for i, value in zip((0, 7, 14, 21, 28, 35),
                            (self.model['linear_velocity_variance'], .01, 1e6, 1e6, 1e6, self.model['angular_velocity_variance'])):
            msg.twist.covariance[i] = value
        self.delayed(self.wheel_pub, msg)
        self.counts['wheel'] += 1

    def imu(self, acceleration, angular_velocity):
        msg = Imu()
        msg.header.stamp, msg.header.frame_id = stamp(self.sim_time), 'imu_link'
        msg.orientation.w, msg.orientation_covariance[0] = 1., -1.
        acceleration = np.asarray(acceleration) + self.rng.normal(0, self.model['accel_noise_std_m_s2'], 3)
        angular_velocity = np.asarray(angular_velocity) + self.rng.normal(0, self.model['gyro_noise_std_rad_s'], 3)
        for field, data in ((msg.linear_acceleration, acceleration), (msg.angular_velocity, angular_velocity)):
            field.x, field.y, field.z = map(float, data)
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = self.model['gyro_variance']
            msg.linear_acceleration_covariance[i] = self.model['accel_variance']
        self.delayed(self.imu_pub, msg)
        self.counts['imu'] += 1

    def scan(self, ranges, start_time):
        msg = LaserScan()
        msg.header.stamp, msg.header.frame_id = stamp(start_time), 'laser'
        msg.angle_min, msg.angle_max = -math.pi, math.pi
        msg.angle_increment = 2*math.pi/(len(ranges)-1)
        msg.scan_time = 1/self.model['scan_hz']
        msg.time_increment = msg.scan_time/(len(ranges)-1)
        msg.range_min, msg.range_max = self.model['scan_range_min_m'], self.model['scan_range_max_m']
        values = np.asarray(ranges, dtype=float)
        valid = np.isfinite(values)
        values[valid] += self.rng.normal(0, self.model['scan_noise_std_m'], int(valid.sum()))
        values[(values < msg.range_min) | (values > msg.range_max)] = np.inf
        msg.ranges = values.tolist()
        self.delayed(self.scan_pub, msg)
        self.counts['scan'] += 1

    def truth(self, position, quaternion, linear, angular):
        # Evaluation only. No TF publication and no feed to the EKF.
        R = rotation(quaternion)
        offset = R @ np.array([0., 0., -.086])
        p = position + offset
        v = R.T @ (linear + np.cross(angular, offset))
        w = R.T @ angular
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp(self.sim_time), 'map', 'base_footprint'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, p)
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z = map(float, quaternion)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = map(float, v)
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = map(float, w)
        self.truth_pub.publish(msg)
        self.counts['truth'] += 1
