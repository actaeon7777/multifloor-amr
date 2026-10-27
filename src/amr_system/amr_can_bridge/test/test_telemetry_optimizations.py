"""Message serialization and scheduling without ROS nodes or CAN transports."""
from math import cos, sin
import struct
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState

from amr_can_bridge import can_telemetry
from amr_can_bridge.telemetry import WheelOdometry, ImuPairer
from amr_control.time_guard import ClockMonitor


class Publisher:
    def __init__(self, subscribers=1):
        self.subscribers = subscribers
        self.messages = []
        self.identities = []

    def get_subscription_count(self):
        return self.subscribers

    def publish(self, message):
        self.messages.append(serialize_message(message))
        self.identities.append(id(message))


@pytest.fixture
def telemetry():
    node = object.__new__(can_telemetry.CanTelemetry)
    node.config = dict(odom_frame='odom_custom', base_frame='base_custom', imu_frame='imu_custom',
                       linear_velocity_variance=.003, angular_velocity_variance=.012,
                       gyro_variance=.0008, accel_variance=.02, max_frame_age=.25)
    node.wheels = WheelOdometry()
    node.imu = ImuPairer()
    node.init_messages()
    node.last_joint_publish = node.last_clock_check = None
    node.odom_pub, node.joints_pub, node.imu_pub = Publisher(), Publisher(), Publisher()
    node.motor_pubs = {side: Publisher(0) for side in ('left', 'right')}
    node.last_wheel = node.last_imu = None
    node.received = node.invalid = node.stale = node.odom_count = node.imu_count = 0
    node.lock = Lock()
    node.clock_monitor = ClockMonitor()
    node.clock_resets = 0
    return node


def old_wheel_messages(node, sample):
    """Original per-sample construction, retained as the serialized-byte oracle."""
    c = node.config
    msg = Odometry()
    msg.header.stamp = Time(nanoseconds=round(sample.stamp * 1e9)).to_msg()
    msg.header.frame_id, msg.child_frame_id = c['odom_frame'], c['base_frame']
    msg.pose.pose.position.x, msg.pose.pose.position.y = sample.x, sample.y
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = sin(sample.yaw/2), cos(sample.yaw/2)
    msg.twist.twist.linear.x, msg.twist.twist.angular.z = sample.v, sample.w
    position_var = .01 + .02*node.wheels.travel + node.wheels.rebases
    yaw_var = .01 + .02*(node.wheels.travel+node.wheels.rotation) + node.wheels.rebases
    for index, variance in zip((0,7,14,21,28,35),
                               (position_var, position_var, 1e6, 1e6, 1e6, yaw_var)):
        msg.pose.covariance[index] = float(variance)
    for index, variance in zip((0,7,14,21,28,35),
                               (c['linear_velocity_variance'], .01, 1e6, 1e6, 1e6,
                                c['angular_velocity_variance'])):
        msg.twist.covariance[index] = float(variance)
    joints = JointState()
    joints.header.stamp = msg.header.stamp
    joints.header.frame_id = c['base_frame']
    joints.name = ['left_drive_wheel_joint', 'right_drive_wheel_joint']
    joints.position, joints.velocity = list(sample.positions), list(sample.velocities)
    return msg, joints


def old_imu_message(node, sample):
    msg = Imu()
    msg.header.stamp = Time(nanoseconds=round(sample.stamp * 1e9)).to_msg()
    msg.header.frame_id = node.config['imu_frame']
    msg.orientation.w = 1.
    msg.orientation_covariance[0] = -1.
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = sample.acceleration
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = sample.angular_velocity
    for index in (0,4,8):
        msg.linear_acceleration_covariance[index] = node.config['accel_variance']
        msg.angular_velocity_covariance[index] = node.config['gyro_variance']
    return msg


def test_reused_messages_equal_original_serialized_bytes(telemetry, monkeypatch):
    node = telemetry
    ids = tuple(map(id, (node.odom_msg, node.joints_msg, node.imu_msg)))
    stamp_ids = tuple(id(msg.header.stamp) for msg in (node.odom_msg, node.joints_msg, node.imu_msg))
    for index, stamp in enumerate((1.0000000006, 3.9999999996, 1729000000.1234567, 2.5)):
        monkeypatch.setattr(can_telemetry, 'monotonic', lambda: float(index))
        node.wheels.travel, node.wheels.rotation, node.wheels.rebases = index*.3, index*.2, index
        sample = SimpleNamespace(stamp=stamp, x=index*.7, y=-index*.4, yaw=index*.8,
                                 v=-index*.3, w=index*.2, positions=(index*.5, -index*.7),
                                 velocities=(-index*.8, index*.9),
                                 acceleration=(index*.3, -index*.5, 9.8),
                                 angular_velocity=(-index*.1, index*.2, -index*.3))
        odom, joints = old_wheel_messages(node, sample)
        imu = old_imu_message(node, sample)
        node.publish_wheel(sample)
        node.publish_imu(sample)
        assert node.odom_pub.messages[-1] == serialize_message(odom)
        assert node.joints_pub.messages[-1] == serialize_message(joints)
        assert node.imu_pub.messages[-1] == serialize_message(imu)
        assert tuple(id(msg.header.stamp) for msg in (node.odom_msg, node.joints_msg, node.imu_msg)) == stamp_ids
    for publisher, identity in zip((node.odom_pub, node.joints_pub, node.imu_pub), ids):
        assert publisher.identities == [identity] * 4
    assert node.odom_count == node.imu_count == 4
    assert node.last_wheel == node.last_imu == 2.5


def test_joint_rate_limited_with_continuous_wheels_and_full_rate_odometry(telemetry, monkeypatch):
    node = telemetry
    joint_times = []
    for index in range(200):
        steady = index / 100.
        monkeypatch.setattr(can_telemetry, 'monotonic', lambda: steady)
        stamp = 100. + steady
        assert node.wheels.update('left', -index*.01, -1., stamp) is None
        sample = node.wheels.update('right', index*.01, 1., stamp)
        before = len(node.joints_pub.messages)
        node.publish_wheel(sample)
        old_odom, old_joints = old_wheel_messages(node, sample)
        assert node.odom_pub.messages[-1] == serialize_message(old_odom)
        if len(node.joints_pub.messages) != before:
            joint_times.append(steady)
            assert node.joints_pub.messages[-1] == serialize_message(old_joints)
    assert len(node.odom_pub.messages) == node.odom_count == 200
    assert 18 <= len(joint_times) <= 20
    assert all(right >= left + .1 for left, right in zip(joint_times, joint_times[1:]))
    assert node.wheels.travel > 1.
    assert node.last_wheel == 101.99


def test_clock_checks_throttled_and_jump_resets_under_lock(telemetry, monkeypatch):
    node = telemetry
    checks = []
    observe = node.clock_monitor.observe

    def record(ros, steady):
        assert node.lock.locked()
        checks.append(steady)
        return observe(ros, steady)

    node.clock_monitor.observe = record
    logger = Mock()
    monkeypatch.setattr(node, 'get_logger', lambda: logger)
    node.last_wheel = node.last_imu = 100.
    node.wheels.x = 4.
    node.wheels.reset_time = Mock(wraps=node.wheels.reset_time)
    node.imu.reset_time = Mock(wraps=node.imu.reset_time)
    for index in range(501):
        steady = index / 1000.
        monkeypatch.setattr(can_telemetry, 'monotonic', lambda: steady)
        monkeypatch.setattr(can_telemetry, 'time', lambda: 100. + steady + (2. if index >= 150 else 0.))
        node.check_clock()
    assert 5 <= len(checks) <= 6
    assert all(right >= left + .1 for left, right in zip(checks, checks[1:]))
    assert node.clock_resets == 1
    node.wheels.reset_time.assert_called_once()
    node.imu.reset_time.assert_called_once()
    assert node.wheels.x == 4.
    assert node.last_wheel is node.last_imu is None


@pytest.mark.parametrize('cid,side,source,data', [
    (0x017, 'right', 'power', struct.pack('<ff', 48., 2.)),
    (0x037, 'left', 'power', struct.pack('<ff', 47., -1.)),
    (0x001, 'right', 'heartbeat', struct.pack('<IBBBB', 16, 8, 0, 0, 0)),
    (0x021, 'left', 'heartbeat', struct.pack('<IBBBB', 0, 1, 0, 0, 0)),
])
def test_motor_skip_keeps_receive_counters(telemetry, monkeypatch, cid, side, source, data):
    node = telemetry
    message = SimpleNamespace(arbitration_id=cid, data=data, timestamp=100.,
                              is_extended_id=False, is_remote_frame=False,
                              is_error_frame=False, is_fd=False)
    constructor = Mock(wraps=can_telemetry.MotorStatus)
    monkeypatch.setattr(can_telemetry, 'MotorStatus', constructor)
    node.process_message(message, 100.)
    constructor.assert_not_called()
    assert node.received == 1 and node.invalid == node.stale == 0
    node.motor_pubs[side].subscribers = 1
    node.process_message(message, 100.)
    constructor.assert_called_once()
    assert len(node.motor_pubs[side].messages) == 1
    assert node.received == 2 and node.invalid == node.stale == 0


def test_preview_skips_work_without_subscribers(monkeypatch):
    from amr_can_bridge import cmd_vel_to_can
    from amr_can_bridge.cmd_vel_to_can import CmdVelToCan
    monkeypatch.setattr(cmd_vel_to_can.rclpy, 'ok', lambda context=None: True)
    node = SimpleNamespace(preview=Publisher(0), context=object())
    # No sender/payload: returning early must precede formatting.
    CmdVelToCan.publish_preview(node, 1., 2., None, 'test')
    assert node.preview.messages == []


def test_preview_after_context_shutdown_does_not_touch_rcl(monkeypatch):
    """The shutdown stop frame calls publish_preview after the context may be gone."""
    from amr_can_bridge import cmd_vel_to_can
    from amr_can_bridge.cmd_vel_to_can import CmdVelToCan
    monkeypatch.setattr(cmd_vel_to_can.rclpy, 'ok', lambda context=None: False)

    class Gone:
        def get_subscription_count(self):
            raise RuntimeError('context is not valid')

    CmdVelToCan.publish_preview(SimpleNamespace(preview=Gone(), context=object()), 0., 0., None, 'stop')
