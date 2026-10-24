"""Opt-in ROS integration test using an isolated domain and no physical driver.

Run after sourcing the built workspace with AMR_RUN_ROS_TESTS=1,
ROS_DOMAIN_ID=173 and ROS_LOCALHOST_ONLY=1. Never run on a robot's active domain.
"""

import json
import os
from math import pi
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time

import pytest

if os.environ.get('AMR_RUN_ROS_TESTS') != '1':
    pytest.skip('Set AMR_RUN_ROS_TESTS=1 to run the ROS pipeline test', allow_module_level=True)

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Joy, LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def test_manual_pipeline():
    assert os.environ.get('ROS_DOMAIN_ID') == '173'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    node = rclpy.create_node('manual_pipeline_test')
    wheel_speed_limit = pi * 0.172 * 1.8
    full_turn_scale = wheel_speed_limit / (1.0 + 0.8 * 0.529 / 2)
    full_turn = (full_turn_scale, 0.8 * full_turn_scale)
    received = []
    raw = []
    previews = []
    publisher = node.create_publisher(Joy, '/joy', 1)
    auto_publisher = node.create_publisher(Twist, '/cmd_vel_auto', 1)
    # Bringup now requires fresh sensor/TF evidence before AUTO. Exercise the
    # actual time-health node with synthetic stationary data, never vehicle IO.
    sensor_publishers = [(node.create_publisher(typ, topic, 5), typ, frame)
                         for typ, topic, frame in (
                             (LaserScan, '/scan', 'laser'),
                             (Odometry, '/wheel/odometry', 'odom'),
                             (Imu, '/imu/data_raw', 'imu_link'),
                             (Odometry, '/odometry/filtered', 'odom'))]
    tf = TransformBroadcaster(node)
    health = []
    node.create_subscription(String, '/system/time_health', lambda msg: health.append(json.loads(msg.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def publish_sensors():
        stamp = node.get_clock().now().to_msg()
        for output, typ, frame in sensor_publishers:
            message = typ()
            message.header.stamp, message.header.frame_id = stamp, frame
            output.publish(message)
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id = stamp, 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.w = 1.
        tf.sendTransform(transform)

    node.create_timer(.04, publish_sensors)
    modes = []
    auto_received = []
    node.create_subscription(String, '/control_mux/mode', lambda msg: modes.append(msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Twist, '/cmd_vel_auto', lambda msg: auto_received.append(
        (time.monotonic(), msg.linear.x, msg.angular.z)), 20)
    node.create_subscription(Twist, '/cmd_vel', lambda msg: received.append(
        (msg.linear.x, msg.angular.z)), 20)
    node.create_subscription(Twist, '/cmd_vel_manual', lambda msg: raw.append(
        (msg.linear.x, msg.angular.z)), 20)
    node.create_subscription(String, '/cmd_vel_to_can/tx_preview',
                             lambda msg: previews.append(msg.data), 20)
    log = tempfile.TemporaryFile(mode='w+')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'amr_stack', 'bringup.launch.py',
         'start_joy:=false', 'can_dry_run:=true', 'start_lidar:=false',
         'start_can_telemetry:=false', 'start_ekf:=false'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )
    script = None

    def drive(held=None, linear=0.0, angular=0.0, other_button=False, duration=0.55,
              rb=False, auto_command=None):
        received.clear()
        raw.clear()
        previews.clear()
        end = time.monotonic() + duration
        next_send = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_send:
                if held is not None:
                    msg = Joy()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    # Deliberately move unused stick axes too.
                    msg.axes = [0.8, linear, angular, -0.6, 1.0, 1.0, 0.0, 0.0]
                    msg.buttons = [int(other_button)] + [0] * 5 + [int(held), int(rb)] + [0] * 7
                    publisher.publish(msg)
                if auto_command is not None:
                    msg = Twist()
                    msg.linear.x, msg.angular.z = auto_command
                    auto_publisher.publish(msg)
                next_send = now + 0.04
            rclpy.spin_once(node, timeout_sec=0.01)
        if duration >= 0.15:
            assert len(received) >= 3, 'No sustained /cmd_vel output'
        return received[-3:]

    def expect(values, target):
        for value in values:
            assert value == pytest.approx(target, abs=1e-6)
        assert len(previews) >= 3, 'No CAN payload preview'
        for line in previews[-3:]:
            assert line.startswith('DRY_RUN can0 201#')
            payload = bytes.fromhex(line.split('#')[1].split()[0])
            assert len(payload) == 8
            assert struct.unpack('<ff', payload) == pytest.approx(target, abs=1e-6)

    def brief_button_bounce():
        received.clear()
        previews.clear()
        start = time.monotonic()
        next_send = start
        while time.monotonic() - start < 0.35:
            now = time.monotonic()
            if now >= next_send:
                message = Joy()
                message.axes = [0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
                # A 40 ms false pulse must not make either output drop to zero.
                message.buttons = [0] * 6 + [int(now - start >= 0.04)] + [0] * 8
                publisher.publish(message)
                next_send = now + 0.02
            rclpy.spin_once(node, timeout_sec=0.005)
        assert len(received) >= 3 and len(previews) >= 3
        assert all(value == pytest.approx(full_turn, abs=1e-6) for value in received)
        for line in previews:
            payload = bytes.fromhex(line.split('#')[1].split()[0])
            assert struct.unpack('<ff', payload) == pytest.approx(full_turn, abs=1e-6)

    try:
        deadline = time.monotonic() + 15.0
        while (publisher.get_subscription_count() < 2
               or node.count_publishers('/cmd_vel') < 1
               or node.count_publishers('/cmd_vel_to_can/tx_preview') < 1
               or not health or not health[-1]['healthy']):
            assert launch.poll() is None, 'Bringup exited'
            assert time.monotonic() < deadline, 'ROS discovery timed out'
            rclpy.spin_once(node, timeout_sec=0.1)

        expect(drive(True, 1.0, 1.0), (0.0, 0.0))  # Held at startup.
        expect(drive(False), (0.0, 0.0))
        expect(drive(True, 1.0, 1.0), full_turn)
        assert raw[-1] == pytest.approx((1.0, 0.8))
        brief_button_bounce()
        expect(drive(None, duration=0.30), full_turn)  # Brief scheduling/input gap.
        expect(drive(True, 1.0, 1.0), full_turn)  # Recovery without a release.
        expect(drive(False, 1.0, 1.0, other_button=True), (0.0, 0.0))
        expect(drive(True, -1.0, -1.0), (-full_turn[0], -full_turn[1]))
        expect(drive(True), (0.0, 0.0))
        expect(drive(True, 1.0, 0.0), (wheel_speed_limit, 0.0))
        expect(drive(None, duration=0.85), (0.0, 0.0))  # Sustained loss of /joy.
        expect(drive(True, 1.0, 0.0), (0.0, 0.0))  # No automatic restart.
        expect(drive(False), (0.0, 0.0))
        expect(drive(True, 1.0, 0.0), (wheel_speed_limit, 0.0))
        # LB released: manual stops, then AUTO input drives after manual_holdoff even
        # though the joystick stays connected. RB has no role.
        expect(drive(False, rb=True, auto_command=(0.1, -0.2), duration=1.0), (0.1, -0.2))
        assert modes[-1] == 'AUTO'
        expect(drive(False, 1.0, 1.0, auto_command=(1.0, 0.8)), full_turn)
        expect(drive(None, auto_command=(0.1, -0.2)), (0.1, -0.2))  # Bluetooth loss: AUTO continues.
        expect(drive(False, auto_command=(0.1, -0.2)), (0.1, -0.2))  # Reconnect: still AUTO.
        expect(drive(False, duration=0.85), (0.0, 0.0))
        assert modes[-1] == 'AUTO'  # Timeout cannot fall back to manual.
        expect(drive(False, auto_command=(0.1, -0.2)), (0.1, -0.2))
        # One LB press takes over at once and arms manual; AUTO input is ignored.
        expect(drive(True, 1.0, 0.0, auto_command=(0.1, -0.2)), (wheel_speed_limit, 0.0))
        assert modes[-1] == 'MANUAL'
        expect(drive(True, auto_command=(0.1, -0.2)), (0.0, 0.0))  # LB held, stick neutral.
        expect(drive(False, auto_command=(0.1, -0.2), duration=1.0), (0.1, -0.2))
        assert modes[-1] == 'AUTO'

        # Exercise the actual standalone script through mux and CAN preview.
        expect(drive(False, duration=1.0), (0.0, 0.0))  # Let the last AUTO command expire.
        script = subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve().parents[4] / 'scripts/auto_turn_test.py')],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        deadline = time.monotonic() + 10.0
        while node.count_publishers('/cmd_vel_auto') < 2:
            assert script.poll() is None
            assert time.monotonic() < deadline
            drive(False, duration=0.2)
        expect(drive(False), (0.0, 0.0))  # Already AUTO at script start: no turn.
        def expect_script_turn(angular):
            # Mode delivery/discovery is asynchronous. Anchor checks to the
            # script's observed phase, not wall time since LB was released.
            first_sample = len(auto_received)
            deadline = time.monotonic() + 5.0
            while not any(w == pytest.approx(angular) for _, _, w
                          in auto_received[first_sample:]):
                assert script.poll() is None, 'Auto turn script exited'
                assert time.monotonic() < deadline, 'Auto turn phase timed out'
                drive(False, duration=0.1)
            expect(drive(False, duration=0.25), (0.0, angular))

        drive(True, duration=0.3)  # LB press and release: MANUAL -> AUTO starts the script.
        assert modes[-1] == 'MANUAL'
        auto_received.clear()
        expect_script_turn(-0.2)
        expect_script_turn(0.2)
        expect_script_turn(-0.2)
        right = next(t for t, v, w in auto_received if w < 0)
        left = next(t for t, v, w in auto_received if w > 0)
        repeated_right = next(t for t, v, w in auto_received if t > left and w < 0)
        assert left - right == pytest.approx(2.0, abs=0.15)
        assert repeated_right - left == pytest.approx(2.0, abs=0.15)
        assert all(v == 0.0 for _, v, _ in auto_received)
        # LB takes over from the script; releasing it repeats from the right turn.
        expect(drive(True, 1.0, 1.0), full_turn)
        assert modes[-1] == 'MANUAL'
        expect_script_turn(-0.2)
        expect(drive(True), (0.0, 0.0))
    finally:
        for process in (script, launch):
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
        log.seek(0)
        print(log.read())
        log.close()
        node.destroy_node()
        rclpy.shutdown()


def test_mux_drops_motion_until_estop_ready():
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from std_msgs.msg import String

    from amr_control.control_mux import ControlMux

    rclpy.init()
    mux = ControlMux(parameter_overrides=[Parameter('require_estop', value=True)])
    sent = []
    mux.publisher.publish = lambda message: sent.append((message.linear.x, message.angular.z))
    sequence = [0]

    def estop(state):
        sequence[0] += 1
        mux.on_estop(String(data=json.dumps(dict(
            session='bridge', sequence=sequence[0], stamp=time.time(), epoch=1, state=state,
            healthy=state == 'READY', latched=state == 'ESTOP'))))

    def auto(v):
        message = Twist()
        message.linear.x = v
        mux.on_auto_command(message)
        return sent[-1]

    try:
        assert auto(0.1) == (0.0, 0.0)  # No status yet.
        assert mux.last_status == 'estop_not_ready'
        estop('READY')
        assert auto(0.1) == (pytest.approx(0.1), 0.0)
        estop('ESTOP')
        assert sent[-1] == (0.0, 0.0)
        assert mux.gate.auto_command is None  # Cached AUTO motion is discarded.
        estop('ARMING')
        mux.publish_command()
        assert sent[-1] == (0.0, 0.0)
        estop('READY')
        assert sent[-1] == (0.0, 0.0)  # Nothing cached survives the E-stop.
        assert auto(0.1) == (pytest.approx(0.1), 0.0)
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_estop_needs_a_new_lb_press():
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from std_msgs.msg import String

    from amr_control.control_mux import ControlMux

    rclpy.init()
    mux = ControlMux(parameter_overrides=[Parameter('require_estop', value=True)])
    sent = []
    mux.publisher.publish = lambda message: sent.append((message.linear.x, message.angular.z))
    sequence = [0]

    def estop(state):
        sequence[0] += 1
        mux.on_estop(String(data=json.dumps(dict(
            session='bridge', sequence=sequence[0], stamp=time.time(), epoch=1, state=state,
            healthy=state == 'READY', latched=state == 'ESTOP'))))

    def joy(lb=0, linear=0.0):
        message = Joy()
        message.axes = [0.0] * 6
        message.axes[1] = linear
        message.buttons = [0] * 15
        message.buttons[6] = lb
        mux.on_joy(message)

    def manual(v):
        message = Twist()
        message.linear.x = v
        mux.on_command(message)
        return sent[-1]

    def hold_lb(duration=0.2, linear=0.0):
        # Longer than mode_stop_duration, so a MANUAL takeover can pass commands.
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            joy(lb=1, linear=linear)
            time.sleep(0.02)

    try:
        estop('READY')
        joy(lb=0)
        hold_lb()
        assert manual(0.1) == (pytest.approx(0.1), 0.0)

        # LB and stick held through the E-stop: READY alone must not move the robot.
        estop('ESTOP')
        joy(lb=1, linear=0.5)
        assert manual(0.1) == (0.0, 0.0)
        estop('ARMING')
        hold_lb(linear=0.5)
        estop('READY')
        hold_lb(linear=0.5)
        assert manual(0.1) == (0.0, 0.0)  # LB held through ARMING is not a new press.
        joy(lb=0)
        hold_lb()
        assert manual(0.1) == (pytest.approx(0.1), 0.0)
    finally:
        mux.destroy_node()
        rclpy.shutdown()
