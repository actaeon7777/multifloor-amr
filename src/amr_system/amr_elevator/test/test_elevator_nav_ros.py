"""Opt-in ROS test: maneuver lease (navigation_manager) and localization suspend (navigation_startup)."""
import json
import os
import time
from types import SimpleNamespace

import pytest

if os.environ.get('AMR_RUN_ELEVATOR_NAV_TESTS') != '1':
    pytest.skip('Set AMR_RUN_ELEVATOR_NAV_TESTS=1 in domain 193', allow_module_level=True)

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool

from amr_stack.navigation_manager import NavigationManager
from amr_stack.navigation_startup import NavigationStartup

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@pytest.fixture
def ros():
    assert os.environ['ROS_DOMAIN_ID'] == '193'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rclpy.init()
    executor = SingleThreadedExecutor()
    probe = rclpy.create_node('elevator_nav_probe')
    executor.add_node(probe)

    def spin(duration, until=None):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
            if until is not None and until():
                return True
        return until is None

    def call(client, value):
        assert client.wait_for_service(timeout_sec=3.)
        future = client.call_async(SetBool.Request(data=value))
        assert spin(3., future.done)
        return future.result()

    yield SimpleNamespace(executor=executor, probe=probe, spin=spin, call=call)
    executor.shutdown()
    rclpy.shutdown()


def test_maneuver_lease_gates_commands(ros):
    manager = NavigationManager()
    ros.executor.add_node(manager)
    outputs, statuses = [], []
    ros.probe.create_subscription(Twist, '/cmd_vel_auto', outputs.append, 50)
    ros.probe.create_subscription(String, '/navigation/status', lambda m: statuses.append(m.data), LATCHED)
    mode = ros.probe.create_publisher(String, '/control_mux/mode', LATCHED)
    command = ros.probe.create_publisher(Twist, '/cmd_vel_maneuver', 10)
    lease = ros.probe.create_client(SetBool, '/navigation/maneuver_lease')
    mode.publish(String(data='AUTO'))
    ros.spin(.3)

    twist = Twist()
    twist.linear.x, twist.angular.z = .2, .1
    command.publish(twist)  # no lease yet: ignored
    ros.spin(.2)
    assert all(o.linear.x == 0. for o in outputs)

    assert ros.call(lease, True).success
    assert ros.spin(1., lambda: statuses and statuses[-1] == 'MANEUVER')
    outputs.clear()
    for _ in range(6):
        command.publish(twist)
        ros.spin(.05)
    assert any(abs(o.linear.x - .2) < 1e-9 and abs(o.angular.z - .1) < 1e-9 for o in outputs)
    ros.spin(.4)  # command timeout -> zero
    assert outputs[-1].linear.x == 0.

    goals = ActionClient(ros.probe, NavigateToPose, '/navigate_to_pose')
    assert goals.wait_for_server(timeout_sec=3.)
    goal = NavigateToPose.Goal(pose=PoseStamped())
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.orientation.w = 1.
    sent = goals.send_goal_async(goal)
    assert ros.spin(3., sent.done)
    assert not sent.result().accepted  # no Nav2 goal while the lease is held

    mode.publish(String(data='MANUAL'))  # RB: lease revoked immediately
    ros.spin(.3)
    mode.publish(String(data='AUTO'))
    ros.spin(.3)
    outputs.clear()
    for _ in range(4):
        command.publish(twist)
        ros.spin(.05)
    assert all(o.linear.x == 0. for o in outputs)
    assert statuses[-1] != 'MANEUVER'

    assert ros.call(lease, True).success
    assert ros.call(lease, False).success
    assert ros.spin(1., lambda: statuses[-1] != 'MANEUVER')
    manager.destroy_node()


def test_localization_suspend_and_resume(ros):
    startup = NavigationStartup()
    ros.executor.add_node(startup)
    statuses = []
    ros.probe.create_subscription(String, '/localization/status',
                                  lambda m: statuses.append(json.loads(m.data)), LATCHED)
    suspend = ros.probe.create_client(SetBool, '/localization/suspend')
    initial = ros.probe.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    ros.spin(.5)
    generation = statuses[-1]['generation']

    response = ros.call(suspend, True)
    assert response.success
    assert ros.spin(1., lambda: statuses[-1]['state'] == 'SUSPENDED' and statuses[-1]['ready'] is False)
    assert statuses[-1]['generation'] > generation
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.pose.pose.orientation.w = 1.
    initial.publish(pose)  # ignored while suspended
    ros.spin(.5)
    assert statuses[-1]['state'] == 'SUSPENDED'
    assert ros.call(suspend, True).success  # idempotent

    assert ros.call(suspend, False).success
    assert ros.spin(1., lambda: statuses[-1]['state'] == 'WAITING_INITIAL_POSE')
    startup.destroy_node()
