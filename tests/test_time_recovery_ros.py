"""Opt-in DDS time/reset tests. Synthetic sensors only, no vehicle interface."""
import json
import os
import time

import pytest

if os.environ.get('AMR_RUN_TIME_TESTS') != '1':
    pytest.skip('AMR_RUN_TIME_TESTS=1, domain 194 only', allow_module_level=True)

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import GoalResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, Joy, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from amr_control.control_mux import ControlMux
from amr_control.time_health import TimeHealth
from amr_stack.navigation_manager import NavigationManager
from amr_web_ui.ros_bridge import WebBridge
from amr_web_ui.server import SharedState


@pytest.fixture
def ros():
    assert os.environ['ROS_DOMAIN_ID'] == '194'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []
    def add(node):
        nodes.append(node); executor.add_node(node); return node
    def spin(seconds=.1):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.005)
    def until(predicate, timeout=6):
        end = time.monotonic()+timeout
        while not predicate():
            assert time.monotonic() < end
            spin(.02)
    yield add, spin, until
    executor.shutdown()
    for node in reversed(nodes):
        if isinstance(node, NavigationManager): node.server.destroy()
        node.destroy_node()
    rclpy.shutdown()


def transform(parent, child, stamp, x=0.):
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id = parent, child
    msg.header.stamp = Time(seconds=stamp).to_msg()
    msg.transform.rotation.w = 1.
    msg.transform.translation.x = x
    return msg


def test_web_survives_clock_reset_and_real_style_session_restart(ros):
    add, spin, until = ros
    shared = SharedState()
    bridge = add(WebBridge(shared, parameter_overrides=[Parameter('use_sim_time', value=True)]))
    probe = add(rclpy.create_node('restart_scene'))
    clock = probe.create_publisher(Clock, '/clock', 10)
    status = probe.create_publisher(String, '/localization/status',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    dynamic, static = TransformBroadcaster(probe), StaticTransformBroadcaster(probe)
    static.sendTransform(transform('base_footprint', 'laser', 0., .2))
    state = dict(t=300., session='old', generation=999, x=10.)
    def publish():
        clock.publish(Clock(clock=Time(seconds=state['t']).to_msg()))
        dynamic.sendTransform([transform('map', 'odom', state['t'], state['x']),
                               transform('odom', 'base_footprint', state['t'])])
        status.publish(String(data=json.dumps(dict(state='READY', ready=True, detail='test', source='auto',
            stamp=time.time(), session=state['session'], generation=state['generation']))))
    probe.create_timer(.02, publish)
    until(lambda: shared.state['robot'] is not None and bridge.tf.can_transform('map', 'laser', Time()))
    assert shared.state['robot']['x'] == 10.
    old_epoch = shared.connection_epoch
    state.update(t=1., session='new', generation=0, x=2.)
    until(lambda: shared.connection_epoch > old_epoch and shared.localization_session == 'new'
          and shared.state['robot'] is not None and shared.state['robot']['x'] == 2.
          and bridge.tf.can_transform('map', 'laser', Time()))
    assert bridge.tf.lookup_transform('map', 'base_footprint', Time()).header.stamp.sec == 1
    # A stack restart on real hardware need not move the clock backwards.
    old_epoch = shared.connection_epoch
    state.update(t=2., session='third', generation=0, x=3.)
    until(lambda: shared.connection_epoch > old_epoch and shared.state['robot'] is not None
          and shared.state['robot']['x'] == 3.)
    status.publish(String(data=json.dumps(dict(state='READY', ready=True, detail='old',
        stamp=time.time(), session='old', generation=1000))))
    spin(.1)
    assert shared.localization_session == 'third'
    assert shared.state['robot']['x'] == 3.
    # Resetting the simulation clock alone must not reuse the previous READY.
    old_epoch = shared.connection_epoch
    state['t'] = .5
    until(lambda: shared.connection_epoch > old_epoch)
    spin(.1)
    assert shared.state['localization_reset_pending']
    assert not shared.snapshot()['localization']['ready']
    state['generation'] += 1
    until(lambda: shared.snapshot().get('localization', {}).get('ready')
          and bridge.tf.can_transform('map', 'laser', Time()))


def test_robot_side_gate_blocks_stale_data_clock_fault_and_guard_loss(ros):
    add, spin, until = ros
    guard = add(TimeHealth(parameter_overrides=[Parameter('recovery_duration', value=.1)]))
    mux = add(ControlMux(parameter_overrides=[Parameter('require_time_health', value=True)]))
    manager = add(NavigationManager(parameter_overrides=[
        Parameter('require_time_health', value=True), Parameter('require_localization_ready', value=False)]))
    probe = add(rclpy.create_node('time_sensor_scene'))
    offset = [0.]
    guard.now = lambda: time.time()+offset[0]  # Inject a step without changing the host clock.
    pubs = {name: probe.create_publisher(typ, topic, 10) for name, typ, topic in [
        ('scan', LaserScan, '/scan'), ('wheel', Odometry, '/wheel/odometry'),
        ('imu', Imu, '/imu/data_raw'), ('odom', Odometry, '/odometry/filtered')]}
    tf = TransformBroadcaster(probe)
    command = probe.create_publisher(Twist, '/cmd_vel_auto', 1)
    joy = probe.create_publisher(Joy, '/joy', 1)
    outputs = []
    probe.create_subscription(Twist, '/cmd_vel', lambda m: outputs.append(m.linear.x), 100)
    enabled = {'scan': True}
    def publish():
        now = guard.now()
        for kind, pub in pubs.items():
            if kind == 'scan' and not enabled['scan']: continue
            msg = pub.msg_type(); msg.header.stamp = Time(seconds=now).to_msg(); pub.publish(msg)
        tf.sendTransform(transform('odom', 'base_footprint', now))
        joy.publish(Joy(axes=[0., 0., 0.], buttons=[0]*8))
        msg = Twist(); msg.linear.x = .1; command.publish(msg)
    probe.create_timer(.02, publish)
    mux.gate.auto_enabled = True
    goal = NavigateToPose.Goal(); goal.pose.header.frame_id = 'map'; goal.pose.pose.orientation.w = 1.
    until(lambda: mux.time_health.healthy() and outputs and outputs[-1] > 0)
    assert manager.on_goal(goal) == GoalResponse.ACCEPT
    enabled['scan'] = False
    until(lambda: not mux.time_health.healthy() and outputs[-1] == 0.)
    assert manager.on_goal(goal) == GoalResponse.REJECT
    enabled['scan'] = True
    until(lambda: mux.time_health.healthy() and outputs[-1] > 0)
    offset[0] = -10.
    until(lambda: guard.latched and outputs[-1] == 0.)
    spin(.3)
    assert not mux.time_health.healthy()
    guard.speed = (.1, 0.)
    assert not guard.reset_fault(Trigger.Request(), Trigger.Response()).success
    guard.speed = (0., 0.)
    response = guard.reset_fault(Trigger.Request(), Trigger.Response())
    assert response.success  # Synthetic odometry is fresh and stationary.
    until(lambda: mux.time_health.healthy() and outputs[-1] > 0)
    for timer in tuple(guard.timers): guard.destroy_timer(timer)
    until(lambda: not mux.time_health.healthy() and outputs[-1] == 0.)
    assert manager.on_goal(goal) == GoalResponse.REJECT
