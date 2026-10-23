"""Startup safety checks with real ROS topics/services and synthetic sensors.

No Nav2 controller, hardware driver, simulator, or motion output is started.
The only clock acceleration affects the supervisor's internal wall watchdogs.
"""

import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace

import numpy as np
import pytest

if os.environ.get('AMR_RUN_STARTUP_TESTS') != '1':
    pytest.skip('Opt in with AMR_RUN_STARTUP_TESTS=1, domain 194 localhost only', allow_module_level=True)

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from amr_localization_interfaces.srv import GlobalLocalize
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.msg import Particle, ParticleCloud
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from rclpy.time import Time
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from amr_stack import navigation_startup
from amr_stack.navigation_startup import NavigationStartup


class SyntheticLocalization(Node):
    """Two identical corridors deliberately require a human pose prior."""

    def __init__(self, speed=0., map_tf=True, yaw_variance=.01,
                 global_delay=0., start_success=True, real_amcl=False,
                 unique_map=False, existing_pose=False, parameter_fail_value=None,
                 parameter_readback=None, parameter_delay=0.,
                 freeze_after_tracking=False, mismatch_after_tracking=False,
                 freeze_scan_after_tracking=False, scan_count=120,
                 tim_backend=False, tim_delay=0., tim_result='success',
                 laser_extrinsic=(0., 0., 0.)):
        super().__init__('synthetic_localization_for_startup_test')
        self.speed = speed
        self.map_tf = map_tf
        self.yaw_variance = yaw_variance
        self.global_delay = global_delay
        self.tim_delay, self.tim_result = tim_delay, tim_result
        self.tim_calls = self.tim_completed = 0
        self.tim_requests, self.initial_seeds = [], []
        self.laser_extrinsic = laser_extrinsic
        self.start_success = start_success
        self.scan_count = scan_count
        self.emit_sensors = True
        self.emit_pose = existing_pose
        self.navigation_active = False
        self.global_calls = 0
        self.global_completed = 0
        self.nomotion_calls = 0
        self.start_commands = []
        self.max_beams = 360
        self.parameter_fail_value = parameter_fail_value
        self.parameter_readback = parameter_readback
        self.parameter_delay = parameter_delay
        self.freeze_after_tracking = freeze_after_tracking
        self.mismatch_after_tracking = mismatch_after_tracking
        self.freeze_scan_after_tracking = freeze_scan_after_tracking
        self.last_scan = self.frozen_scan = None
        self.events = []
        self.last_pose = self.last_cloud = None
        self.parameter_inflight = 0
        self.max_parameter_inflight = 0
        self.pose = (4.15, 2.55, .13) if unique_map else (2.05, 1.55, .13)
        self.odom_pose = (0., 0., 0.)
        self.latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', self.latched)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', qos_profile_sensor_data)
        self.pose_pub = (None if real_amcl else
                         self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', self.latched))
        self.cloud_pub = (None if real_amcl else
                          self.create_publisher(ParticleCloud, '/particle_cloud', qos_profile_sensor_data))
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(self.transform('base_footprint', 'laser', laser_extrinsic))
        if tim_backend:
            self.create_service(GlobalLocalize, '/localization/global_localize', self.tim_localize,
                                callback_group=ReentrantCallbackGroup())
            self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.accept_seed, 10)
        for name in ('amcl', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator'):
            if name == 'amcl' and real_amcl:
                continue
            self.create_service(GetState, f'/{name}/get_state',
                                lambda request, response, node_name=name: self.lifecycle(response, node_name))
        self.create_service(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes', self.manage)
        if not real_amcl:
            self.create_service(SetParametersAtomically, '/amcl/set_parameters_atomically',
                                self.set_amcl_parameters, callback_group=ReentrantCallbackGroup())
            self.create_service(GetParameters, '/amcl/get_parameters', self.get_amcl_parameters)
            self.create_service(Empty, '/reinitialize_global_localization', self.reset_global,
                                callback_group=ReentrantCallbackGroup())
            self.create_service(Empty, '/request_nomotion_update', self.nomotion)
        if unique_map:
            self.grid = np.full((60, 100), -1, dtype=int)
            self.grid[5:55, 5:95] = 0
            self.grid[[5, 54], 5:95] = 100
            self.grid[5:55, [5, 94]] = 100
            for y1, y2, x1, x2 in ((35, 55, 20, 29), (5, 19, 52, 64),
                                    (31, 40, 68, 80), (13, 18, 16, 22)):
                self.grid[y1:y2, x1:x2] = 100
        else:
            self.grid = np.full((40, 100), -1, dtype=int)
            for x0 in (0, 50):
                self.grid[5:35, x0 + 5:x0 + 45] = 0
                self.grid[[5, 34], x0 + 5:x0 + 45] = 100
                self.grid[5:35, [x0 + 5, x0 + 44]] = 100
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.info.height, message.info.width = self.grid.shape
        message.info.resolution = .1
        message.info.origin.orientation.w = 1.
        message.data = self.grid.ravel().tolist()
        self.map_message = message
        self.map_pub.publish(message)
        self.ranges = self.raycast()
        self.create_timer(.05, self.publish_sensors)

    def raycast(self):
        ranges = []
        pose = self.laser_pose()
        for angle in -math.pi + np.arange(self.scan_count) * 2 * math.pi / self.scan_count + pose[2]:
            result = float('inf')
            for distance in np.arange(.02, 12., .02):
                x = math.floor((pose[0] + distance * math.cos(angle)) / .1)
                y = math.floor((pose[1] + distance * math.sin(angle)) / .1)
                if not (0 <= x < self.grid.shape[1] and 0 <= y < self.grid.shape[0]):
                    break
                if self.grid[y, x] == 100:
                    result = float(distance)
                    break
            ranges.append(result)
        return ranges

    def laser_pose(self):
        x, y, theta = self.pose
        dx, dy, dtheta = self.laser_extrinsic
        return (x+math.cos(theta)*dx-math.sin(theta)*dy,
                y+math.sin(theta)*dx+math.cos(theta)*dy, theta+dtheta)

    def accept_seed(self, message):
        self.initial_seeds.append(message)
        self.event('initialpose', self.max_beams)
        self.emit_pose = True

    async def tim_localize(self, request, response):
        captured_laser_pose = self.laser_pose()
        self.tim_calls += 1
        self.tim_requests.append(request)
        self.event('tim_request', self.max_beams)
        if self.tim_delay:
            future = Future()

            def finish():
                timer.cancel()
                future.set_result(None)

            timer = self.create_timer(self.tim_delay, finish)
            await future
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = request.scan.header.stamp
        x, y, theta = captured_laser_pose
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(theta/2), math.cos(theta/2)
        response.success, response.message = self.tim_result != 'failure', 'Synthetic TIM result'
        response.poses, response.scores, response.elapsed_sec = [pose], [.99], self.tim_delay
        if self.tim_result == 'empty':
            response.poses, response.scores = [], []
        elif self.tim_result == 'nan_score':
            response.scores = [float('nan')]
        elif self.tim_result == 'nan_pose':
            pose.pose.position.x = float('nan')
        elif self.tim_result == 'wrong_frame':
            pose.header.frame_id = 'odom'
        elif self.tim_result == 'wrong_stamp':
            pose.header.stamp.sec += 1
        elif self.tim_result == 'low_quality':
            pose.pose.position.x += 1.
        self.tim_completed += 1
        return response

    def transform(self, parent, child, pose, stamp=None):
        message = TransformStamped()
        message.header.frame_id, message.child_frame_id = parent, child
        message.header.stamp = stamp or self.get_clock().now().to_msg()
        message.transform.translation.x, message.transform.translation.y = pose[:2]
        message.transform.rotation.z = math.sin(pose[2] / 2)
        message.transform.rotation.w = math.cos(pose[2] / 2)
        return message

    def lifecycle(self, response, name):
        response.current_state.id = (State.PRIMARY_STATE_ACTIVE
                                     if name == 'amcl' or self.navigation_active
                                     else State.PRIMARY_STATE_UNCONFIGURED)
        return response

    def manage(self, request, response):
        self.start_commands.append(request.command)
        self.event('startup', request.command)
        response.success = self.start_success and request.command == ManageLifecycleNodes.Request.STARTUP
        self.navigation_active = response.success
        return response

    async def reset_global(self, request, response):
        self.global_calls += 1
        self.event('global_reset', self.max_beams)
        if self.global_delay:
            future = Future()

            def finish():
                timer.cancel()
                future.set_result(None)

            timer = self.create_timer(self.global_delay, finish)
            await future
        self.emit_pose = True
        self.global_completed += 1
        return response

    def event(self, kind, value):
        self.events.append(dict(kind=kind, value=value, max_beams=self.max_beams,
                                stamp=self.get_clock().now().nanoseconds))

    async def set_amcl_parameters(self, request, response):
        # Setting only this parameter must not reconfigure AMCL's particles,
        # range filter, or lifecycle as a side effect of changing beam phases.
        assert len(request.parameters) == 1
        parameter = request.parameters[0]
        assert parameter.name == 'max_beams'
        assert parameter.value.type == ParameterType.PARAMETER_INTEGER
        requested = parameter.value.integer_value
        self.event('set_requested', requested)
        self.parameter_inflight += 1
        self.max_parameter_inflight = max(self.max_parameter_inflight, self.parameter_inflight)
        try:
            if self.parameter_delay:
                future = Future()

                def finish():
                    timer.cancel()
                    future.set_result(None)

                timer = self.create_timer(self.parameter_delay, finish)
                await future
            response.result.successful = requested != self.parameter_fail_value
            if response.result.successful:
                self.max_beams = requested
                self.event('set_applied', requested)
                if requested == 60:
                    if self.freeze_after_tracking:
                        self.emit_pose = False
                    if self.mismatch_after_tracking:
                        self.ranges = [.2] * len(self.ranges)
                    if self.freeze_scan_after_tracking:
                        self.frozen_scan = self.last_scan
            else:
                response.result.reason = 'Injected AMCL parameter rejection'
            return response
        finally:
            self.parameter_inflight -= 1

    def get_amcl_parameters(self, request, response):
        assert request.names == ['max_beams']
        value = self.max_beams if self.parameter_readback is None else self.parameter_readback
        self.event('get_confirmed', value)
        response.values = [ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)]
        return response

    def nomotion(self, request, response):
        self.nomotion_calls += 1
        return response

    def estimated_pose(self, stamp=None):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = stamp or self.get_clock().now().to_msg()
        message.pose.pose.position.x, message.pose.pose.position.y = self.pose[:2]
        message.pose.pose.orientation.z = math.sin(self.pose[2] / 2)
        message.pose.pose.orientation.w = math.cos(self.pose[2] / 2)
        message.pose.covariance[0] = message.pose.covariance[7] = .01
        message.pose.covariance[35] = self.yaw_variance
        return message

    def publish_sensors(self):
        if not self.emit_sensors:
            return
        stamp = self.get_clock().now().to_msg()
        transforms = [self.transform('odom', 'base_footprint', self.odom_pose, stamp)]
        if self.map_tf:
            map_odom = navigation_startup.compose_se2(self.pose, navigation_startup.inverse_se2(self.odom_pose))
            transforms.append(self.transform('map', 'odom', map_odom, stamp))
        self.tf.sendTransform(transforms)
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
        odom.header.stamp = stamp
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.odom_pose[:2]
        odom.pose.pose.orientation.z = math.sin(self.odom_pose[2]/2)
        odom.pose.pose.orientation.w = math.cos(self.odom_pose[2]/2)
        odom.twist.twist.linear.x = self.speed
        self.odom_pub.publish(odom)
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'laser', stamp
        scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / self.scan_count
        scan.angle_max = scan.angle_min + (self.scan_count - 1) * scan.angle_increment
        scan.range_min, scan.range_max = .01, 12.
        scan.ranges = self.ranges
        self.last_scan = scan
        self.scan_pub.publish(self.frozen_scan or scan)
        if self.emit_pose and self.pose_pub is not None:
            pose = self.estimated_pose(stamp)
            self.last_pose = pose
            self.pose_pub.publish(pose)
            cloud = ParticleCloud()
            cloud.header = pose.header
            cloud.particles = [Particle(pose=pose.pose.pose, weight=.01) for _ in range(100)]
            self.last_cloud = cloud
            self.cloud_pub.publish(cloud)


class Harness:
    def __init__(self, automatic=False, startup_parameters=None, **backend_options):
        assert os.environ['ROS_DOMAIN_ID'] == '194'
        assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
        rclpy.init()
        if (startup_parameters or {}).get('global_localization_backend') == 'tim':
            backend_options.setdefault('tim_backend', True)
        self.backend = SyntheticLocalization(**backend_options)
        overrides = dict(automatic_localization=automatic, verification_duration=.1,
                         manual_verification_duration=.1, manual_verification_samples=2,
                         manual_refinement=False,
                         global_localization_backend='amcl',
                         verification_samples=2, stationary_duration=.15,
                         search_timeout=10., data_timeout=.7, pose_timeout=1.,
                         initialization_max_beams=360, tracking_max_beams=60,
                         quality_max_beams=360, navigation_start_timeout=5.)
        overrides.update(startup_parameters or {})
        self.supervisor = NavigationStartup(parameter_overrides=[Parameter(k, value=v) for k, v in overrides.items()])
        self.probe = Node('navigation_startup_test_probe')
        self.initial_pub = self.probe.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.statuses = []
        self.probe.create_subscription(String, '/localization/status',
                                       lambda m: self.statuses.append(json.loads(m.data)), self.backend.latched)
        self.retry = self.probe.create_client(Trigger, '/localization/relocalize')
        self.executor = SingleThreadedExecutor()
        for node in (self.backend, self.supervisor, self.probe):
            self.executor.add_node(node)
        self.until(lambda: self.supervisor.map_quality is not None
                   and self.initial_pub.get_subscription_count() > 0)

    def until(self, predicate, timeout=10.):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, (self.supervisor.state, self.supervisor.detail,
                                                 self.backend.start_commands, self.supervisor.metrics)
            self.executor.spin_once(timeout_sec=.01)

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.01)

    def set_initial_pose(self):
        self.initial_pub.publish(self.backend.estimated_pose())
        self.until(lambda: self.supervisor.source == 'manual')
        # AMCL's response must arrive after the initial-pose message timestamp.
        self.backend.emit_pose = True

    def ready(self):
        self.set_initial_pose()
        self.until(lambda: self.supervisor.state == 'READY')
        self.until(lambda: self.statuses and self.statuses[-1]['ready'])

    def close(self):
        self.executor.shutdown()
        for node in (self.supervisor, self.backend, self.probe):
            node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def harnesses():
    created = []

    def create(**kwargs):
        result = Harness(**kwargs)
        created.append(result)
        return result

    try:
        yield create
    finally:
        for harness in created:
            harness.close()


def test_late_initial_pose_starts_navigation_without_restarting_after_sixty_seconds(harnesses, monkeypatch):
    elapsed = [0.]
    clock = SimpleNamespace(monotonic=lambda: time.monotonic() + elapsed[0],
                            time=time.time, time_ns=time.time_ns)
    monkeypatch.setattr(navigation_startup, 'time', clock)
    h = harnesses()
    h.until(lambda: h.supervisor.state == 'WAITING_INITIAL_POSE')
    elapsed[0] += 65.
    h.spin(1.6)
    assert h.supervisor.state == 'WAITING_INITIAL_POSE'
    assert not h.backend.start_commands
    session = h.supervisor.session
    h.ready()
    assert h.supervisor.session == session
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]
    assert h.backend.global_calls == 0


def test_automatic_search_waits_for_stationary_and_rejects_repeated_corridors(harnesses):
    h = harnesses(automatic=True, speed=.1)
    h.until(lambda: h.supervisor.data_problem() is None)
    h.spin(.7)
    assert h.backend.global_calls == 0
    h.until(h.retry.service_is_ready)
    rejected = h.retry.call_async(Trigger.Request())
    h.until(rejected.done)
    assert not rejected.result().success
    h.backend.speed = 0.
    h.until(lambda: h.backend.global_calls == 1)
    h.until(lambda: h.supervisor.state == 'WAITING_INITIAL_POSE', timeout=14.)
    assert h.supervisor.metrics['particle_concentration'] == pytest.approx(1.)
    assert h.supervisor.metrics['rival_search_completed']
    assert h.supervisor.metrics['rival_score'] >= .9
    assert not h.backend.start_commands
    # A human prior resolves the deliberate repeated-corridor ambiguity.
    h.ready()
    assert h.backend.global_calls == 1
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]


def test_missing_map_transform_blocks_startup_until_transform_is_available(harnesses):
    h = harnesses(map_tf=False)
    h.until(lambda: h.supervisor.data_problem() is None)
    h.set_initial_pose()
    h.spin(1.1)
    assert h.supervisor.state != 'READY'
    assert not h.backend.start_commands
    h.backend.map_tf = True
    h.until(lambda: h.supervisor.state == 'READY')
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]


def test_large_yaw_covariance_blocks_otherwise_good_scan_match(harnesses):
    h = harnesses(yaw_variance=.8)
    h.set_initial_pose()
    h.until(lambda: '불확실' in h.supervisor.detail)
    assert h.supervisor.state != 'READY'
    assert not h.backend.start_commands
    assert '불확실' in h.supervisor.detail
    h.backend.yaw_variance = .01
    h.until(lambda: h.supervisor.state == 'READY')


def test_stale_sensors_revoke_ready_and_reverify_without_second_startup(harnesses):
    h = harnesses()
    h.ready()
    h.backend.emit_sensors = False
    h.until(lambda: h.supervisor.state == 'WAITING_DATA')
    h.until(lambda: h.statuses and not h.statuses[-1]['ready'])
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]
    h.backend.emit_sensors = True
    h.until(lambda: h.supervisor.state == 'READY')
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]


def test_scan_ahead_of_tf_uses_fresh_history_without_extending_freshness(harnesses):
    h = harnesses()
    h.ready()
    h.backend.emit_sensors = False
    # Drain the final ordinary publication so its exact-stamp TF is available.
    h.spin(.08)
    ahead = copy.deepcopy(h.supervisor.scan)
    ahead.header.stamp = h.probe.get_clock().now().to_msg()
    ahead_stamp = navigation_startup.stamp_ns(ahead.header.stamp)
    h.backend.scan_pub.publish(ahead)
    h.until(lambda: h.supervisor.scan_samples
            and navigation_startup.stamp_ns(h.supervisor.scan_samples[-1][0].header.stamp) == ahead_stamp)
    assert not h.supervisor.tf_buffer.can_transform('odom', 'base_footprint',
                                                    Time.from_msg(ahead.header.stamp))
    h.supervisor.tick()
    assert h.supervisor.state == 'READY'
    # It must select a genuinely older measured scan with exact-stamp TF; it
    # must neither extrapolate nor pretend the newest scan has a valid TF.
    assert navigation_startup.stamp_ns(h.supervisor.scan.header.stamp) < ahead_stamp
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]
    h.until(lambda: h.supervisor.state == 'WAITING_DATA', timeout=3.)
    assert not h.supervisor.fresh('scan', h.supervisor.cfg['data_timeout'])
    h.until(lambda: h.statuses and not h.statuses[-1]['ready'])


def test_previous_amcl_pose_cannot_count_as_post_initialization_evidence(harnesses):
    h = harnesses()
    h.backend.emit_pose = True
    h.until(lambda: h.supervisor.pose is not None)
    h.backend.emit_pose = False
    h.initial_pub.publish(h.backend.estimated_pose())
    h.until(lambda: h.supervisor.source == 'manual')
    h.spin(1.1)
    assert h.supervisor.pose is None
    assert not h.backend.start_commands
    h.backend.emit_pose = True
    h.until(lambda: h.supervisor.state == 'READY')


def test_delayed_global_reset_timeout_cannot_be_cleared_by_manual_pose(harnesses):
    # Allow the set/readback handshake (two supervisor ticks) to complete,
    # then deliberately exceed the same watchdog on the global reset itself.
    h = harnesses(automatic=True, global_delay=3., startup_parameters={'service_timeout': 1.3})
    h.until(lambda: h.backend.global_calls == 1)
    h.until(lambda: h.supervisor.state == 'FAILED')
    assert h.supervisor.reset_uncertain
    h.initial_pub.publish(h.backend.estimated_pose())
    h.until(h.retry.service_is_ready)
    retry = h.retry.call_async(Trigger.Request())
    h.until(retry.done)
    assert not retry.result().success
    # Canceling the caller Future cannot cancel the remote reset.  Its eventual
    # success and subsequent pose traffic must never silently release the gate.
    h.until(lambda: h.backend.global_completed == 1)
    h.spin(1.1)
    assert h.supervisor.state == 'FAILED'
    assert not h.backend.start_commands
    assert not h.statuses[-1]['ready']


def test_failed_navigation_startup_is_not_repeated(harnesses):
    h = harnesses(start_success=False)
    h.set_initial_pose()
    h.until(lambda: h.supervisor.state == 'FAILED')
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]
    h.spin(1.1)
    assert h.supervisor.state == 'FAILED'
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]


def test_automatic_unique_scan_reaches_ready_without_manual_pose(harnesses):
    h = harnesses(automatic=True, unique_map=True)
    h.until(lambda: h.supervisor.state == 'READY')
    assert h.supervisor.source == 'auto'
    assert h.backend.global_calls == h.backend.global_completed == 1
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]
    events = h.backend.events
    initial_confirmation = next(i for i, event in enumerate(events)
                                if event['kind'] == 'get_confirmed' and event['value'] == 360)
    global_reset = next(i for i, event in enumerate(events) if event['kind'] == 'global_reset')
    tracking_confirmation = next(i for i, event in enumerate(events)
                                 if event['kind'] == 'get_confirmed' and event['value'] == 60)
    startup = next(i for i, event in enumerate(events) if event['kind'] == 'startup')
    assert initial_confirmation < global_reset < tracking_confirmation < startup
    assert events[global_reset]['max_beams'] == 360
    assert events[startup]['max_beams'] == 60
    assert navigation_startup.stamp_ns(h.supervisor.pose.header.stamp) > events[tracking_confirmation]['stamp']
    assert h.supervisor.metrics['rival_search_completed']
    assert h.supervisor.metrics['rival_score'] < h.supervisor.metrics['match_ratio'] - .1
    h.spin(1.1)
    assert h.supervisor.state == 'READY'
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]


def test_tracking_phase_requires_fresh_pose_and_scan_evidence(harnesses):
    h = harnesses(freeze_after_tracking=True)
    h.set_initial_pose()
    h.until(lambda: any(event['kind'] == 'get_confirmed' and event['value'] == 60
                        for event in h.backend.events))
    assert h.backend.last_pose is not None and h.backend.last_cloud is not None
    h.spin(.7)
    assert not h.backend.start_commands
    assert h.supervisor.state != 'READY'
    # Reliable DDS replay after a parameter response must not turn observations
    # measured with the initial beam phase into post-switch verification.
    h.backend.pose_pub.publish(h.backend.last_pose)
    h.backend.cloud_pub.publish(h.backend.last_cloud)
    h.spin(.7)
    assert not h.backend.start_commands
    assert h.supervisor.state != 'READY'
    h.backend.emit_pose = True
    h.until(lambda: h.supervisor.state == 'READY')
    assert h.backend.max_beams == 60
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]


def test_tracking_phase_bad_scan_cannot_reuse_successful_initial_verification(harnesses):
    h = harnesses(mismatch_after_tracking=True)
    original_ranges = list(h.backend.ranges)
    h.set_initial_pose()
    h.until(lambda: any(event['kind'] == 'get_confirmed' and event['value'] == 60
                        for event in h.backend.events))
    h.spin(1.6)
    assert not h.backend.start_commands
    assert h.supervisor.state != 'READY'
    h.backend.ranges = original_ranges
    h.until(lambda: h.supervisor.state == 'READY')


def test_tracking_phase_requires_scan_measured_after_parameter_confirmation(harnesses):
    h = harnesses(freeze_scan_after_tracking=True)
    h.set_initial_pose()
    h.until(lambda: any(event['kind'] == 'get_confirmed' and event['value'] == 60
                        for event in h.backend.events))
    h.spin(1.2)
    # Fresh odometry and AMCL messages cannot legitimize a replayed scan stamp.
    assert h.supervisor.fresh('pose', h.supervisor.cfg['pose_timeout'])
    assert not h.backend.start_commands
    assert h.supervisor.state != 'READY'
    h.backend.frozen_scan = None
    h.until(lambda: h.supervisor.state == 'READY')


def test_explicit_retry_restores_initial_beams_before_new_global_search(harnesses):
    h = harnesses(automatic=True, unique_map=True,
                  startup_parameters={'initialization_max_beams': 1080})
    h.until(lambda: h.supervisor.state == 'READY')
    start_index = len(h.backend.events)
    generation = h.supervisor.generation
    h.until(h.retry.service_is_ready)
    retry = h.retry.call_async(Trigger.Request())
    h.until(retry.done)
    assert retry.result().success
    h.until(lambda: h.supervisor.state == 'READY' and h.supervisor.generation > generation)
    events = h.backend.events[start_index:]
    initial_confirmation = next(i for i, event in enumerate(events)
                                if event['kind'] == 'get_confirmed' and event['value'] == 1080)
    global_reset = next(i for i, event in enumerate(events) if event['kind'] == 'global_reset')
    tracking_confirmation = next(i for i, event in enumerate(events)
                                 if event['kind'] == 'get_confirmed' and event['value'] == 60)
    assert initial_confirmation < global_reset < tracking_confirmation
    assert h.backend.global_calls == 2
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]
    assert h.backend.max_parameter_inflight == 1


def test_deferred_global_reset_rechecks_stationarity_after_beam_confirmation(harnesses):
    h = harnesses(automatic=True, unique_map=True, parameter_delay=.7)
    h.until(lambda: any(event['kind'] == 'set_requested' and event['value'] == 360
                        for event in h.backend.events))
    h.backend.speed = .1
    h.until(lambda: any(event['kind'] == 'get_confirmed' and event['value'] == 360
                        for event in h.backend.events))
    h.spin(1.1)
    assert h.backend.global_calls == 0
    assert not h.backend.start_commands
    h.backend.speed = 0.
    h.until(lambda: h.supervisor.state == 'READY', timeout=15.)
    assert h.backend.global_calls == 1


def test_new_manual_pose_supersedes_inflight_tracking_change_without_overlap(harnesses):
    h = harnesses(parameter_delay=.7)
    h.set_initial_pose()
    h.until(lambda: any(event['kind'] == 'set_requested' and event['value'] == 60
                        for event in h.backend.events))
    generation = h.supervisor.generation
    event_index = len(h.backend.events)
    h.initial_pub.publish(h.backend.estimated_pose())
    h.until(lambda: h.supervisor.generation > generation)
    h.until(lambda: h.supervisor.state == 'READY', timeout=15.)
    events = h.backend.events[event_index:]
    # The old remote set may finish, but its result must not count as evidence
    # for the newer pose. Serialize, restore360, verify, then restore60 again.
    initial_confirmation = next(i for i, event in enumerate(events)
                                if event['kind'] == 'get_confirmed' and event['value'] == 360)
    tracking_confirmations = [i for i, event in enumerate(events)
                              if event['kind'] == 'get_confirmed' and event['value'] == 60]
    startup = next(i for i, event in enumerate(events) if event['kind'] == 'startup')
    assert tracking_confirmations[-1] > initial_confirmation
    assert startup > tracking_confirmations[-1]
    assert h.backend.max_parameter_inflight == 1
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]


@pytest.mark.parametrize('failure', ['rejected_initial', 'rejected_tracking', 'mismatched_readback'])
def test_unconfirmed_beam_parameter_blocks_navigation(harnesses, failure):
    options = ({'parameter_fail_value': 360} if failure == 'rejected_initial'
               else {'parameter_fail_value': 60} if failure == 'rejected_tracking'
               else {'parameter_readback': 61})
    h = harnesses(**options)
    h.set_initial_pose()
    h.until(lambda: h.supervisor.state == 'FAILED')
    h.spin(.7)
    assert not h.backend.start_commands
    assert h.backend.global_calls == 0
    assert not h.statuses[-1]['ready']


def test_parameter_timeout_cannot_be_overridden_by_manual_pose_or_retry(harnesses):
    h = harnesses(automatic=True, parameter_delay=1.5,
                  startup_parameters={'service_timeout': .3})
    h.until(lambda: any(event['kind'] == 'set_requested' for event in h.backend.events))
    h.until(lambda: h.supervisor.state == 'FAILED')
    h.initial_pub.publish(h.backend.estimated_pose())
    h.until(h.retry.service_is_ready)
    retry = h.retry.call_async(Trigger.Request())
    h.until(retry.done)
    assert not retry.result().success
    # A remote parameter change can still complete after the client times out.
    # Its eventual application cannot silently lift the fail-closed gate.
    h.until(lambda: any(event['kind'] == 'set_applied' for event in h.backend.events))
    h.spin(1.1)
    assert h.supervisor.state == 'FAILED'
    assert not h.backend.start_commands
    assert h.backend.global_calls == 0
    assert not h.statuses[-1]['ready']


def test_existing_amcl_estimate_is_preserved_but_requires_new_evidence(harnesses):
    h = harnesses(automatic=True, unique_map=True, existing_pose=True)
    h.until(lambda: h.supervisor.pose is not None and h.supervisor.cloud is not None)
    assert not h.supervisor.initialized
    old_pose, old_cloud = h.supervisor.pose, h.supervisor.cloud
    old_generation = h.supervisor.generation
    h.backend.emit_pose = False
    h.spin(.08)
    h.until(lambda: h.supervisor.initialized)
    assert h.supervisor.generation != old_generation
    assert h.supervisor.source == 'auto'
    assert h.supervisor.pose is None and h.supervisor.cloud is None
    assert h.backend.global_calls == 0
    h.spin(.6)
    assert h.supervisor.state != 'READY'
    assert not h.backend.start_commands
    # Re-delivery is not fresh evidence merely because it arrived after the
    # new supervisor generation; the AMCL observation stamp must also advance.
    h.backend.pose_pub.publish(old_pose)
    h.backend.cloud_pub.publish(old_cloud)
    h.until(lambda: h.supervisor.pose is not None and h.supervisor.cloud is not None)
    h.supervisor.tick()
    assert h.supervisor.state != 'READY'
    assert not h.backend.start_commands
    h.backend.emit_pose = True
    h.until(lambda: h.supervisor.state == 'READY')
    assert h.backend.global_calls == 0
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]
    assert h.supervisor.metrics['rival_search_completed']
    assert navigation_startup.stamp_ns(h.supervisor.pose.header.stamp) >= h.supervisor.attempt_stamp


def test_tim_success_preserves_automatic_verification_and_beam_order(harnesses):
    h = harnesses(automatic=True, unique_map=True,
                  startup_parameters={'global_localization_backend': 'tim',
                                      'initialization_max_beams': 1080})
    h.until(lambda: h.supervisor.state == 'READY', timeout=15.)
    assert h.supervisor.source == 'auto'
    assert h.backend.tim_calls == h.backend.tim_completed == 1
    assert h.backend.global_calls == 0
    assert len(h.backend.initial_seeds) == 1
    assert h.supervisor.metrics['rival_search_completed']
    assert h.supervisor.tim_metrics['tim_status'] == 'accepted'
    events = h.backend.events
    kinds = [event['kind'] for event in events]
    tim = kinds.index('tim_request')
    seed = kinds.index('initialpose')
    track = next(i for i, event in enumerate(events)
                 if event['kind'] == 'get_confirmed' and event['value'] == 60)
    assert any(event['kind'] == 'get_confirmed' and event['value'] == 1080 for event in events[:tim])
    assert tim < seed < track < kinds.index('startup')
    request = h.backend.tim_requests[0]
    assert request.map.info.width == h.backend.map_message.info.width
    assert len(request.scan.ranges) == h.backend.scan_count
    assert request.max_candidates == 5 and request.timeout_sec == 30.
    h.until(lambda: h.statuses[-1]['ready'])
    assert h.statuses[-1]['initializer_backend'] == 'tim'
    assert h.statuses[-1]['tim_candidates'] == 1


def test_tim_laser_pose_is_converted_using_translation_and_yaw_extrinsic(harnesses):
    h = harnesses(automatic=True, unique_map=True, laser_extrinsic=(.24, -.13, .37),
                  startup_parameters={'global_localization_backend': 'tim'})
    h.until(lambda: h.supervisor.state == 'READY', timeout=15.)
    seed = h.backend.initial_seeds[0]
    assert navigation_startup.planar(seed.pose.pose) == pytest.approx(h.backend.pose)
    assert navigation_startup.stamp_ns(seed.header.stamp) >= navigation_startup.stamp_ns(
        h.backend.tim_requests[0].scan.header.stamp)
    assert h.supervisor.source == 'auto'


@pytest.mark.parametrize('odom_yaw', [.8, math.pi-.003])
def test_tim_transports_quiet_drift_to_exact_latest_scan_with_offset_laser(harnesses, odom_yaw, monkeypatch):
    h = harnesses(automatic=True, unique_map=True, tim_delay=1.5,
                  laser_extrinsic=(.24, -.13, .37),
                  startup_parameters={'global_localization_backend': 'tim'})
    h.backend.odom_pose = (1., -2., odom_yaw)
    h.until(lambda: h.backend.tim_calls == 1)
    original = h.backend.pose
    # A known 8mm forward / 6mm sideways / .006rad quiet drift expressed in
    # the captured base axes. Nonzero odom yaw and a pi crossing catch frame
    # order and angle wrapping errors independently of endpoint map matching.
    dx, dy, turn = .008, -.006, .006
    h.backend.pose = (original[0]+math.cos(original[2])*dx-math.sin(original[2])*dy,
                      original[1]+math.sin(original[2])*dx+math.cos(original[2])*dy,
                      original[2]+turn)
    h.backend.odom_pose = (1.+math.cos(odom_yaw)*dx-math.sin(odom_yaw)*dy,
                           -2.+math.sin(odom_yaw)*dx+math.cos(odom_yaw)*dy, odom_yaw+turn)
    h.backend.ranges = h.backend.raycast()
    original_score = h.supervisor.map_quality.score
    scored_laser_poses = []

    def record_score(*args, **kwargs):
        # The supervisor calls by keyword; the independent rival worker also
        # calls this scorer positionally and must retain its real behavior.
        if not args:
            scored_laser_poses.append((tuple(kwargs['laser_pose']), tuple(kwargs['ranges'])))
        return original_score(*args, **kwargs)

    monkeypatch.setattr(h.supervisor.map_quality, 'score', record_score)
    h.until(lambda: h.supervisor.state == 'READY', timeout=15.)
    seed = h.backend.initial_seeds[0]
    assert navigation_startup.planar(seed.pose.pose) == pytest.approx(h.backend.pose, abs=1e-8)
    assert any(np.allclose(pose, h.backend.laser_pose(), atol=1e-8)
               and np.allclose(ranges, h.backend.ranges) for pose, ranges in scored_laser_poses)
    metrics = h.supervisor.tim_metrics
    assert metrics['tim_compensation_translation'] == pytest.approx(.01)
    assert metrics['tim_compensation_yaw'] == pytest.approx(turn)
    assert metrics['tim_max_translation'] == pytest.approx(.01)
    assert metrics['tim_max_yaw'] == pytest.approx(turn)
    assert navigation_startup.stamp_ns(seed.header.stamp) == metrics['tim_seed_stamp_ns']
    assert metrics['tim_seed_stamp_ns'] > metrics['tim_input_stamp_ns']
    assert metrics['tim_motion_events'] == metrics['tim_goal_events'] == 0
    assert metrics['tim_rejection_reason'] is None
    assert h.supervisor.source == 'auto' and h.supervisor.metrics['rival_search_completed']


@pytest.mark.parametrize('interruption,expected', [
    ('translation_roundtrip', 'discarded_excursion'), ('yaw_roundtrip', 'discarded_excursion'),
    ('tiny_command', 'discarded_command'), ('goal_pulse', 'discarded_goal'),
    ('odom_gap', 'discarded_odom_gap'), ('scan_gap', 'discarded_scan_gap'),
    ('odom_bad_stamp', 'invalid_odom_stamp'), ('scan_bad_stamp', 'invalid_scan_stamp'),
    ('odom_bad_frame', 'invalid_odom'), ('odom_bad_child', 'invalid_odom'),
    ('odom_bad_pose', 'invalid_odom'), ('odom_bad_quaternion', 'invalid_odom'),
    ('scan_bad_metadata', 'invalid_scan'), ('scan_empty', 'invalid_scan'),
    ('scan_insufficient', 'invalid_scan'),
])
def test_tim_latches_transient_fault_before_recovery(harnesses, interruption, expected):
    h = harnesses(automatic=True, unique_map=True, tim_delay=1.5,
                  startup_parameters={'global_localization_backend': 'tim'})
    h.until(lambda: h.backend.tim_calls == 1)
    supervisor = h.supervisor
    # These events recover before a timer poll and before the remote response.
    # Checking only the final pose, status, speed or sensor age would accept it.
    if interruption.endswith('roundtrip'):
        original = copy.deepcopy(supervisor.odom)
        changed = copy.deepcopy(original)
        if interruption.startswith('translation'):
            changed.pose.pose.position.x += .021
        else:
            changed.pose.pose.orientation.z = math.sin(.011/2)
            changed.pose.pose.orientation.w = math.cos(.011/2)
        supervisor.on_odom(changed)
        supervisor.on_odom(original)
    elif interruption == 'tiny_command':
        command = Twist()
        command.linear.x = .0001
        supervisor.on_command(command)
        supervisor.on_command(Twist())
    elif interruption == 'goal_pulse':
        supervisor.on_navigation_status(String(data='RUNNING'))
        supervisor.on_navigation_status(String(data='IDLE'))
    else:
        name = interruption.split('_')[0]
        original, received = supervisor.sensor_history[name]
        original = copy.deepcopy(original)
        changed = copy.deepcopy(original)
        if interruption.endswith('_gap'):
            supervisor.sensor_history[name] = (original, received-supervisor.cfg['data_timeout']-1.)
        elif interruption.endswith('_bad_stamp'):
            changed.header.stamp.sec = 0
            changed.header.stamp.nanosec = 0
        elif interruption.endswith('_bad_frame'):
            changed.header.frame_id = 'unrelated'
        elif interruption.endswith('_bad_child'):
            changed.child_frame_id = 'unrelated'
        elif interruption.endswith('_bad_pose'):
            changed.pose.pose.position.x = float('nan')
        elif interruption == 'scan_bad_metadata':
            changed.angle_increment = float('nan')
        elif interruption == 'scan_empty':
            changed.ranges = []
        elif interruption == 'scan_insufficient':
            changed.ranges = [float('inf')] * len(changed.ranges)
        else:
            changed.pose.pose.orientation.w = 0.
        getattr(supervisor, 'on_'+name)(changed)
        getattr(supervisor, 'on_'+name)(original)
    supervisor.last_motion -= 1.
    h.until(lambda: supervisor.tim_metrics['tim_status'] == expected)
    h.until(lambda: h.backend.tim_completed == 1)
    h.spin(.3)
    assert not h.backend.initial_seeds and not h.backend.start_commands
    assert supervisor.tim_metrics['tim_rejection_reason'] == expected
    json.dumps(supervisor.tim_metrics, allow_nan=False)
    if interruption == 'translation_roundtrip':
        assert supervisor.tim_metrics['tim_max_translation'] == pytest.approx(.021)
    elif interruption == 'yaw_roundtrip':
        assert supervisor.tim_metrics['tim_max_yaw'] == pytest.approx(.011)
    elif interruption == 'goal_pulse':
        assert supervisor.tim_metrics['tim_goal_events'] == 1


@pytest.mark.parametrize('interruption', ['manual', 'map', 'motion', 'move_then_stop', 'goal', 'stale'])
def test_tim_late_result_cannot_overwrite_interrupted_request(harnesses, interruption):
    h = harnesses(automatic=True, unique_map=True, tim_delay=2.,
                  startup_parameters={'global_localization_backend': 'tim'})
    h.until(lambda: h.backend.tim_calls == 1)
    generation = h.supervisor.generation
    if interruption == 'manual':
        h.set_initial_pose()
        h.until(lambda: len(h.backend.initial_seeds) == 1)
        assert h.supervisor.generation > generation
    elif interruption == 'map':
        changed = copy.deepcopy(h.backend.map_message)
        changed.data[0] = 100
        h.backend.map_pub.publish(changed)
        h.until(lambda: h.supervisor.generation > generation)
    elif interruption == 'motion':
        h.backend.speed = .1
        h.until(lambda: h.supervisor.tim_metrics['tim_status'] == 'discarded_motion')
        h.backend.speed = 0.
        h.until(h.supervisor.stationary)
    elif interruption == 'move_then_stop':
        # Motion can occur entirely between supervisor polls. Receiving a
        # stopped robot at completion must not legitimize the old scan.
        moving = Twist()
        moving.linear.x = .1
        h.supervisor.on_command(moving)
        h.supervisor.on_command(Twist())
        h.supervisor.last_motion -= 1.
        assert h.supervisor.stationary()
    elif interruption == 'goal':
        h.supervisor.on_navigation_status(String(data='RUNNING'))
        h.until(lambda: h.supervisor.tim_metrics['tim_status'] == 'discarded_goal')
        h.supervisor.on_navigation_status(String(data='IDLE'))
    else:
        h.backend.emit_sensors = False
    h.until(lambda: h.backend.tim_completed == 1)
    h.spin(.8)
    assert h.backend.global_calls == 0
    assert len(h.backend.initial_seeds) == (1 if interruption == 'manual' else 0)
    if interruption == 'manual':
        assert h.supervisor.source == 'manual'
        h.until(lambda: h.supervisor.state == 'READY')
    else:
        assert not h.backend.start_commands
        assert h.supervisor.state != 'READY'


def test_tim_timeout_does_not_poison_subsequent_manual_initialization(harnesses):
    h = harnesses(automatic=True, unique_map=True, tim_delay=2., startup_parameters={
        'global_localization_backend': 'tim', 'tim_timeout_sec': .7, 'service_timeout': 5.,
    })
    h.until(lambda: h.supervisor.tim_metrics['tim_status'] == 'timeout')
    assert not h.supervisor.reset_uncertain
    assert not h.backend.initial_seeds and h.backend.global_calls == 0
    h.ready()
    h.until(lambda: h.backend.tim_completed == 1)
    h.spin(.6)
    assert h.supervisor.source == 'manual' and len(h.backend.initial_seeds) == 1
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]


def test_tim_does_not_require_expired_request_timestamp_tf_after_inference(harnesses, monkeypatch):
    h = harnesses(automatic=True, unique_map=True, tim_delay=1.5,
                  laser_extrinsic=(.24, -.13, .37),
                  startup_parameters={'global_localization_backend': 'tim'})
    h.until(lambda: h.backend.tim_calls == 1)
    captured = navigation_startup.stamp_ns(h.backend.tim_requests[0].scan.header.stamp)
    original_lookup = h.supervisor.tf_buffer.lookup_transform

    def lookup_without_old_history(target, source, when, *args, **kwargs):
        assert when.nanoseconds != captured, 'TIM completion must use the captured scan-time transforms'
        return original_lookup(target, source, when, *args, **kwargs)

    monkeypatch.setattr(h.supervisor.tf_buffer, 'lookup_transform', lookup_without_old_history)
    h.until(lambda: h.supervisor.state == 'READY', timeout=15.)
    assert navigation_startup.planar(h.backend.initial_seeds[0].pose.pose) == pytest.approx(h.backend.pose)


@pytest.mark.parametrize('result', ['failure', 'empty', 'nan_score', 'nan_pose',
                                   'wrong_frame', 'wrong_stamp', 'low_quality'])
def test_tim_failed_or_invalid_response_never_seeds_amcl(harnesses, result):
    h = harnesses(automatic=True, unique_map=True, tim_result=result,
                  startup_parameters={'global_localization_backend': 'tim'})
    h.until(lambda: h.backend.tim_completed == 1)
    h.until(lambda: h.supervisor.state == 'WAITING_INITIAL_POSE')
    h.spin(.6)
    assert not h.backend.initial_seeds and not h.backend.start_commands
    assert h.backend.global_calls == 0


def test_tim_retains_independent_repeated_corridor_rejection(harnesses):
    h = harnesses(automatic=True, startup_parameters={'global_localization_backend': 'tim'})
    h.until(lambda: h.backend.tim_completed == 1)
    h.until(lambda: h.supervisor.state == 'WAITING_INITIAL_POSE', timeout=15.)
    assert h.supervisor.source == 'auto'
    assert h.supervisor.metrics['rival_search_completed']
    assert h.supervisor.metrics['rival_score'] >= .9
    assert not h.backend.start_commands


def test_tim_backend_never_silently_falls_back_to_available_amcl_reset(harnesses):
    h = harnesses(automatic=True, unique_map=True, tim_backend=False,
                  startup_parameters={'global_localization_backend': 'tim'})
    h.until(lambda: h.supervisor.data_problem() is None)
    h.spin(1.2)
    assert h.backend.global_calls == 0
    assert not h.backend.start_commands


def test_real_amcl_manual_initialization_and_nomotion_updates(harnesses, tmp_path):
    """Exercise actual AMCL lifecycle, map QoS, scan TF, poses, and particles."""
    h = harnesses(real_amcl=True, map_tf=False, scan_count=3240, startup_parameters={
        'search_timeout': 15., 'pose_timeout': 2., 'data_timeout': 1.5,
        'initialization_max_beams': 1080,
        'verification_duration': 1.,
        'manual_verification_duration': 1.,
    })
    config = Path(__file__).resolve().parents[1] / 'config' / 'localization.yaml'
    log_path = tmp_path / 'amcl.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([
            '/opt/ros/jazzy/lib/nav2_amcl/amcl', '--ros-args', '--params-file', str(config),
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            state_client = h.probe.create_client(ChangeState, '/amcl/change_state')
            h.until(state_client.service_is_ready)
            for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                request = ChangeState.Request()
                request.transition.id = transition
                future = state_client.call_async(request)
                h.until(future.done)
                assert future.result().success, log_path.read_text()
            h.until(lambda: h.supervisor.lifecycle_active('amcl'))
            parameter_client = h.probe.create_client(GetParameters, '/amcl/get_parameters')
            h.until(parameter_client.service_is_ready)
            initial_parameters = parameter_client.call_async(GetParameters.Request(names=['max_beams']))
            h.until(initial_parameters.done)
            assert initial_parameters.result().values[0].integer_value == 1080
            h.until(lambda: h.initial_pub.get_subscription_count() >= 2)
            h.set_initial_pose()
            h.until(lambda: h.supervisor.state == 'READY', timeout=18.)
            assert process.poll() is None, log_path.read_text()
            assert h.probe.count_publishers('/amcl_pose') == 1
            assert len(h.supervisor.scan.ranges) == 3240
            assert h.supervisor.metrics['valid_beams'] == 360
            assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]
            tracking_parameters = parameter_client.call_async(GetParameters.Request(names=['max_beams']))
            h.until(tracking_parameters.done)
            assert tracking_parameters.result().values[0].integer_value == 60
            h.until(lambda: h.supervisor.cloud is not None)
            assert h.supervisor.cloud.header.frame_id == 'map'
            assert h.supervisor.cloud.particles
            first_stamp = navigation_startup.stamp_ns(h.supervisor.pose.header.stamp)
            # The synthetic robot is stationary throughout: a newer AMCL pose
            # verifies interoperability with /request_nomotion_update.
            h.until(lambda: navigation_startup.stamp_ns(h.supervisor.pose.header.stamp) > first_stamp,
                    timeout=4.)
            assert h.supervisor.state == 'READY'
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)


def offset_pose(message, dx, dy, dyaw):
    pose = copy.deepcopy(message)
    x, y, theta = navigation_startup.planar(pose.pose.pose)
    pose.pose.pose.position.x, pose.pose.pose.position.y = x + dx, y + dy
    pose.pose.pose.orientation.z = math.sin((theta + dyaw) / 2)
    pose.pose.pose.orientation.w = math.cos((theta + dyaw) / 2)
    pose.pose.covariance[0] = pose.pose.covariance[7] = .25
    pose.pose.covariance[35] = .0685
    return pose


def pose_error(message, expected):
    x, y, theta = navigation_startup.planar(message.pose.pose)
    return (math.hypot(x - expected[0], y - expected[1]),
            abs(math.atan2(math.sin(theta - expected[2]), math.cos(theta - expected[2]))))


def test_manual_click_is_scan_matched_before_verification(harnesses):
    h = harnesses(unique_map=True, scan_count=360, laser_extrinsic=(.1, 0., math.pi),
                  startup_parameters={'manual_refinement': True})
    seeds = []
    h.probe.create_subscription(PoseWithCovarianceStamped, '/initialpose', seeds.append, 10)
    h.until(lambda: h.supervisor.state == 'WAITING_INITIAL_POSE')
    h.initial_pub.publish(offset_pose(h.backend.estimated_pose(), .4, -.3, .25))
    h.until(lambda: h.supervisor.source == 'manual')
    h.backend.emit_pose = True
    h.until(lambda: h.supervisor.state == 'READY')
    assert len(seeds) == 2
    distance, heading = pose_error(seeds[1], h.backend.pose)
    assert distance < .06 and heading < .03
    assert seeds[1].pose.covariance[0] == .01
    metrics = h.statuses[-1]
    assert metrics['refinement_accepted'] is True
    assert metrics['refinement_match_ratio'] > .9 > metrics['refinement_hint_match_ratio']


def test_manual_refinement_below_threshold_keeps_the_operator_pose(harnesses):
    h = harnesses(unique_map=True, startup_parameters={'manual_refinement': True})
    seeds = []
    h.probe.create_subscription(PoseWithCovarianceStamped, '/initialpose', seeds.append, 10)
    h.until(lambda: h.supervisor.state == 'WAITING_INITIAL_POSE')
    # Returns that fit no wall anywhere in the window.
    h.backend.ranges = [.2] * len(h.backend.ranges)
    h.initial_pub.publish(offset_pose(h.backend.estimated_pose(), .4, -.3, .25))
    h.until(lambda: h.supervisor.refinement_metrics)
    h.spin(.3)
    assert len(seeds) == 1
    assert h.supervisor.refinement_metrics['refinement_accepted'] is False
    assert h.supervisor.source == 'manual' and h.supervisor.refinement is None


def test_manual_refinement_is_discarded_when_the_robot_moves(harnesses, monkeypatch):
    h = harnesses(unique_map=True, startup_parameters={'manual_refinement': True})
    seeds = []
    h.probe.create_subscription(PoseWithCovarianceStamped, '/initialpose', seeds.append, 10)
    h.until(lambda: h.supervisor.state == 'WAITING_INITIAL_POSE')
    original = h.supervisor.map_quality.refine_pose

    def slow(*args, **kwargs):
        time.sleep(.5)
        return original(*args, **kwargs)

    monkeypatch.setattr(h.supervisor.map_quality, 'refine_pose', slow)
    h.initial_pub.publish(offset_pose(h.backend.estimated_pose(), .4, -.3, .25))
    h.until(lambda: h.supervisor.refinement is not None and h.supervisor.refinement['future'] is not None)
    h.supervisor.record_motion()
    h.until(lambda: h.supervisor.refinement is None)
    h.spin(.2)
    assert len(seeds) == 1


def test_ready_stopped_pose_is_saved_per_map_and_offered_in_status(harnesses, tmp_path):
    path = tmp_path / 'nested' / 'last_pose.json'
    h = harnesses(startup_parameters={'last_pose_file': str(path)})
    h.until(lambda: h.supervisor.state == 'WAITING_INITIAL_POSE')
    h.spin(.3)
    assert not path.exists() and h.statuses[-1]['last_pose'] is None
    h.ready()
    h.until(path.exists)
    saved = json.loads(path.read_text())
    assert list(saved) == [h.supervisor.map_id]
    entry = saved[h.supervisor.map_id]
    assert math.hypot(entry['x'] - h.backend.pose[0], entry['y'] - h.backend.pose[1]) < 1e-6
    assert abs(entry['yaw'] - h.backend.pose[2]) < 1e-6
    h.until(lambda: h.statuses[-1]['last_pose'] == entry)
    assert h.supervisor.load_saved_poses() == saved
    path.write_text('{"bad": {"x": "nan"}, "other": [1]}')
    assert h.supervisor.load_saved_poses() == {}


def test_last_pose_is_not_saved_while_moving(harnesses, tmp_path):
    path = tmp_path / 'last_pose.json'
    h = harnesses(startup_parameters={'last_pose_file': str(path)})
    h.ready()
    h.backend.speed = .2
    h.supervisor.saved_poses = {}
    if path.exists():
        path.unlink()
    h.spin(2.5)
    assert not path.exists()


def test_real_amcl_converges_from_a_rough_click_with_refinement(harnesses, tmp_path):
    h = harnesses(real_amcl=True, map_tf=False, unique_map=True, scan_count=3240, startup_parameters={
        'search_timeout': 15., 'pose_timeout': 2., 'data_timeout': 1.5,
        'initialization_max_beams': 1080, 'manual_verification_duration': 1.,
        'manual_refinement': True,
    })
    config = Path(__file__).resolve().parents[1] / 'config' / 'localization.yaml'
    log_path = tmp_path / 'amcl.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([
            '/opt/ros/jazzy/lib/nav2_amcl/amcl', '--ros-args', '--params-file', str(config),
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            state_client = h.probe.create_client(ChangeState, '/amcl/change_state')
            h.until(state_client.service_is_ready)
            for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                request = ChangeState.Request()
                request.transition.id = transition
                future = state_client.call_async(request)
                h.until(future.done)
                assert future.result().success, log_path.read_text()
            h.until(lambda: h.supervisor.lifecycle_active('amcl'))
            h.until(lambda: h.initial_pub.get_subscription_count() >= 2)
            h.initial_pub.publish(offset_pose(h.backend.estimated_pose(), .4, -.3, .25))
            h.until(lambda: h.supervisor.source == 'manual')
            h.until(lambda: h.supervisor.state == 'READY', timeout=18.)
            assert h.supervisor.refinement_metrics['refinement_accepted'] is True
            distance, heading = pose_error(h.supervisor.pose, h.backend.pose)
            assert distance < .1 and heading < .05, log_path.read_text()
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)


def test_manual_fresh_scans_cannot_reuse_one_amcl_pose_stamp(harnesses, monkeypatch):
    h = harnesses(startup_parameters={
        'manual_verification_duration': .1, 'manual_verification_samples': 4,
        'pose_timeout': 3.,
    })
    h.set_initial_pose()
    h.backend.emit_pose = False
    h.until(lambda: h.supervisor.applied_beams == 360
            and h.supervisor.beam_confirmed_revision == h.supervisor.beam_revision)
    h.until(lambda: navigation_startup.stamp_ns(h.backend.last_scan.header.stamp)
            > h.supervisor.attempt_stamp)
    estimated_pose = h.backend.estimated_pose
    frozen_pose = estimated_pose(h.backend.last_scan.header.stamp)
    frozen_stamp = navigation_startup.stamp_ns(frozen_pose.header.stamp)
    monkeypatch.setattr(h.backend, 'estimated_pose', lambda stamp=None: copy.deepcopy(frozen_pose))
    h.backend.emit_pose = True
    h.until(lambda: h.supervisor.good_count == 1)
    first_scan = navigation_startup.stamp_ns(h.supervisor.scan.header.stamp)
    first_reception = h.supervisor.received['pose']
    h.spin(.6)
    # Both topic callbacks remain live, but replaying one AMCL observation
    # cannot supply four independent post-initialization observation pairs.
    assert h.supervisor.received['pose'] > first_reception
    assert navigation_startup.stamp_ns(h.supervisor.scan.header.stamp) > first_scan
    assert navigation_startup.stamp_ns(h.supervisor.pose.header.stamp) == frozen_stamp
    assert h.supervisor.pose_evidence()[1] is None
    assert h.supervisor.good_count == 1
    assert h.supervisor.beam_phase == 'initialization'
    assert not h.backend.start_commands
    monkeypatch.setattr(h.backend, 'estimated_pose', estimated_pose)
    h.until(lambda: h.supervisor.state == 'READY')


def test_manual_short_window_requires_four_new_pairs_after_each_beam_confirmation(harnesses):
    h = harnesses(startup_parameters={
        'verification_duration': 2.5, 'verification_samples': 6,
        'manual_verification_duration': .15, 'manual_verification_samples': 4,
        'pose_timeout': 3.,
    })
    h.set_initial_pose()
    h.backend.emit_pose = False
    pair_events = []
    for phase, beams in (('initialization', 360), ('tracking', 60)):
        h.until(lambda: h.supervisor.beam_phase == phase
                and h.supervisor.applied_beams == beams
                and h.supervisor.beam_confirmed_revision == h.supervisor.beam_revision)
        confirmation = next(event for event in reversed(h.backend.events)
                            if event['kind'] == 'get_confirmed' and event['value'] == beams)
        assert h.supervisor.good_count == 0
        assert h.supervisor.attempt_stamp >= confirmation['stamp']
        assert h.supervisor.verification_requirements() == (.15, 4)
        last_stamp = h.supervisor.attempt_stamp
        for index in range(1, 5):
            h.until(lambda: navigation_startup.stamp_ns(h.backend.last_scan.header.stamp) > last_stamp)
            pose = h.backend.estimated_pose(h.backend.last_scan.header.stamp)
            last_stamp = navigation_startup.stamp_ns(pose.header.stamp)
            h.backend.event('manual_pair', index)
            event = h.backend.events[-1]
            event.update(phase=phase, pose_stamp=last_stamp, scan_stamp=last_stamp)
            pair_events.append(event)
            h.backend.pose_pub.publish(pose)
            if index < 4:
                h.until(lambda: h.supervisor.good_count == index)
                # The duration alone is insufficient: fresh scans continue,
                # while only this many distinct AMCL estimates have arrived.
                h.spin(.2)
                assert h.supervisor.good_count == index
                assert h.supervisor.beam_phase == phase
                assert not h.backend.start_commands
            elif phase == 'initialization':
                h.until(lambda: h.supervisor.beam_phase == 'tracking')
                assert not h.backend.start_commands
            else:
                h.until(lambda: bool(h.backend.start_commands))
        assert all(event['pose_stamp'] > confirmation['stamp']
                   and event['scan_stamp'] > confirmation['stamp']
                   for event in pair_events if event['phase'] == phase)
    h.backend.emit_pose = True
    h.until(lambda: h.supervisor.state == 'READY')
    events = h.backend.events
    first_confirmation = next(i for i, event in enumerate(events)
                              if event['kind'] == 'get_confirmed' and event['value'] == 360)
    tracking_confirmation = next(i for i, event in enumerate(events)
                                 if event['kind'] == 'get_confirmed' and event['value'] == 60)
    startup = next(i for i, event in enumerate(events) if event['kind'] == 'startup')
    initial_pairs = [i for i, event in enumerate(events)
                     if event['kind'] == 'manual_pair' and event['phase'] == 'initialization']
    tracking_pairs = [i for i, event in enumerate(events)
                      if event['kind'] == 'manual_pair' and event['phase'] == 'tracking']
    assert len(initial_pairs) == len(tracking_pairs) == 4
    assert first_confirmation < min(initial_pairs) <= max(initial_pairs) < tracking_confirmation
    assert tracking_confirmation < min(tracking_pairs) <= max(tracking_pairs) < startup
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]


def test_manual_verification_bounds_gradual_drift_from_a_fixed_anchor(harnesses):
    h = harnesses(startup_parameters={
        'manual_verification_duration': .6, 'manual_verification_samples': 4,
    })
    h.set_initial_pose()
    h.until(lambda: h.supervisor.good_since is not None)
    first_window = h.supervisor.good_since
    initial_pose = h.backend.pose
    # Each 10 cm correction is below the 25 cm stability radius. Together
    # they exceed it before the verification window can finish. Re-render
    # scans with each changed map pose so rejection is about unstable pose,
    # not a deliberately inconsistent scan or stale transform.
    for index in range(1, 9):
        h.backend.pose = (initial_pose[0] + .1*index, initial_pose[1], initial_pose[2])
        h.backend.ranges = h.backend.raycast()
        h.spin(.16)
        assert h.supervisor.pose_evidence()[1] is None
        assert h.supervisor.beam_phase == 'initialization'
        assert not h.backend.start_commands
        if index >= 3:
            assert h.supervisor.good_since > first_window
    # Once the estimate stops drifting, the same authentic sensor stream
    # must be able to complete both manual verification phases normally.
    h.until(lambda: h.supervisor.state == 'READY')
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]


def _ready_tracking_fixture(harnesses, monkeypatch, **parameters):
    """Complete genuine automatic initialization before faulting its local score."""
    from dataclasses import replace

    overrides = dict(tracking_verification_duration=.15, tracking_verification_samples=4,
                     pose_timeout=3.)
    overrides.update(parameters)
    h = harnesses(automatic=True, unique_map=True, startup_parameters=overrides)
    h.until(lambda: h.supervisor.state == 'READY')
    h.until(lambda: h.statuses and h.statuses[-1]['ready'])
    h.spin(.12)
    assert h.supervisor.prior_verified and h.supervisor.source == 'auto'
    assert h.backend.global_calls == h.backend.global_completed == 1
    saved = SimpleNamespace(generation=h.supervisor.generation,
                            event_index=len(h.backend.events),
                            global_calls=h.backend.global_calls, tim_calls=h.backend.tim_calls)
    faults = SimpleNamespace(quality={}, scored=[], rivals=[])
    original_score = h.supervisor.map_quality.score
    original_rivals = h.supervisor.check_rivals

    def score(*args, **kwargs):
        result = original_score(*args, **kwargs)
        faults.scored.append((navigation_startup.stamp_ns(h.supervisor.scan.header.stamp),
                              dict(faults.quality)))
        return replace(result, **faults.quality)

    def check_rivals(*args, **kwargs):
        faults.rivals.append(args)
        return original_rivals(*args, **kwargs)

    # Patch this already-initialized map instance, not MapQuality globally:
    # startup/global ambiguity checks must still use real synthetic geometry.
    monkeypatch.setattr(h.supervisor.map_quality, 'score', score)
    monkeypatch.setattr(h.supervisor, 'check_rivals', check_rivals)
    h.statuses.clear()
    return h, faults, saved


def _assert_tracking_prior_retained(h, faults, saved):
    assert h.supervisor.prior_verified
    assert h.supervisor.generation == saved.generation
    assert h.supervisor.source == 'auto'
    assert h.supervisor.beam_phase == 'tracking' and h.backend.max_beams == 60
    assert h.backend.global_calls == saved.global_calls
    assert h.backend.tim_calls == saved.tim_calls
    assert not faults.rivals
    assert not any(event['kind'] in ('set_requested', 'global_reset', 'tim_request')
                   for event in h.backend.events[saved.event_index:])
    assert h.backend.start_commands == [ManageLifecycleNodes.Request.STARTUP]


def _freeze_one_tracking_scan(h, faults):
    h.backend.frozen_scan = copy.deepcopy(h.supervisor.scan)
    # Drain ordinary scans already queued before enabling replay. Otherwise a
    # final in-flight newer scan could legitimately add a second bad sample.
    h.spin(.12)
    frozen_stamp = navigation_startup.stamp_ns(h.backend.frozen_scan.header.stamp)
    h.until(lambda: navigation_startup.stamp_ns(h.supervisor.scan_samples[-1][0].header.stamp)
            == frozen_stamp)
    faults.scored.clear()


@pytest.mark.parametrize('quality_field', ['match_ratio', 'known_ratio'])
def test_tracking_transient_soft_mismatch_preserves_ready_and_verified_prior(
        harnesses, monkeypatch, quality_field):
    h, faults, saved = _ready_tracking_fixture(harnesses, monkeypatch)
    _freeze_one_tracking_scan(h, faults)
    faults.quality[quality_field] = .1
    h.supervisor.tick()
    assert h.supervisor.state == 'READY'
    h.spin(.12)
    assert h.supervisor.state == 'READY'
    assert len({stamp for stamp, quality in faults.scored if quality}) == 1
    faults.quality.clear()
    h.backend.frozen_scan = None
    h.spin(.65)
    assert h.supervisor.state == 'READY'
    assert h.statuses and all(status['ready'] for status in h.statuses)
    _assert_tracking_prior_retained(h, faults, saved)


@pytest.mark.parametrize('offset,expect_ready', [(.7, True), (1.5, False)])
def test_tracking_bounded_amcl_tf_jump_is_a_warning_and_a_large_one_closes_the_gate(
        harnesses, monkeypatch, offset, expect_ready):
    h, faults, saved = _ready_tracking_fixture(harnesses, monkeypatch)
    original = h.backend.estimated_pose

    def shifted(stamp=None):
        # The published pose steps ahead of the TF the backend keeps broadcasting.
        message = original(stamp)
        message.pose.pose.position.x += offset
        return message

    monkeypatch.setattr(h.backend, 'estimated_pose', shifted)
    h.spin(.6)
    assert (h.supervisor.state == 'READY') is expect_ready
    if expect_ready:
        assert h.supervisor.tracking_warning and '지도 좌표 변환' in h.supervisor.tracking_warning
        assert h.statuses[-1]['ready']
    else:
        assert '일치하지 않습니다' in h.supervisor.detail
    monkeypatch.setattr(h.backend, 'estimated_pose', original)
    h.until(lambda: h.supervisor.state == 'READY' and not h.supervisor.tracking_warning, timeout=6.)
    _assert_tracking_prior_retained(h, faults, saved)


def test_tracking_corridor_covariance_below_the_tracking_limit_raises_no_warning(harnesses, monkeypatch):
    h, faults, saved = _ready_tracking_fixture(harnesses, monkeypatch)
    original_pose = h.backend.estimated_pose

    def corridor_pose(stamp=None):
        pose = original_pose(stamp)
        pose.pose.covariance[0] = pose.pose.covariance[7] = .39 ** 2  # above the 0.35 m admission limit
        return pose

    monkeypatch.setattr(h.backend, 'estimated_pose', corridor_pose)
    h.spin(.8)
    assert h.supervisor.state == 'READY' and not h.supervisor.tracking_warning
    assert h.supervisor.metrics['xy_std'] == pytest.approx(.39, abs=.01)
    _assert_tracking_prior_retained(h, faults, saved)


@pytest.mark.parametrize('warning', ['match_ratio', 'known_ratio', 'xy_covariance', 'yaw_covariance'])
def test_tracking_initial_confidence_limits_are_diagnostics_after_ready(
        harnesses, monkeypatch, warning):
    h, faults, saved = _ready_tracking_fixture(harnesses, monkeypatch)
    h.backend.speed = .1
    h.until(lambda: not h.supervisor.stationary())
    original_pose = h.backend.estimated_pose

    def estimated_pose(stamp=None):
        pose = original_pose(stamp)
        if warning == 'xy_covariance':
            # Just past the tracking limit (0.6 m); the former 0.352 m crossing
            # that aborted the closed-loop Isaac run is now inside it.
            pose.pose.covariance[0] = pose.pose.covariance[7] = .61 ** 2
        elif warning == 'yaw_covariance':
            pose.pose.covariance[35] = .8
        return pose

    monkeypatch.setattr(h.backend, 'estimated_pose', estimated_pose)
    if warning in ('match_ratio', 'known_ratio'):
        faults.quality[warning] = .1
    h.until(lambda: bool(h.supervisor.tracking_warning))
    # Exceed both former cancellation limits (3 distinct scans / 0.5 seconds).
    h.spin(1.1)
    assert h.supervisor.state == 'READY'
    assert h.supervisor.tracking_bad_count > 3
    assert time.monotonic() - h.supervisor.tracking_bad_since > .5
    assert h.statuses and all(status['ready'] for status in h.statuses)
    _assert_tracking_prior_retained(h, faults, saved)
    faults.quality.clear()
    monkeypatch.setattr(h.backend, 'estimated_pose', original_pose)
    h.until(lambda: not h.supervisor.tracking_warning)
    assert h.supervisor.state == 'READY'
    _assert_tracking_prior_retained(h, faults, saved)


def test_tracking_invalid_geometry_stops_then_requires_local_fresh_pairs(
        harnesses, monkeypatch):
    h, faults, saved = _ready_tracking_fixture(harnesses, monkeypatch)
    h.backend.speed = .1
    h.until(lambda: not h.supervisor.stationary())
    # Prevent a queued AMCL publication from supplying post-stop evidence.
    h.backend.emit_pose = False
    faults.quality['origin_free'] = False
    h.until(lambda: h.supervisor.state == 'VERIFYING', timeout=1.5)
    h.until(lambda: h.statuses and not h.statuses[-1]['ready'])
    recovery_stamp = h.supervisor.attempt_stamp
    _assert_tracking_prior_retained(h, faults, saved)

    faults.quality.clear()
    h.spin(.25)
    assert h.supervisor.state != 'READY'
    assert not h.supervisor.stationary()
    # A post-fault pose still measured while moving cannot be the first of
    # the four stopped recovery observations.
    h.backend.pose_pub.publish(h.backend.estimated_pose(h.backend.last_scan.header.stamp))
    h.until(lambda: h.supervisor.pose is not None)
    h.spin(.12)
    h.backend.speed = 0.
    h.until(h.supervisor.stationary)
    h.spin(.2)
    assert h.supervisor.state == 'VERIFYING'
    assert h.supervisor.good_count == 0

    last_stamp = recovery_stamp
    for index in range(1, 5):
        h.until(lambda: navigation_startup.stamp_ns(h.backend.last_scan.header.stamp) > last_stamp)
        pose = h.backend.estimated_pose(h.backend.last_scan.header.stamp)
        last_stamp = navigation_startup.stamp_ns(pose.header.stamp)
        h.backend.pose_pub.publish(pose)
        if index < 4:
            h.until(lambda: h.supervisor.good_count == index)
            h.spin(.18)
            assert h.supervisor.good_count == index
            assert h.supervisor.state == 'VERIFYING'
        else:
            h.until(lambda: h.supervisor.state == 'READY')
    h.backend.emit_pose = True
    assert last_stamp > recovery_stamp
    _assert_tracking_prior_retained(h, faults, saved)


@pytest.mark.parametrize('fault', ['nan_covariance', 'negative_covariance', 'zero_covariance',
                                   'tf', 'stale_data', 'lifecycle',
                                   'origin', 'valid_beams'])
def test_tracking_soft_warning_never_delays_a_hard_fault(harnesses, monkeypatch, fault):
    h, faults, saved = _ready_tracking_fixture(harnesses, monkeypatch)
    _freeze_one_tracking_scan(h, faults)
    faults.quality['match_ratio'] = .1
    h.supervisor.tick()
    assert h.supervisor.state == 'READY'
    if fault == 'nan_covariance':
        h.supervisor.pose.pose.covariance[35] = float('nan')
    elif fault == 'negative_covariance':
        h.supervisor.pose.pose.covariance[35] = -.1
    elif fault == 'zero_covariance':
        h.supervisor.pose.pose.covariance = [0.] * 36
    elif fault == 'tf':
        original_lookup = h.supervisor.tf_buffer.lookup_transform

        def missing_map_transform(target, source, *args, **kwargs):
            if target == 'map':
                raise navigation_startup.TransformException('Injected map TF loss')
            return original_lookup(target, source, *args, **kwargs)

        monkeypatch.setattr(h.supervisor.tf_buffer, 'lookup_transform', missing_map_transform)
    elif fault == 'stale_data':
        h.backend.emit_sensors = False
        h.until(lambda: not h.supervisor.fresh('scan', h.supervisor.cfg['data_timeout']), timeout=1.5)
    elif fault == 'lifecycle':
        original_active = h.supervisor.lifecycle_active
        monkeypatch.setattr(h.supervisor, 'lifecycle_active',
                            lambda name: name != 'controller_server' and original_active(name))
    elif fault == 'origin':
        faults.quality['origin_free'] = False
    else:
        faults.quality['valid_beams'] = 0
    if fault in ('origin', 'valid_beams'):
        # Scan-derived evidence is cached per scan/pose pair while READY; these
        # faults can only arrive with a new scan, so release the frozen one.
        frozen_stamp = navigation_startup.stamp_ns(h.backend.frozen_scan.header.stamp)
        h.backend.frozen_scan = None
        h.until(lambda: navigation_startup.stamp_ns(h.supervisor.scan_samples[-1][0].header.stamp) > frozen_stamp)
        h.until(lambda: h.supervisor.state != 'READY')
    h.supervisor.tick()
    assert h.supervisor.state != 'READY'
    h.until(lambda: h.statuses and not h.statuses[-1]['ready'])
    assert not faults.rivals
    assert h.backend.global_calls == saved.global_calls
    assert h.backend.tim_calls == saved.tim_calls


def test_tracking_repeated_bad_scan_expires_only_at_sensor_watchdog(harnesses, monkeypatch):
    h, faults, saved = _ready_tracking_fixture(harnesses, monkeypatch, data_timeout=2.)
    _freeze_one_tracking_scan(h, faults)
    faults.quality['match_ratio'] = .1
    h.supervisor.tick()
    assert h.supervisor.state == 'READY'
    h.spin(.2)
    assert h.supervisor.state == 'READY'
    # A repeated measurement is one warning sample, never a timed quality
    # failure. Its measurement timestamp must still expire at the data watchdog.
    h.spin(.5)
    assert h.supervisor.state == 'READY'
    assert h.supervisor.tracking_bad_count == 1
    assert len({stamp for stamp, quality in faults.scored if quality}) == 1
    h.until(lambda: h.supervisor.state == 'WAITING_DATA', timeout=2.)
    assert '시간 정보' in h.supervisor.detail
    _assert_tracking_prior_retained(h, faults, saved)


@pytest.mark.parametrize('trigger', ['manual', 'retry', 'map'])
def test_tracking_verified_prior_is_cleared_by_explicit_reinitialization(
        harnesses, monkeypatch, trigger):
    h, faults, saved = _ready_tracking_fixture(harnesses, monkeypatch)
    h.backend.emit_pose = False
    if trigger == 'manual':
        h.initial_pub.publish(h.backend.estimated_pose())
    elif trigger == 'retry':
        h.until(h.retry.service_is_ready)
        response = h.retry.call_async(Trigger.Request())
        h.until(response.done)
        assert response.result().success
    else:
        changed = copy.deepcopy(h.backend.map_message)
        changed.data[0] = 100
        h.backend.map_pub.publish(changed)
    h.until(lambda: h.supervisor.generation > saved.generation)
    assert not h.supervisor.prior_verified
    assert h.supervisor.state != 'READY'
    assert h.supervisor.beam_phase == 'initialization'


def test_rolling_scan_waits_for_end_time_tf_using_only_fresh_history(harnesses):
    h = harnesses()
    h.ready()
    h.backend.emit_sensors = False
    h.spin(.08)
    original = copy.deepcopy(h.supervisor.scan)
    rolling = copy.deepcopy(original)
    rolling.time_increment = .2/(len(rolling.ranges)-1)
    end_ns = navigation_startup.stamp_ns(rolling.header.stamp) + round(.2e9)
    assert h.supervisor.tf_buffer.can_transform('odom', 'laser', Time.from_msg(rolling.header.stamp))
    assert not h.supervisor.tf_buffer.can_transform('odom', 'laser', Time(nanoseconds=end_ns))
    h.backend.scan_pub.publish(rolling)
    h.until(lambda: h.supervisor.scan_samples[-1][0].time_increment > 0)
    h.supervisor.tick()
    assert h.supervisor.state == 'READY'
    assert h.supervisor.scan.time_increment == 0.
    # This is a real previously measured complete scan, never an extrapolated
    # rolling scan. It must still expire under the existing freshness timeout.
    h.until(lambda: h.supervisor.state == 'WAITING_DATA', timeout=3.)
    assert not h.supervisor.fresh('scan', h.supervisor.cfg['data_timeout'])


@pytest.mark.parametrize('fault', ['nan_time', 'negative_time', 'long_time', 'no_returns'])
def test_invalid_latest_scan_cannot_be_hidden_by_valid_history(harnesses, fault):
    h = harnesses()
    h.ready()
    h.backend.emit_sensors = False
    h.spin(.08)
    invalid = copy.deepcopy(h.supervisor.scan)
    if fault == 'no_returns':
        invalid.ranges = [float('inf')] * len(invalid.ranges)
    else:
        invalid.time_increment = dict(nan_time=float('nan'), negative_time=-.001,
                                      long_time=.1)[fault]
    h.backend.scan_pub.publish(invalid)
    h.until(lambda: not h.supervisor.valid_scan(h.supervisor.scan_samples[-1][0]))
    h.supervisor.tick()
    assert h.supervisor.state == 'WAITING_DATA'
    assert h.supervisor.fresh('scan', h.supervisor.cfg['data_timeout'])
    assert '스캔 형식' in h.supervisor.detail


def test_verified_tracking_range_does_not_reduce_global_initialization_range(harnesses):
    h = harnesses(startup_parameters={'tracking_quality_max_range': 5.0})
    assert h.supervisor.quality_range() == 30.
    h.ready()
    assert h.supervisor.quality_range() == 5.
    h.until(lambda: h.statuses and h.statuses[-1]['quality_max_range'] == 5.)
    # A new operator seed must restore global-range verification until that
    # new prior has independently passed the original startup checks.
    h.backend.emit_pose = False
    old_generation = h.supervisor.generation
    h.initial_pub.publish(h.backend.estimated_pose())
    h.until(lambda: h.supervisor.generation > old_generation)
    assert h.supervisor.quality_range() == 30.
    assert not h.supervisor.prior_verified


def test_slow_rival_result_keeps_validation_window_and_rejects_motion(harnesses, monkeypatch):
    from amr_stack.localization_quality import RivalQuality
    h = harnesses(automatic=False, unique_map=True)
    n = h.supervisor
    h.until(lambda: n.map_quality is not None and n.scan is not None)
    candidate = (4.15, 2.55, .13)
    future = Future()
    future.set_result(RivalQuality(True, 1000, .1, None))
    n.rival_future = future
    n.rival_pose = candidate
    n.rival_submitted_score = .95
    n.rival_submitted_at = time.monotonic()-6.
    n.rival_generation, n.rival_map_key = n.generation, n.map_key
    n.rival_evidence_epoch = n.evidence_epoch
    n.rival_motion_revision = n.motion_revision
    submitted = []
    monkeypatch.setattr(n.worker, 'submit', lambda *a, **kw: submitted.append(kw) or Future())
    assert n.check_rivals(candidate, .95) is None
    assert n.metrics['rival_calculation_sec'] >= 6.
    n.rival_completed_at = time.monotonic()-4.
    assert n.check_rivals(candidate, .95) is None
    assert not submitted
    n.motion_revision += 1
    assert n.check_rivals(candidate, .95) is not None
    assert n.rival_result is None and len(submitted) == 1


@pytest.mark.parametrize('invalid_covariance_index', [14, 35])
def test_amcl_pose_before_matching_tf_uses_fresh_complete_previous_pose(harnesses, monkeypatch, invalid_covariance_index):
    h, faults, saved = _ready_tracking_fixture(harnesses, monkeypatch)
    n = h.supervisor
    h.backend.emit_pose = False
    h.spin(.12)
    previous = copy.deepcopy(n.pose)
    latest_tf = n.tf_buffer.lookup_transform('odom', 'base_footprint', Time())
    ahead = copy.deepcopy(previous)
    ahead.header.stamp = Time(nanoseconds=navigation_startup.stamp_ns(latest_tf.header.stamp)+10_000_000).to_msg()
    assert not n.tf_buffer.can_transform('map', 'base_footprint', Time.from_msg(ahead.header.stamp))
    n.receive('pose', ahead)
    # With only the latest pose, its exact-time TF is unavailable.
    assert n.pose_evidence()[1] is not None
    assert n.data_problem() is None
    assert navigation_startup.stamp_ns(n.pose.header.stamp) < navigation_startup.stamp_ns(ahead.header.stamp)
    assert n.pose_evidence()[1] is None
    n.tick()
    assert n.state == 'READY'
    _assert_tracking_prior_retained(h, faults, saved)
    # A malformed new pose cannot hide behind the usable history.
    ahead.pose.covariance[invalid_covariance_index] = float('nan')
    n.receive('pose', ahead)
    n.tick()
    assert n.state != 'READY'


def test_tracking_recovery_accepts_valid_stopped_evidence_with_soft_warning(harnesses, monkeypatch):
    h, faults, saved = _ready_tracking_fixture(harnesses, monkeypatch)
    faults.quality['origin_free'] = False
    h.until(lambda: h.supervisor.state == 'VERIFYING')
    faults.quality.clear()
    faults.quality['match_ratio'] = .4
    h.until(lambda: h.supervisor.state == 'READY')
    assert h.supervisor.tracking_warning
    assert h.supervisor.good_count >= h.supervisor.cfg['tracking_verification_samples']
    _assert_tracking_prior_retained(h, faults, saved)
