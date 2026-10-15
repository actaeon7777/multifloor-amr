"""Opt-in TIM -> AMCL startup checks using synthetic sensors on loopback DDS.

Run after building/sourcing the workspace:
  AMR_RUN_TIM_TESTS=1 ROS_DOMAIN_ID=196 ROS_LOCALHOST_ONLY=1 pytest -q <this file>

Only TIM, AMCL, and an in-process supervisor are real. Navigation lifecycle
services record requests without launching controllers, physics, or hardware.
The fixture never publishes /initialpose: the supervisor must obtain it from TIM.
"""

import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest

if os.environ.get('AMR_RUN_TIM_TESTS') != '1':
    pytest.skip('Opt in with AMR_RUN_TIM_TESTS=1, domain 196 localhost only', allow_module_level=True)

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from amr_localization_interfaces.srv import GlobalLocalize
from amr_stack.navigation_startup import NavigationStartup, planar, stamp_ns


def compose(first, second):
    c, s = math.cos(first[2]), math.sin(first[2])
    return (first[0] + c * second[0] - s * second[1],
            first[1] + s * second[0] + c * second[1], first[2] + second[2])


def inverse(pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (-c * pose[0] - s * pose[1], s * pose[0] - c * pose[1], -pose[2])


def assert_pose_close(actual, expected, position=.2, angle=.12):
    assert math.hypot(actual[0] - expected[0], actual[1] - expected[1]) < position, (actual, expected)
    assert abs(math.atan2(math.sin(actual[2] - expected[2]),
                          math.cos(actual[2] - expected[2]))) < angle, (actual, expected)


class SyntheticScene(Node):
    def __init__(self, ambiguous=False, extrinsic=(.27, -.11, .31), scan_count=720):
        super().__init__('tim_synthetic_scene')
        self.base_pose = (2.05, 1.55, .13) if ambiguous else (4.15, 2.55, .43)
        self.initial_base_pose = self.base_pose
        self.odom_pose = (0., 0., 0.)
        self.measured_twist = (0., 0., 0.)
        self.extrinsic = extrinsic
        self.laser_pose = compose(self.base_pose, self.extrinsic)
        self.scan_count = scan_count
        self.start_commands = []
        self.navigation_active = False
        self.last_scan = None
        self.latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', self.latched)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(self.transform('base_footprint', 'laser', self.extrinsic))
        for name in ('planner_server', 'controller_server', 'behavior_server', 'bt_navigator'):
            self.create_service(GetState, f'/{name}/get_state', self.lifecycle)
        self.create_service(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes', self.manage)
        if ambiguous:
            self.grid = np.full((40, 100), -1, dtype=np.int8)
            for x0 in (0, 50):
                self.grid[5:35, x0+5:x0+45] = 0
                self.grid[[5, 34], x0+5:x0+45] = 100
                self.grid[5:35, [x0+5, x0+44]] = 100
        else:
            self.grid = np.full((60, 100), -1, dtype=np.int8)
            self.grid[5:55, 5:95] = 0
            self.grid[[5, 54], 5:95] = 100
            self.grid[5:55, [5, 94]] = 100
            for y1, y2, x1, x2 in ((35, 55, 20, 29), (5, 19, 52, 64),
                                    (31, 40, 68, 80), (13, 18, 16, 22)):
                self.grid[y1:y2, x1:x2] = 100
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'map'
        self.map.info.height, self.map.info.width = self.grid.shape
        self.map.info.resolution = .1
        self.map.info.origin.orientation.w = 1.
        self.map.data = self.grid.ravel().tolist()
        self.map_pub.publish(self.map)
        self.ranges = self.raycast()
        self.create_timer(.05, self.publish_sensors)

    def raycast(self, laser_pose=None):
        pose = self.laser_pose if laser_pose is None else laser_pose
        angles = -math.pi + np.arange(self.scan_count) * 2*math.pi/self.scan_count + pose[2]
        directions = np.stack((np.cos(angles), np.sin(angles)))
        result = np.full(self.scan_count, float('inf'))
        active = np.arange(self.scan_count)
        # March all active beams together so dense moving scans remain cheap to
        # prepare, using the same one-centimetre ray steps as the fixed fixture.
        for distance in np.arange(.01, 12., .01):
            if not len(active):
                break
            cells = np.floor((np.asarray(pose[:2])[:, None] + distance * directions[:, active]) / .1).astype(int)
            inside = ((cells[0] >= 0) & (cells[0] < self.grid.shape[1])
                      & (cells[1] >= 0) & (cells[1] < self.grid.shape[0]))
            active, cells = active[inside], cells[:, inside]
            hit = self.grid[cells[1], cells[0]] == 100
            result[active[hit]] = distance
            active = active[~hit]
        return result.tolist()

    def set_pose(self, pose, ranges, measured_twist=(0., 0., 0.)):
        self.base_pose = pose
        self.laser_pose = compose(pose, self.extrinsic)
        # Odom starts at the original robot pose, including its nonzero yaw.
        self.odom_pose = compose(inverse(self.initial_base_pose), pose)
        self.ranges = ranges
        self.measured_twist = measured_twist

    def transform(self, parent, child, pose, stamp=None):
        result = TransformStamped()
        result.header.frame_id, result.child_frame_id = parent, child
        result.header.stamp = stamp or self.get_clock().now().to_msg()
        result.transform.translation.x, result.transform.translation.y = pose[:2]
        result.transform.rotation.z = math.sin(pose[2]/2)
        result.transform.rotation.w = math.cos(pose[2]/2)
        return result

    def lifecycle(self, request, response):
        response.current_state.id = (State.PRIMARY_STATE_ACTIVE if self.navigation_active
                                     else State.PRIMARY_STATE_UNCONFIGURED)
        return response

    def manage(self, request, response):
        self.start_commands.append(request.command)
        response.success = request.command == ManageLifecycleNodes.Request.STARTUP
        self.navigation_active = response.success
        return response

    def publish_sensors(self):
        stamp = self.get_clock().now().to_msg()
        # AMCL alone supplies map -> odom. Truth updates affect TF, odometry,
        # and raycast scans together; no velocity command is published.
        self.tf.sendTransform(self.transform('odom', 'base_footprint', self.odom_pose, stamp))
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
        odom.header.stamp = stamp
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.odom_pose[:2]
        odom.pose.pose.orientation.z = math.sin(self.odom_pose[2]/2)
        odom.pose.pose.orientation.w = math.cos(self.odom_pose[2]/2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = self.measured_twist
        self.odom_pub.publish(odom)
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'laser', stamp
        scan.angle_min = -math.pi
        scan.angle_increment = 2*math.pi/self.scan_count
        scan.angle_max = scan.angle_min + (self.scan_count-1)*scan.angle_increment
        scan.range_min, scan.range_max = .01, 12.
        scan.ranges = self.ranges
        self.last_scan = scan
        self.scan_pub.publish(scan)


class TimHarness:
    def __init__(self, directory, ambiguous=False, startup=True, extrinsic=(.27, -.11, .31), scan_count=720):
        assert os.environ.get('ROS_DOMAIN_ID') == '196', 'TIM tests require isolated domain 196'
        assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'TIM tests require loopback DDS'
        self.directory = directory
        self.processes = []
        self.logs = []
        self.supervisor = None
        self.nodes = []
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        try:
            self.scene = SyntheticScene(ambiguous, extrinsic, scan_count)
            self.probe = Node('tim_integration_probe')
            self.nodes.extend((self.scene, self.probe))
            self.initial_poses, self.commands, self.statuses = [], [], []
            self.probe.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_poses.append, 10)
            self.probe.create_subscription(Twist, '/cmd_vel', self.commands.append, 10)
            self.probe.create_subscription(String, '/localization/status',
                                           lambda m: self.statuses.append(json.loads(m.data)), self.scene.latched)
            for node in self.nodes:
                self.executor.add_node(node)
            tim_prefix = Path(get_package_prefix('amr_tim_localization'))
            self.start_process('tim', [str(tim_prefix / 'lib/amr_tim_localization/tim_global_localizer')])
            self.client = self.probe.create_client(GlobalLocalize, '/localization/global_localize')
            self.until(self.client.service_is_ready, timeout=15.)
            self.until(lambda: self.scene.last_scan is not None)
            if startup:
                config = Path(__file__).resolve().parents[1] / 'config/localization.yaml'
                self.start_process('amcl', ['/opt/ros/jazzy/lib/nav2_amcl/amcl', '--ros-args',
                                             '--params-file', str(config)])
                lifecycle = self.probe.create_client(ChangeState, '/amcl/change_state')
                self.until(lifecycle.service_is_ready)
                for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                    request = ChangeState.Request()
                    request.transition.id = transition
                    result = lifecycle.call_async(request)
                    self.until(result.done)
                    assert result.result().success, self.diagnostics()
                options = dict(automatic_localization=True, global_localization_backend='tim',
                               tim_timeout_sec=30., tim_max_candidates=8, search_timeout=35.,
                               stationary_duration=.15, verification_duration=1., verification_samples=3,
                               pose_timeout=3., data_timeout=1.5, navigation_start_timeout=5.)
                self.supervisor = NavigationStartup(parameter_overrides=[
                    Parameter(name, value=value) for name, value in options.items()])
                self.nodes.append(self.supervisor)
                self.executor.add_node(self.supervisor)
        except BaseException:
            self.close()
            raise

    def start_process(self, name, args):
        log = (self.directory / f'{name}.log').open('w')
        self.logs.append(log)
        environment = dict(os.environ, OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='1')
        self.processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                               env=environment, start_new_session=True))

    def diagnostics(self):
        state = None if self.supervisor is None else (
            self.supervisor.state, self.supervisor.detail, self.supervisor.metrics)
        logs = {Path(f.name).name: Path(f.name).read_text()[-5000:] for f in self.logs}
        return {'supervisor': state, 'logs': logs, 'initial_poses': len(self.initial_poses)}

    def until(self, predicate, timeout=15.):
        end = time.monotonic() + timeout
        while not predicate():
            assert all(p.poll() is None for p in self.processes), self.diagnostics()
            assert time.monotonic() < end, self.diagnostics()
            self.executor.spin_once(timeout_sec=.02)

    def spin(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.02)

    def localize(self, map_message=None, scan_message=None):
        request = GlobalLocalize.Request()
        request.map = map_message or self.scene.map
        request.scan = copy.deepcopy(scan_message or self.scene.last_scan)
        request.max_candidates = 8
        request.timeout_sec = 30.
        future = self.client.call_async(request)
        self.until(future.done, timeout=35.)
        return request, future.result()

    def close(self):
        for process in reversed(self.processes):
            if process.poll() is None:
                # A failed latency test must not leave its TIM child stopped.
                process.send_signal(signal.SIGCONT)
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)
        self.executor.shutdown()
        for node in reversed(self.nodes):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for log in self.logs:
            log.close()


@pytest.fixture
def tim_harness(tmp_path, monkeypatch):
    ros_logs = tmp_path / 'ros_logs'
    ros_logs.mkdir()
    monkeypatch.setenv('ROS_LOG_DIR', str(ros_logs))
    instances = []

    def create(**kwargs):
        result = TimHarness(tmp_path, **kwargs)
        instances.append(result)
        return result

    yield create
    for instance in instances:
        instance.close()


@pytest.mark.parametrize('extrinsic', [(.27, -.11, .31), (0., 0., math.pi)],
                         ids=['translated_rotated_laser', 'urdf_backward_laser'])
def test_real_tim_and_amcl_unknown_pose_with_nonzero_laser_extrinsic(tim_harness, extrinsic):
    h = tim_harness(extrinsic=extrinsic)
    h.until(lambda: h.supervisor.auto_attempted)
    h.until(lambda: h.supervisor.state in ('READY', 'WAITING_INITIAL_POSE', 'FAILED'), timeout=70.)
    assert h.supervisor.state == 'READY', h.diagnostics()
    assert h.supervisor.source == 'auto'
    assert h.statuses[-1]['initializer_backend'] == 'tim'
    assert h.initial_poses, h.diagnostics()
    assert_pose_close(planar(h.initial_poses[0].pose.pose), h.scene.base_pose)
    assert_pose_close(planar(h.supervisor.pose.pose.pose), h.scene.base_pose)
    assert h.probe.count_publishers('/amcl_pose') == 1
    assert h.supervisor.metrics['rival_search_completed']
    assert h.scene.start_commands == [ManageLifecycleNodes.Request.STARTUP]
    assert not h.commands
    h.spin(1.)
    assert h.supervisor.state == 'READY', h.diagnostics()


def test_real_tim_transports_small_drift_before_real_amcl_verification(tim_harness):
    # Fine refinement needs actual dense input, not 720 points relabelled 1080.
    h = tim_harness(scan_count=3240)
    original = h.scene.base_pose
    trajectory = []
    for step in range(1, 13):
        pose = (original[0] + .006 * step/12, original[1], original[2] + .003 * step/12)
        trajectory.append((pose, h.scene.raycast(compose(pose, h.scene.extrinsic))))
    h.until(lambda: h.supervisor.tim_operation is not None)
    operation = h.supervisor.tim_operation
    assert len(operation['scan'].ranges) == 3240
    assert not operation['future'].done()
    tim_process = h.processes[0]
    # Suspend only this test's real TIM child after dispatch. The executor and
    # synthetic sensor stream remain active while the response is outstanding.
    # This makes response latency deterministic without substituting a service.
    tim_process.send_signal(signal.SIGSTOP)
    try:
        for pose, ranges in trajectory:
            # 0.006 m / 0.6 s and 0.003 rad / 0.6 s, below stationarity limits.
            h.scene.set_pose(pose, ranges, (.01*math.cos(pose[2]), -.01*math.sin(pose[2]), .005))
            h.scene.publish_sensors()
            h.spin(.05)
            assert h.supervisor.tim_operation is operation, h.diagnostics()
            assert not h.initial_poses, h.diagnostics()
        h.scene.measured_twist = (0., 0., 0.)
        h.scene.publish_sensors()
        h.spin(.1)
    finally:
        if tim_process.poll() is None:
            tim_process.send_signal(signal.SIGCONT)
    assert operation['scan'].ranges != h.scene.last_scan.ranges
    h.until(lambda: bool(h.initial_poses), timeout=35.)
    response = operation['future'].result()
    assert response.success and response.poses, h.diagnostics()
    delta = compose(inverse(operation['odom_pose']), h.scene.odom_pose)
    base_candidates = [compose(planar(pose.pose), inverse(h.scene.extrinsic)) for pose in response.poses]
    seed = planar(h.initial_poses[0].pose.pose)
    transported = [compose(candidate, delta) for candidate in base_candidates]
    assert any(math.hypot(seed[0]-pose[0], seed[1]-pose[1]) < 1e-7
               and abs(math.atan2(math.sin(seed[2]-pose[2]), math.cos(seed[2]-pose[2]))) < 1e-7
               for pose in transported), (seed, transported, h.diagnostics())
    # A millimetre-scale transport must not pass merely because the map-pose
    # accuracy assertion below has a larger tolerance for ICP and AMCL noise.
    assert all(math.hypot(seed[0]-pose[0], seed[1]-pose[1]) > .004 for pose in base_candidates)
    assert stamp_ns(h.initial_poses[0].header.stamp) > stamp_ns(operation['scan'].header.stamp)
    assert stamp_ns(h.initial_poses[0].header.stamp) == h.supervisor.tim_metrics['tim_seed_stamp_ns']
    assert h.supervisor.tim_metrics['tim_compensation_translation'] == pytest.approx(.006, abs=1e-8)
    assert h.supervisor.tim_metrics['tim_compensation_yaw'] == pytest.approx(.003, abs=1e-8)
    assert h.supervisor.tim_metrics['tim_motion_events'] == h.supervisor.tim_metrics['tim_goal_events'] == 0
    h.until(lambda: h.supervisor.state in ('READY', 'WAITING_INITIAL_POSE', 'FAILED'), timeout=70.)
    assert h.supervisor.state == 'READY', h.diagnostics()
    assert h.supervisor.source == 'auto'
    assert h.statuses[-1]['initializer_backend'] == 'tim'
    assert_pose_close(seed, h.scene.base_pose)
    assert_pose_close(planar(h.supervisor.pose.pose.pose), h.scene.base_pose)
    transform = h.supervisor.tf_buffer.lookup_transform('map', 'base_footprint', Time())
    map_base = (transform.transform.translation.x, transform.transform.translation.y,
                2*math.atan2(transform.transform.rotation.z, transform.transform.rotation.w))
    assert_pose_close(map_base, h.scene.base_pose)
    assert h.probe.count_publishers('/amcl_pose') == 1
    assert h.supervisor.metrics['rival_search_completed']
    assert h.scene.start_commands == [ManageLifecycleNodes.Request.STARTUP]
    assert not h.commands


def test_real_tim_repeated_corridors_never_release_navigation(tim_harness):
    h = tim_harness(ambiguous=True)
    h.until(lambda: h.supervisor.auto_attempted)
    h.until(lambda: h.supervisor.state in ('WAITING_INITIAL_POSE', 'FAILED'), timeout=75.)
    assert not h.scene.start_commands
    assert not h.commands
    assert not any(status.get('ready') for status in h.statuses)
    assert h.statuses[-1]['initializer_backend'] == 'tim'
    # A completed alternative search must reveal an equally good corridor.
    assert h.supervisor.metrics['rival_search_completed'], h.diagnostics()
    assert h.supervisor.metrics['rival_score'] >= .9, h.diagnostics()


def test_real_tim_returns_laser_pose_and_invalidates_rotated_origin_cache(tim_harness):
    h = tim_harness(startup=False)
    original_request, original = h.localize()
    assert original.success and original.poses, (original.message, h.diagnostics())
    assert_pose_close(planar(original.poses[0].pose), h.scene.laser_pose)
    changed = copy.deepcopy(h.scene.map)
    origin = (-3.2, 1.7, .61)
    changed.info.origin.position.x, changed.info.origin.position.y = origin[:2]
    changed.info.origin.orientation.z = math.sin(origin[2]/2)
    changed.info.origin.orientation.w = math.cos(origin[2]/2)
    # Cell data and dimensions remain identical: origin belongs in cache identity.
    request, result = h.localize(map_message=changed)
    assert result.success and result.poses, (result.message, h.diagnostics())
    assert_pose_close(planar(result.poses[0].pose), compose(origin, h.scene.laser_pose))
    assert len(result.poses) == len(result.scores) <= request.max_candidates
    assert all(math.isfinite(score) for score in result.scores)
    for pose in result.poses:
        assert pose.header.frame_id == 'map'
        assert stamp_ns(pose.header.stamp) == stamp_ns(request.scan.header.stamp)
    assert not h.initial_poses
    assert not h.commands
    invalid = copy.deepcopy(original_request.scan)
    invalid.ranges = [float('nan')] * len(invalid.ranges)
    _, rejected = h.localize(scan_message=invalid)
    assert not rejected.success and not rejected.poses
    assert all(math.isfinite(score) for score in rejected.scores)
