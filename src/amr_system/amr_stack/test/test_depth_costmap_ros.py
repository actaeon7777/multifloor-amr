"""Exercise low-obstacle marking with real Nav2 and synthetic clouds, without IO."""
import copy
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import pytest
import yaml

if os.environ.get('AMR_RUN_DEPTH_COSTMAP_TESTS') != '1':
    pytest.skip('Opt in with AMR_RUN_DEPTH_COSTMAP_TESTS=1, domain 197 localhost only',
                allow_module_level=True)

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster


@pytest.mark.parametrize('legacy_threshold', [True, False], ids=['previous-setting', 'configured'])
@pytest.mark.parametrize('height', [.10, .12], ids=['10cm', '12cm'])
def test_low_box_marking_floor_rejection_and_clearing(tmp_path, legacy_threshold, height):
    assert os.environ['ROS_DOMAIN_ID'] == '197'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    assert os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    config = Path(__file__).resolve().parents[1] / 'config/navigation.yaml'
    params = copy.deepcopy(yaml.safe_load(config.read_text())[
        'local_costmap']['local_costmap']['ros__parameters'])
    params.update(plugins=['depth_layer', 'inflation_layer'],
                  always_send_full_costmap=True, publish_frequency=10.0)
    # The layer itself is under test: mark from the raw cloud, not the filter output.
    params['depth_layer']['depth_mark']['topic'] = '/camera/depth/points'
    if legacy_threshold:
        params['depth_layer']['mark_threshold'] = 1
    config_path = tmp_path / 'costmap.yaml'
    config_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    env = dict(os.environ, ROS_LOG_DIR=str(tmp_path / 'ros_logs'))
    rclpy.init()
    node = rclpy.create_node('depth_costmap_test_input', enable_rosout=False)
    log_path = tmp_path / 'costmap.log'
    log = log_path.open('w')
    process = None
    try:
        process = subprocess.Popen([
            '/opt/ros/jazzy/lib/nav2_costmap_2d/nav2_costmap_2d', '--ros-args',
            '-r', '__node:=depth_costmap_test', '--params-file', str(config_path),
        ], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        tf = StaticTransformBroadcaster(node)
        transforms = []
        sensor_origin = (.320488, .025, .147017)
        for child, position in [('base_footprint', (0., 0., 0.)),
                                ('depth_test_sensor', sensor_origin)]:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = 'odom', child
            t.header.stamp = node.get_clock().now().to_msg()
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = position
            t.transform.rotation.w = 1.
            transforms.append(t)
        tf.sendTransform(transforms)
        publisher = node.create_publisher(PointCloud2, '/camera/depth/points', qos_profile_sensor_data)
        grids = []
        node.create_subscription(OccupancyGrid, '/costmap', grids.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def spin_until(predicate, timeout=15.):
            deadline = time.monotonic() + timeout
            while not predicate():
                assert process.poll() is None, log_path.read_text()
                assert time.monotonic() < deadline, log_path.read_text()
                rclpy.spin_once(node, timeout_sec=.02)

        client = node.create_client(ChangeState, '/depth_costmap_test/change_state')
        spin_until(client.service_is_ready)
        for transition in [Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]:
            future = client.call_async(ChangeState.Request(transition=Transition(id=transition)))
            spin_until(future.done)
            assert future.result().success, log_path.read_text()
        spin_until(lambda: publisher.get_subscription_count() == 2 and bool(grids))

        def feed(world_points, seconds=1.2):
            payload = b''.join(struct.pack('<fff', *(v-o for v, o in zip(point, sensor_origin)))
                               for point in world_points)
            cloud = PointCloud2(height=1, width=len(world_points), point_step=12,
                                row_step=len(payload), data=payload, is_dense=True,
                                fields=[PointField(name=name, offset=i*4,
                                                   datatype=PointField.FLOAT32, count=1)
                                        for i, name in enumerate(('x', 'y', 'z'))])
            cloud.header.frame_id = 'depth_test_sensor'
            count = len(grids)
            deadline, next_send = time.monotonic()+seconds, 0.
            while time.monotonic() < deadline:
                if time.monotonic() >= next_send:
                    cloud.header.stamp = node.get_clock().now().to_msg()
                    publisher.publish(cloud)
                    next_send = time.monotonic()+.1
                rclpy.spin_once(node, timeout_sec=.01)
            assert len(grids) > count, 'Costmap stopped publishing'
            return grids[-1]

        def cost(grid, x, y):
            mx = int((x-grid.info.origin.position.x)/grid.info.resolution)
            my = int((y-grid.info.origin.position.y)/grid.info.resolution)
            return grid.data[my*grid.info.width+mx]

        # Floor noise below 6 cm must not become a lethal obstacle.
        floor = [(2.425, y, .04) for y in [-.125, .025, .175]]
        grid = feed(floor)
        assert 100 not in grid.data

        # A low box top surface occupies only one vertical voxel per XY cell.
        # Repeating observations cannot satisfy the former two-voxel requirement.
        box = [(x, y, height) for x in [2.375, 2.425, 2.475]
               for y in [-.125, -.075, -.025, .025, .075, .125, .175]]
        grid = feed(box)
        assert (cost(grid, 2.425, .025) == 100) is (not legacy_threshold)

        if not legacy_threshold:
            # Empty camera frames alone must not erase a remembered obstacle.
            grid = feed([])
            assert cost(grid, 2.425, .025) == 100
            # Floor and farther surface returns clear both possible height bins.
            grid = feed([(2.825, .025, z) for z in [0., .12]])
            # The farther return may inflate this cell, but the old box is gone.
            assert cost(grid, 2.425, .025) < 100
    finally:
        if process is not None and process.poll() is None:
            for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]:
                os.killpg(process.pid, sig)
                try:
                    process.wait(timeout=5)
                    break
                except subprocess.TimeoutExpired:
                    continue
        log.close()
        node.destroy_node()
        rclpy.shutdown()


def test_depth_only_box_invalidates_straight_path_and_gets_detour(tmp_path):
    """A low box absent from map/LiDAR must reach Smac's path-validity check."""
    assert os.environ['ROS_DOMAIN_ID'] == '197'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    assert os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    config = Path(__file__).resolve().parents[1] / 'config/navigation.yaml'
    params = yaml.safe_load(config.read_text())
    params['global_costmap']['global_costmap']['ros__parameters'].update(
        always_send_full_costmap=True)
    params['global_costmap']['global_costmap']['ros__parameters']['depth_layer'][
        'depth_mark']['topic'] = '/camera/depth/points'
    config_path = tmp_path / 'planner.yaml'
    config_path.write_text(yaml.safe_dump(params))
    rclpy.init()
    node = rclpy.create_node('depth_planning_test_input', enable_rosout=False)
    log_path = tmp_path / 'planner.log'
    log = log_path.open('w')
    process = None
    trace = {}
    try:
        process = subprocess.Popen([
            '/opt/ros/jazzy/lib/nav2_planner/planner_server', '--ros-args',
            '--params-file', str(config_path),
        ], env=dict(os.environ, ROS_LOG_DIR=str(tmp_path/'ros_logs')),
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_pub = node.create_publisher(OccupancyGrid, '/map', latched)
        scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        cloud_pub = node.create_publisher(PointCloud2, '/camera/depth/points', qos_profile_sensor_data)
        grids = []
        node.create_subscription(OccupancyGrid, '/global_costmap/costmap', grids.append, latched)
        tf = StaticTransformBroadcaster(node)
        transforms = []
        sensor_origin = (.320488, .025, .147017)
        for child, origin in [('base_footprint', (0., 0., 0.)), ('laser', (0., 0., .195)),
                              ('depth_test_sensor', sensor_origin)]:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = 'map', child
            t.header.stamp = node.get_clock().now().to_msg()
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = origin
            t.transform.rotation.w = 1.
            transforms.append(t)
        tf.sendTransform(transforms)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = node.get_clock().now().to_msg()
        grid.info.resolution, grid.info.width, grid.info.height = .05, 120, 100
        grid.info.origin.position.x, grid.info.origin.position.y = -1., -2.5
        grid.info.origin.orientation.w = 1.
        # A 2.8 m corridor: space for a detour on either side of a 40 cm box.
        grid.data = [100 if abs(-2.5+(row+.5)*.05) >= 1.4 else 0
                     for row in range(100) for col in range(120)]
        map_pub.publish(grid)
        points = []

        def publish_inputs():
            scan = LaserScan()
            scan.header.frame_id = 'laser'
            scan.header.stamp = node.get_clock().now().to_msg()
            scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/360
            scan.angle_max = scan.angle_min+359*scan.angle_increment
            scan.range_min, scan.range_max, scan.scan_time = .05, 8., .1
            scan.ranges = [float('inf')]*360  # The low box is invisible to LiDAR.
            scan_pub.publish(scan)
            payload = b''.join(struct.pack('<fff', *(v-o for v, o in zip(p, sensor_origin)))
                               for p in points)
            cloud = PointCloud2(height=1, width=len(points), point_step=12,
                                row_step=len(payload), data=payload, is_dense=True,
                                fields=[PointField(name=name, offset=i*4,
                                                   datatype=PointField.FLOAT32, count=1)
                                        for i, name in enumerate(('x', 'y', 'z'))])
            cloud.header.frame_id = 'depth_test_sensor'
            cloud.header.stamp = scan.header.stamp
            cloud_pub.publish(cloud)

        node.create_timer(.1, publish_inputs)

        def until(predicate, seconds=15.):
            deadline = time.monotonic()+seconds
            while not predicate():
                assert process.poll() is None, log_path.read_text()
                assert time.monotonic() < deadline, log_path.read_text()
                rclpy.spin_once(node, timeout_sec=.02)

        def request(client, value):
            until(client.service_is_ready)
            future = client.call_async(value)
            until(future.done)
            return future.result()

        state_client = node.create_client(ChangeState, '/planner_server/change_state')
        for transition in [Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]:
            response = request(state_client, ChangeState.Request(transition=Transition(id=transition)))
            assert response.success, log_path.read_text()
        until(lambda: bool(grids))
        planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        until(planner.server_is_ready)

        def path():
            poses = []
            for x in [.025, 3.975]:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = node.get_clock().now().to_msg()
                pose.pose.position.x, pose.pose.position.y = x, .025
                pose.pose.orientation.w = 1.
                poses.append(pose)
            future = planner.send_goal_async(ComputePathToPose.Goal(
                start=poses[0], goal=poses[1], use_start=True, planner_id='GridBased'))
            until(future.done)
            handle = future.result()
            assert handle.accepted
            future = handle.get_result_async()
            until(future.done)
            outcome = future.result()
            assert outcome.status == 4 and outcome.result.error_code == 0, str(outcome)
            assert len(outcome.result.path.poses) > 2
            return outcome.result.path

        straight = path()
        trace['before'] = [[p.pose.position.x, p.pose.position.y] for p in straight.poses]
        assert max(abs(y-.025) for _, y in trace['before']) < .10
        # The saved map and LiDAR remain unchanged; only the depth cloud adds a box.
        points.extend((1.325+i*.05, -.175+j*.05, .10) for i in range(6) for j in range(8))
        count = len(grids)
        until(lambda: len(grids) >= count+3)
        observed = grids[-1]
        mx = int((1.425-observed.info.origin.position.x)/observed.info.resolution)
        my = int((.025-observed.info.origin.position.y)/observed.info.resolution)
        trace['box_cost'] = observed.data[my*observed.info.width+mx]
        assert trace['box_cost'] == 100, 'Depth-only box never reached the global costmap'
        valid = node.create_client(IsPathValid, '/is_path_valid')
        result = request(valid, IsPathValid.Request(path=straight))
        trace['original_path_valid_after_box'] = result.is_valid
        assert not result.is_valid
        detour = path()
        trace['after'] = [[p.pose.position.x, p.pose.position.y] for p in detour.poses]
        assert request(valid, IsPathValid.Request(path=detour)).is_valid
        clearance = min(math.hypot(max(1.3-x, 0., x-1.6), max(-.2-y, 0., y-.2))
                        for x, y in trace['after'])
        trace['minimum_center_to_box_distance'] = clearance
        # Allow one grid cell discretization relative to the 44 cm planning radius.
        assert clearance >= .39, trace
        assert max(abs(y-.025) for _, y in trace['after']) > .55
    finally:
        (tmp_path/'path_trace.json').write_text(json.dumps(trace, indent=2)+'\n')
        if process is not None and process.poll() is None:
            for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]:
                os.killpg(process.pid, sig)
                try:
                    process.wait(timeout=5)
                    break
                except subprocess.TimeoutExpired:
                    continue
        log.close()
        node.destroy_node()
        rclpy.shutdown()
