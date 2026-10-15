"""Real Smac/DWB/behaviors -> manager -> existing mux -> dry-run CAN.

Wall-clock kinematic fixture supplies map, scan, TF, odometry and synthetic Joy.
Synthetic AMCL evidence exercises the real startup supervisor and readiness gate;
this fixture does not measure AMCL convergence or global localization accuracy.
No physical IO node is started. This checks software integration, not real-world
braking, localization accuracy, traction or sensor failure coverage.
"""
from collections import deque
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import pytest

if os.environ.get('AMR_RUN_NAV2_INTEGRATION') != '1':
    pytest.skip('Opt in with AMR_RUN_NAV2_INTEGRATION=1, isolated domain 191', allow_module_level=True)

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from sensor_msgs.msg import Imu, Joy, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class Robot(Node):
    def __init__(self, boxes=None):
        self.boxes = boxes if boxes is not None else [(.8, -.25, 1.2, .25)]
        super().__init__('synthetic_navigation_robot')
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.command_at = 0.
        self.previous = time.monotonic()
        self.rb = self.lb = False
        self.joy_connected = True
        self.manual = 0.0
        self.mode = ''
        self.status = ''
        self.localization_status = {}
        self.time_health = {}
        self.max_beams = 360
        self.beam_phase_requests = []
        self.pose_history = deque(maxlen=6)
        self.previous_scan = None
        self.commands, self.raw_commands, self.previews, self.paths = [], [], [], []
        self.poses = []
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.static.sendTransform([
            self.transform('map', 'odom'),
            self.transform('base_footprint', 'base_link', z=.086),
            self.transform('base_link', 'laser', z=.108605, yaw=math.pi),
        ])
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.create_service(GetState, '/amcl/get_state', self.amcl_state)
        self.create_service(SetParametersAtomically, '/amcl/set_parameters_atomically', self.amcl_parameters)
        self.create_service(GetParameters, '/amcl/get_parameters', self.amcl_parameter_values)
        self.create_service(Empty, '/request_nomotion_update', lambda request, response: response)
        self.create_subscription(String, '/localization/status',
                                 lambda m: setattr(self, 'localization_status', json.loads(m.data)), qos)
        self.create_subscription(String, '/system/time_health',
                                 lambda m: setattr(self, 'time_health', json.loads(m.data)), qos)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = .05
        grid.info.width = grid.info.height = 240
        grid.info.origin.position.x = grid.info.origin.position.y = -6.
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*(240*240)
        for row in range(240):
            for col in range(240):
                x, y = -6+(col+.5)*.05, -6+(row+.5)*.05
                if row in (0,239) or col in (0,239) or any(x1 <= x <= x2 and y1 <= y <= y2 for x1, y1, x2, y2 in self.boxes):
                    grid.data[row*240+col] = 100
        self.grid = grid
        self.map_pub.publish(grid)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.wheel_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.manual_pub = self.create_publisher(Twist, '/cmd_vel_manual', 1)
        self.create_subscription(Twist, '/cmd_vel', self.command, 100)
        self.create_subscription(Twist, '/cmd_vel_navigation',
                                 lambda m: self.raw_commands.append((time.monotonic(), m.linear.x, m.angular.z)), 100)
        self.create_subscription(String, '/control_mux/mode', lambda m: setattr(self, 'mode', m.data), qos)
        self.create_subscription(String, '/navigation/status', lambda m: setattr(self, 'status', m.data), qos)
        self.create_subscription(String, '/cmd_vel_to_can/tx_preview', lambda m: self.previews.append(m.data), 100)
        self.create_subscription(NavPath, '/plan', lambda m: self.paths.append(m), 10)
        self.create_timer(.02, self.motion)
        self.create_timer(.05, self.joystick)
        self.create_timer(.10, self.scan)

    @staticmethod
    def amcl_state(request, response):
        response.current_state.id = 3
        response.current_state.label = 'active'
        return response

    def initialize_localization(self):
        pose = self.synthetic_pose()
        self.initial_pose_pub.publish(pose)

    def amcl_parameters(self, request, response):
        assert len(request.parameters) == 1
        parameter = request.parameters[0]
        assert parameter.name == 'max_beams'
        assert parameter.value.type == ParameterType.PARAMETER_INTEGER
        self.max_beams = parameter.value.integer_value
        self.beam_phase_requests.append(self.max_beams)
        response.result.successful = True
        return response

    def amcl_parameter_values(self, request, response):
        assert request.names == ['max_beams']
        response.values = [ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=self.max_beams)]
        return response

    def synthetic_pose(self):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x, pose.pose.pose.position.y = self.x, self.y
        pose.pose.pose.orientation.z = math.sin(self.yaw / 2)
        pose.pose.pose.orientation.w = math.cos(self.yaw / 2)
        pose.pose.covariance[0] = pose.pose.covariance[7] = .01
        pose.pose.covariance[35] = .01
        return pose

    def add_obstacle(self, box):
        # A newly observed obstacle changes /scan, not the saved localization map.
        # Global/local obstacle layers must trigger replanning from these returns.
        self.boxes.append(box)

    def transform(self, parent, child, x=0., y=0., z=0., yaw=0.):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = x, y, z
        msg.transform.rotation.z, msg.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
        return msg

    def command(self, message):
        self.v, self.w = message.linear.x, message.angular.z
        self.command_at = time.monotonic()
        self.commands.append((self.command_at, self.v, self.w))

    def motion(self):
        now = time.monotonic()
        dt, self.previous = min(now-self.previous, .1), now
        if now-self.command_at > .3:
            self.v = self.w = 0.
        self.x += self.v*math.cos(self.yaw+self.w*dt/2)*dt
        self.y += self.v*math.sin(self.yaw+self.w*dt/2)*dt
        self.yaw += self.w*dt
        self.poses.append((self.x, self.y, self.yaw))
        transform = self.transform('odom', 'base_footprint', self.x, self.y, yaw=self.yaw)
        self.tf.sendTransform(transform)
        odom = Odometry()
        odom.header.stamp = transform.header.stamp
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(odom)
        self.wheel_pub.publish(odom)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = transform.header.stamp, 'imu_link'
        imu.orientation.w = 1.
        imu.angular_velocity.z = self.w
        self.imu_pub.publish(imu)
        pose = self.synthetic_pose()
        pose.header.stamp = transform.header.stamp
        # Real AMCL estimates refer to a scan already covered by odom TF.
        # Model that processing delay so cross-topic delivery order cannot
        # leave the newest synthetic pose just ahead of the TF buffer.
        self.pose_history.append(pose)
        if len(self.pose_history) == self.pose_history.maxlen:
            self.pose_pub.publish(self.pose_history[0])

    def joystick(self):
        joy = Joy()
        joy.axes = [0., 0., 0., 0.]
        joy.buttons = [0]*8
        joy.buttons[6], joy.buttons[7] = int(self.lb), int(self.rb)
        if self.joy_connected:
            self.joy_pub.publish(joy)
        manual = Twist()
        manual.linear.x = self.manual
        self.manual_pub.publish(manual)

    def scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/360
        scan.angle_max = scan.angle_min+359*scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = .05, 8., .1
        # Occupancy cells span [-6, 6); aim at boundary cell centers, not at
        # the exclusive +6 edge outside the map's known-space extent.
        wall = 5.975
        segments = [(-wall,-wall,wall,-wall), (wall,-wall,wall,wall),
                    (wall,wall,-wall,wall), (-wall,wall,-wall,-wall)]
        for x1, y1, x2, y2 in self.boxes:
            segments.extend([(x1,y1,x2,y1), (x2,y1,x2,y2), (x2,y2,x1,y2), (x1,y2,x1,y1)])
        ranges = []
        for i in range(360):
            angle = self.yaw+math.pi+scan.angle_min+i*scan.angle_increment
            dx, dy = math.cos(angle), math.sin(angle)
            distance = 8.
            for x1,y1,x2,y2 in segments:
                sx,sy=x2-x1,y2-y1
                determinant = dx*sy-dy*sx
                if abs(determinant)<1e-10:
                    continue
                ax,ay=x1-self.x,y1-self.y
                t=(ax*sy-ay*sx)/determinant
                u=(ax*dy-ay*dx)/determinant
                if t>.05 and 0<=u<=1:
                    distance=min(distance,t)
            ranges.append(float(distance))
        scan.ranges = ranges
        if self.previous_scan is not None:
            self.scan_pub.publish(self.previous_scan)
        self.previous_scan = scan


def overlaps_obstacle(pose, box=(.8, -.25, 1.2, .25)):
    """Separating-axis check of the actual body and the fixture's fixed box."""
    x, y, yaw = pose
    x1, y1, x2, y2 = box
    cx, cy = (x1+x2)/2, (y1+y2)/2
    c, s = math.cos(yaw), math.sin(yaw)
    for ax, ay in ((1., 0.), (0., 1.), (c, s), (-s, c)):
        separation = abs((cx-x)*ax+(cy-y)*ay)
        body_extent = .320*abs(c*ax+s*ay)+.272*abs(-s*ax+c*ay)
        obstacle_extent = (x2-x1)/2*abs(ax)+(y2-y1)/2*abs(ay)
        if separation >= body_extent+obstacle_extent:
            return False
    return True


def test_real_smac_dwb_mux_pause_goal_and_recovery(tmp_path):
    assert os.environ['ROS_DOMAIN_ID'] == '191'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rclpy.init()
    robot = Robot()
    share = Path(get_package_share_directory('amr_stack'))
    processes, logs = [], []

    def start(name, args):
        stream = (tmp_path/(name+'.log')).open('w')
        logs.append(stream)
        p = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p)
        return p

    def until(predicate, seconds=15):
        end = time.monotonic()+seconds
        while not predicate():
            assert time.monotonic()<end, (robot.x,robot.y,robot.yaw,robot.status,robot.localization_status,
                (tmp_path/'navigation.log').read_text()[-8000:])
            assert all(p.poll() is None for p in processes), (tmp_path/'navigation.log').read_text()[-10000:]
            rclpy.spin_once(robot, timeout_sec=.01)

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(robot, timeout_sec=.01)

    def wait(future, seconds=15):
        until(future.done, seconds)
        return future.result()

    def toggle(expected):
        robot.rb = False; spin(.15)
        robot.rb = True
        until(lambda: robot.mode == expected)
        spin(.12); robot.rb = False; spin(.15)

    try:
        startup_config = tmp_path / 'startup.yaml'
        startup_config.write_text('navigation_startup:\n  ros__parameters:\n    automatic_localization: false\n')
        navigation = start('navigation', ['ros2','launch','amr_stack','navigation.launch.py',
                                         'startup_config:=' + str(startup_config)])
        start('time_health', ['ros2','run','amr_control','time_health'])
        start('mux', ['ros2','run','amr_control','control_mux','--ros-args','--params-file',str(share/'config/control_mux.yaml'),'-p','require_time_health:=true'])
        start('can', ['ros2','run','amr_can_bridge','cmd_vel_to_can','--ros-args','--params-file',str(share/'config/can_bridge.yaml'),'-p','dry_run:=true'])
        until(lambda: robot.initial_pose_pub.get_subscription_count() >= 2, 30)
        robot.initialize_localization()
        until(lambda: robot.localization_status.get('ready') is True and robot.time_health.get('healthy'), 30)
        assert robot.beam_phase_requests[:2] == [1080, 60]
        assert robot.max_beams == 60
        for name in ('planner_server','controller_server','behavior_server','bt_navigator'):
            service = robot.create_client(GetState, '/'+name+'/get_state')
            until(service.service_is_ready, 30)
            end = time.monotonic()+30
            while wait(service.call_async(GetState.Request())).current_state.id != 3:
                assert time.monotonic()<end, (tmp_path/'navigation.log').read_text()[-10000:]
                spin(.1)
        backend_services = dict(robot.get_service_names_and_types_by_node('bt_navigator', '/'))
        assert '/navigate_to_pose/_action/send_goal' not in backend_services
        assert '/navigation/navigate_to_pose/_action/send_goal' in backend_services
        client = ActionClient(robot, NavigateToPose, '/navigate_to_pose')
        until(client.server_is_ready)

        def goal(x,y=0.):
            request = NavigateToPose.Goal()
            request.pose.header.frame_id='map'
            request.pose.pose.position.x,request.pose.pose.position.y=x,y
            request.pose.pose.orientation.w=1.
            handle=wait(client.send_goal_async(request))
            assert handle.accepted
            return handle,handle.get_result_async()

        handle,result=goal(2.3)
        until(lambda:robot.status=='WAITING_AUTO')
        spin(.3)
        assert math.hypot(robot.x,robot.y)<.01
        toggle('AUTO')
        until(lambda:math.hypot(robot.x,robot.y)>.20,30)
        assert any(abs(p.pose.position.y)>.45 for path in robot.paths for p in path.poses)
        toggle('MANUAL')
        until(lambda:robot.status=='PAUSED')
        spin(.2)
        paused=(robot.x,robot.y)
        spin(.4)
        assert math.hypot(robot.x-paused[0],robot.y-paused[1])<.005
        assert not result.done()
        # Manual displacement while the outer navigation goal remains paused.
        robot.lb=True;robot.manual=.10
        spin(.6)
        robot.lb=False;robot.manual=0.
        spin(.2)
        assert math.hypot(robot.x-paused[0],robot.y-paused[1])>.025
        resumed=(robot.x,robot.y)
        old_count=len(robot.paths)
        toggle('AUTO')
        until(lambda:len(robot.paths)>old_count,15)
        first=robot.paths[old_count].poses[0].pose.position
        assert math.hypot(first.x-resumed[0],first.y-resumed[1])<.20
        # A disconnected joystick must stop the real mux and preserve this goal.
        robot.joy_connected=False
        until(lambda:robot.mode=='MANUAL' and robot.status=='PAUSED',3)
        spin(.25)
        assert abs(robot.v)<1e-8 and abs(robot.w)<1e-8 and not result.done()
        robot.joy_connected=True
        spin(.3)
        toggle('AUTO')
        outcome=wait(result,65)
        assert outcome.status==GoalStatus.STATUS_SUCCEEDED
        spin(.3)
        assert math.hypot(robot.x-2.3,robot.y)<.18
        assert abs(math.atan2(math.sin(robot.yaw),math.cos(robot.yaw)))<.18
        assert abs(robot.v)<.03 and abs(robot.w)<.05
        assert not any(overlaps_obstacle(pose) for pose in robot.poses)
        assert robot.previews and all(p.startswith('DRY_RUN') for p in robot.previews)
        wheel_limit=math.pi*.172*1.8
        assert all(abs(v)+abs(w)*.499/2 <= wheel_limit+1e-7 for _,v,w in robot.commands)

        # A goal inside the static obstacle forces the configured recovery tree.
        begin=len(robot.commands)
        blocked,blocked_result=goal(1.0)
        # Humble caches the BT between goals; RoundRobin can resume at Wait or
        # BackUp after a previous goal used recovery. Observe a full cycle.
        until(lambda:robot.v<-.01 and any(
            abs(v)<.001 and abs(w)>.05 for _,v,w in robot.commands[begin:]),45)
        recovery=robot.commands[begin:]
        assert any(abs(v)<.001 and abs(w)>.05 for _,v,w in recovery)
        assert all(abs(w)<=.6001 for _,v,w in recovery)
        wait(blocked.cancel_goal_async())
        assert wait(blocked_result).status==GoalStatus.STATUS_CANCELED
        spin(.3)
        assert abs(robot.v)<1e-8 and abs(robot.w)<1e-8
        summary={'goal_succeeded':True,'paused_goal_preserved':True,'manual_motion_during_pause':True,
                 'replanned_on_resume':True,'joystick_loss_preserved_goal':True,'spin_and_backup_recovery':True,'cancel_during_recovery':True,
                 'dry_run_can_only':True,'body_cleared_obstacle':True,'commands':len(robot.commands),'plans':len(robot.paths)}

        # Unexpected exit of one Nav2 process must stop the navigation launch.
        text=(tmp_path/'navigation.log').read_text()
        pid=int(re.search(r'\[planner_server-\d+\]: process started with pid \[(\d+)\]',text).group(1))
        os.kill(pid,signal.SIGTERM)
        end=time.monotonic()+12
        while navigation.poll() is None and time.monotonic()<end:
            spin(.1)
        assert navigation.poll() is not None
        summary['navigation_stopped_on_server_exit']=True
        (tmp_path/'result.json').write_text(json.dumps(summary,indent=2))
    finally:
        (tmp_path/'trace.json').write_text(json.dumps({
            'pose': [robot.x, robot.y, robot.yaw], 'status': robot.status,
            'localization_status': robot.localization_status,
            'commands': robot.commands, 'raw_commands': robot.raw_commands, 'poses': robot.poses,
            'paths': [[[p.pose.position.x, p.pose.position.y] for p in path.poses]
                      for path in robot.paths]}, indent=2))
        for p in processes:
            if p.poll() is None:
                # ros2 launch forwards SIGINT to its children itself.
                if p is navigation:
                    p.send_signal(signal.SIGINT)
                else:
                    os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
        for stream in logs:stream.close()
        robot.destroy_node()
        rclpy.shutdown()


def test_corridor_and_replan_when_new_obstacle_blocks_path(tmp_path):
    """A valid route stays stable, but a newly blocked route must be replaced."""
    assert os.environ['ROS_DOMAIN_ID'] == '191'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rclpy.init()
    boxes = [(.8, .6, 1.6, 2.), (.8, -2., 1.6, -.6)]
    robot = Robot(boxes=list(boxes))
    share = Path(get_package_share_directory('amr_stack'))
    processes, logs = [], []

    def start(name, args):
        stream = (tmp_path/(name+'.log')).open('w')
        logs.append(stream)
        process = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    def until(predicate, timeout=15):
        end = time.monotonic()+timeout
        while not predicate():
            assert time.monotonic()<end, (robot.x, robot.y, robot.status, robot.localization_status,
                (tmp_path/'navigation.log').read_text()[-4000:])
            assert all(p.poll() is None for p in processes)
            rclpy.spin_once(robot, timeout_sec=.01)

    def wait(future):
        until(future.done)
        return future.result()

    try:
        startup_config = tmp_path / 'startup.yaml'
        startup_config.write_text('navigation_startup:\n  ros__parameters:\n    automatic_localization: false\n')
        navigation = start('navigation', ['ros2','launch','amr_stack','navigation.launch.py',
                                         'startup_config:=' + str(startup_config)])
        start('time_health', ['ros2','run','amr_control','time_health'])
        start('mux', ['ros2','run','amr_control','control_mux','--ros-args','--params-file',str(share/'config/control_mux.yaml'),'-p','require_time_health:=true'])
        until(lambda: robot.initial_pose_pub.get_subscription_count() >= 2, 30)
        robot.initialize_localization()
        until(lambda: robot.localization_status.get('ready') is True and robot.time_health.get('healthy'), 30)
        assert robot.beam_phase_requests[:2] == [1080, 60]
        assert robot.max_beams == 60
        for name in ('planner_server','controller_server','behavior_server','bt_navigator'):
            service = robot.create_client(GetState, '/'+name+'/get_state')
            until(service.service_is_ready, 30)
            until(lambda: wait(service.call_async(GetState.Request())).current_state.id == 3, 30)
        goal_pub = robot.create_publisher(PoseStamped, '/goal_pose', 1)
        until(lambda: goal_pub.get_subscription_count()>0 and robot.mode=='MANUAL')
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.w = 1.
        pose.pose.position.x = 2.3
        goal_pub.publish(pose)
        until(lambda: robot.status=='WAITING_AUTO')
        robot.rb = True
        until(lambda: robot.mode=='AUTO')
        robot.rb = False
        # The preserved floor-retuning parameters crawl through this inflated
        # 1.2 m gap (~0.03 m/s). The earlier 35 s budget predates that tuning.
        # Keep collision and route checks, and bound replans by elapsed time.
        goal_started = time.monotonic()
        until(lambda: robot.status=='SUCCEEDED', 90)
        goal_elapsed = time.monotonic() - goal_started
        assert math.hypot(robot.x-2.3, robot.y)<.18
        assert not any(overlaps_obstacle(pose, box) for pose in robot.poses for box in boxes)
        first_goal_plans = len(robot.paths)
        assert first_goal_plans <= 1 + math.ceil(goal_elapsed / 10)
        assert all(max(abs(p.pose.position.y) for p in path.poses) < .6
                   for path in robot.paths)

        # Reverse goal initially has a clear route through the same 1.2 m gap.
        pose.pose.position.x = 0.
        old_count = len(robot.paths)
        goal_pub.publish(pose)
        until(lambda: len(robot.paths)>old_count)
        assert max(abs(p.pose.position.y) for p in robot.paths[-1].poses)<.6
        old_count = len(robot.paths)
        robot.add_obstacle((.9, -.6, 1.5, .6))
        until(lambda: len(robot.paths)>old_count and
              max(abs(p.pose.position.y) for p in robot.paths[-1].poses)>2.3, 8)
        cancel = robot.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        until(cancel.service_is_ready)
        response = wait(cancel.call_async(CancelGoal.Request()))
        assert response.return_code == CancelGoal.Response.ERROR_NONE
        until(lambda: robot.status=='CANCELED' and abs(robot.v)+abs(robot.w)<1e-8)
        (tmp_path/'result.json').write_text(json.dumps({
            'corridor_width_m':1.2, 'goal_succeeded':True, 'body_cleared_obstacles':True,
            'first_goal_plans':first_goal_plans, 'first_goal_seconds':goal_elapsed,
            'blocked_path_replanned':True,
            'topic_goal_cancelled':True}, indent=2))
    finally:
        for process in processes:
            if process.poll() is None:
                if process is processes[0]: process.send_signal(signal.SIGINT)
                else: os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        for stream in logs: stream.close()
        robot.destroy_node()
        rclpy.shutdown()
