"""The public navigation Action is the same manager used by RViz /goal_pose."""
import math
import queue
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PolygonStamped, PoseWithCovarianceStamped
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from amr_control.time_guard import ClockMonitor, SessionOrder
from amr_vehicle_interfaces.msg import VehicleStatus

from .geometry import Grid, transform_xy, yaw
from .vehicle_status import display_snapshot, seconds


class WebBridge(Node):
    def __init__(self, shared, **kwargs):
        super().__init__('amr_web_ui', **kwargs)
        self.shared = shared
        self.shared.require_time_health = self.declare_parameter('require_time_health', False).value
        self.clock_monitor = ClockMonitor()
        self.reset_requested = False
        self.seen_epoch = shared.connection_epoch
        self.vehicle_order = SessionOrder()
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self, spin_thread=False)
        self.clock_jump = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True),
            post_callback=lambda _jump: self.request_clock_reset())
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.relocalize_client = self.create_client(Trigger, '/localization/relocalize')
        self.pending_relocalize = None
        self.initialpose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 5)
        self.goal_handle = None
        self.goal_id = None
        self.pending_scan = None
        self.pending_vectors = {}
        self.robot_stamp = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_list = []
        for name, topic in [('map', '/map'), ('local_costmap', '/local_costmap/costmap'),
                            ('global_costmap', '/global_costmap/costmap')]:
            self.subscriptions_list.append(self.create_subscription(
                OccupancyGrid, topic, lambda m, n=name: self.on_grid(n, m), latched))
            if name != 'map':
                self.subscriptions_list.append(self.create_subscription(
                    OccupancyGridUpdate, topic+'_updates', lambda m, n=name: self.on_update(n, m), 5))
        for name, topic in [('global_path', '/plan'), ('local_path', '/local_plan')]:
            self.subscriptions_list.append(self.create_subscription(
                Path, topic, lambda m, n=name: self.pending_vectors.update({n: (m, time.monotonic())}), 5))
        self.subscriptions_list.append(self.create_subscription(
            PolygonStamped, '/local_costmap/published_footprint',
            lambda m: self.pending_vectors.update({'footprint': (m, time.monotonic())}), 5))
        self.subscriptions_list.append(self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            lambda m: self.pending_vectors.update({'amcl': (m, time.monotonic())}), latched))
        self.subscriptions_list.append(self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data))
        self.subscriptions_list.append(self.create_subscription(
            String, '/navigation/status', lambda m: self.set_state(navigation=m.data), latched))
        self.subscriptions_list.append(self.create_subscription(
            String, '/localization/status', lambda m: self.shared.update_localization(m.data), latched))
        self.subscriptions_list.append(self.create_subscription(
            String, '/control_mux/mode', lambda m: self.set_state(mode=m.data), latched))
        self.subscriptions_list.append(self.create_subscription(
            String, '/system/time_health', lambda m: self.shared.time_health.receive(m.data), latched))
        self.subscriptions_list.append(self.create_subscription(
            Odometry, '/odometry/filtered', self.on_velocity, 5))
        self.subscriptions_list.append(self.create_subscription(
            VehicleStatus, '/vehicle/status', self.on_vehicle_status, QoSProfile(depth=1)))
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def set_state(self, **values):
        with self.shared.lock:
            self.shared.state.update(values)

    def request_clock_reset(self):
        # Clock callbacks may run during /clock delivery. Rebuild subscriptions
        # on the executor's next steady tick, and revoke command readiness now.
        self.reset_requested = True
        self.shared.reset_connection('시간 기준이 바뀌었습니다. 새 위치 데이터를 기다립니다.')
        self.shared.begin_localization_reset()

    def reset_transport(self):
        self.tf.clear()
        self.listener.unregister()
        self.listener = TransformListener(self.tf, self, spin_thread=False)
        self.pending_scan = None
        self.pending_vectors.clear()
        self.robot_stamp = self.goal_id = self.goal_handle = None
        if self.pending_relocalize:
            future = self.pending_relocalize[0]
            self.pending_relocalize = None
            future.cancel()
        self.seen_epoch = self.shared.connection_epoch
        self.reset_requested = False

    def on_grid(self, name, msg):
        p = msg.info.origin
        if msg.header.frame_id != 'map' and name == 'map':
            self.get_logger().warning('Map must use frame map')
            return
        try:
            grid = Grid(msg.info.width, msg.info.height, msg.info.resolution,
                        [p.position.x, p.position.y, yaw(p.orientation)], msg.data,
                        msg.header.frame_id, costmap=name != 'map')
            # Publish image, source origin and render transform as one state.
            # A rolling odom grid must never be briefly rendered in map coordinates.
            grid.info.update(self.grid_render_state(grid))
            with self.shared.lock:
                old = self.shared.grids.get(name)
                grid.version = max(old.version if old else 0, self.shared.grid_versions.get(name, 0))+1
                self.shared.grid_versions[name] = grid.version
                self.shared.grids[name] = grid
        except ValueError as error:
            self.get_logger().warning(str(error))

    def on_update(self, name, msg):
        with self.shared.lock:
            grid = self.shared.grids.get(name)
            if grid:
                if msg.header.frame_id and msg.header.frame_id != grid.info['frame']:
                    return
                try:
                    grid.update(msg.x, msg.y, msg.width, msg.height, list(msg.data))
                except ValueError as error:
                    self.get_logger().warning(str(error))

    def on_scan(self, msg):
        self.pending_scan = (msg, time.monotonic())

    def on_vehicle_status(self, msg):
        wall_stamp = seconds(msg.published_wall_stamp)
        transport_age = time.time()-wall_stamp
        if (not math.isfinite(transport_age) or not -.1<=transport_age<.5
                or not self.vehicle_order.accept(msg.publisher_session,msg.sequence,wall_stamp)):
            return
        telemetry,distance = display_snapshot(msg,max(0.,transport_age))
        with self.shared.lock:
            goal = self.shared.state.get('goal')
            active = goal and goal['status'] in ('SENDING','ACCEPTED','CANCELING')
            self.shared.telemetry.receive_vehicle(telemetry,distance,active)

    def on_velocity(self, msg):
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        if math.isfinite(v) and math.isfinite(w) and -.1 <= age <= 2:
            self.set_state(velocity=dict(linear=v, angular=w, received_at=time.monotonic()))

    def transform(self, frame, stamp=None):
        if stamp is not None:
            age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds/1e9
            if not -.1 <= age <= 2:
                raise TransformException('Message timestamp is outside the current time interval')
        if frame == 'map':
            return None
        return self.tf.lookup_transform('map', frame,
                                        Time.from_msg(stamp) if stamp else Time()).transform

    @staticmethod
    def point(x, y, transform):
        return transform_xy(x, y, transform) if transform else [x, y]

    def update_vectors(self):
        for name, (msg, received) in list(self.pending_vectors.items()):
            try:
                tf = self.transform(msg.header.frame_id, msg.header.stamp)
                if name == 'amcl':
                    p = msg.pose.pose
                    point = self.point(p.position.x, p.position.y, tf)
                    cov = [msg.pose.covariance[i] for i in (0, 1, 6, 7)]
                    # AMCL's global frame is map in this workspace.
                    if tf is not None:
                        continue
                    if not all(math.isfinite(v) for v in cov + point):
                        continue
                    value = dict(x=point[0], y=point[1], yaw=yaw(p.orientation), covariance=cov)
                else:
                    positions = [p.pose.position for p in msg.poses] if name != 'footprint' else msg.polygon.points
                    points = [self.point(p.x, p.y, tf) for p in positions
                              if math.isfinite(p.x) and math.isfinite(p.y)]
                    value = dict(points=points)
                value['received_at'] = received
                self.set_state(**{name: value})
                self.pending_vectors.pop(name, None)
            except TransformException:
                if time.monotonic()-received > 2:
                    self.pending_vectors.pop(name, None)

    def update_scan(self):
        if self.pending_scan is None:
            return
        msg, received = self.pending_scan
        try:
            tf = self.transform(msg.header.frame_id, msg.header.stamp)
            points = []
            # Keep all valid rays; decimation is not needed for a single 2D scan.
            for index, distance in enumerate(msg.ranges):
                if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                    angle = msg.angle_min + index*msg.angle_increment
                    point = self.point(distance*math.cos(angle), distance*math.sin(angle), tf)
                    intensity = msg.intensities[index] if index < len(msg.intensities) else None
                    points.append(point + [intensity if intensity is not None and math.isfinite(intensity) else None])
            self.set_state(scan=dict(points=points, received_at=received))
            self.pending_scan = None
        except TransformException:
            if time.monotonic()-received > 1:
                self.pending_scan = None

    def update_robot(self):
        try:
            msg = self.tf.lookup_transform('map', 'base_footprint', Time())
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            ros_age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
            if not -.1 <= ros_age <= 2:
                self.set_state(tf_error='로봇 위치 데이터가 지연되고 있습니다.')
                return
            p = msg.transform.translation
            if stamp != self.robot_stamp:
                self.set_state(robot=dict(x=p.x, y=p.y, yaw=yaw(msg.transform.rotation),
                                          received_at=time.monotonic()))
                self.robot_stamp = stamp
            self.set_state(tf_error=None)
        except TransformException:
            self.set_state(tf_error='로봇 위치를 기다리고 있습니다. 초기 위치를 설정해 주세요.')

    def grid_render_state(self, grid):
        try:
            original = grid.info['origin']
            tf = self.transform(grid.info['frame'])
            origin = self.point(original[0], original[1], tf)
            origin.append(original[2] + (yaw(tf.rotation) if tf else 0))
            if all(math.isfinite(value) for value in origin):
                return dict(render_origin=origin, transform_valid=True)
        except TransformException:
            pass
        return dict(render_origin=None, transform_valid=False)

    def update_grid_origins(self):
        with self.shared.lock:
            grids = list(self.shared.grids.items())
        for name, grid in grids:
            render = self.grid_render_state(grid)
            with self.shared.lock:
                # Do not attach an old grid's transform to a replacement image.
                if self.shared.grids.get(name) is grid:
                    grid.info.update(render)

    def goal_state(self, request_id, **values):
        with self.shared.lock:
            goal = self.shared.state.get('goal')
            if goal and goal['request_id'] == request_id:
                goal.update(values)

    def process_command(self, command):
        kind, request_id, pose = command['kind'], command['request_id'], command['pose']
        error = self.shared.readiness_error(kind, request_id)
        if error:
            self.shared.complete(request_id, 'FAILED', error)
            if kind in ('initialpose', 'relocalize'):
                self.set_state(localization_reset_pending=False)
            return
        if pose is not None or kind == 'relocalize':
            with self.shared.lock:
                grid = self.shared.grids.get('map')
                if ((pose is not None and not grid)
                        or (grid is not None and grid.version != command['map_version'])):
                    self.shared.complete(request_id, 'FAILED', '지도가 바뀌었습니다. 다시 선택해 주세요.')
                    self.goal_state(request_id, status='FAILED')
                    if kind in ('initialpose', 'relocalize'):
                        self.set_state(localization_reset_pending=False)
                    return
        if kind == 'goal':
            if not self.client.server_is_ready():
                self.shared.complete(request_id, 'FAILED', '내비게이션에 연결되지 않았습니다.')
                self.goal_state(request_id, status='FAILED')
                return
            self.goal_id = request_id
            self.goal_handle = None
            self.set_state(goal=dict(x=pose[0], y=pose[1], yaw=pose[2], request_id=request_id,
                                     kind=kind, status='SENDING'))
            with self.shared.lock:
                self.shared.telemetry.begin_journey(request_id)
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = pose[:2]
            goal.pose.pose.orientation.z = math.sin(pose[2]/2)
            goal.pose.pose.orientation.w = math.cos(pose[2]/2)
            future = self.client.send_goal_async(goal, feedback_callback=lambda m: self.on_feedback(request_id, m))
            future.add_done_callback(lambda f: self.on_goal_response(request_id, f))
        elif kind == 'initialpose':
            if not self.initialpose.get_subscription_count():
                self.shared.complete(request_id, 'FAILED', '위치추정 노드에 연결되지 않았습니다.')
                self.set_state(localization_reset_pending=False)
                return
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
            msg.pose.pose.orientation.z = math.sin(pose[2]/2)
            msg.pose.pose.orientation.w = math.cos(pose[2]/2)
            msg.pose.covariance[0] = msg.pose.covariance[7] = .25
            msg.pose.covariance[35] = 0.06853891909122467
            self.shared.begin_localization_reset()
            self.initialpose.publish(msg)
            self.shared.complete(request_id, 'PUBLISHED', '초기 위치를 전송했습니다. 위치 확인 완료를 기다려 주세요.')
        elif kind == 'relocalize':
            if not self.relocalize_client.service_is_ready():
                self.set_state(localization_reset_pending=False)
                self.shared.complete(request_id, 'FAILED', '자동 위치 찾기 서비스에 연결되지 않았습니다.')
                return
            self.shared.begin_localization_reset()
            future = self.relocalize_client.call_async(Trigger.Request())
            self.pending_relocalize = (future, request_id, time.monotonic())
            future.add_done_callback(lambda f: self.on_relocalize_response(request_id, f))
        elif kind == 'cancel':
            if not self.goal_handle or not self.goal_handle.accepted:
                self.shared.complete(request_id, 'FAILED', '취소할 목표가 없습니다.')
                return
            goal_id = self.goal_id
            self.goal_state(goal_id, status='CANCELING')
            self.goal_handle.cancel_goal_async().add_done_callback(
                lambda f: self.on_cancel_response(request_id, goal_id, f))

    def on_relocalize_response(self, request_id, future):
        if self.pending_relocalize is None or self.pending_relocalize[0] is not future:
            return
        self.pending_relocalize = None
        try:
            result = future.result()
            if not result.success:
                self.set_state(localization_reset_pending=False)
            self.shared.complete(request_id, 'STARTED' if result.success else 'FAILED',
                                 result.message or ('자동 위치 찾기를 시작했습니다. 위치 확인 완료를 기다려 주세요.'
                                                    if result.success else '자동 위치 찾기가 거절되었습니다.'))
        except Exception as error:
            self.shared.complete(request_id, 'FAILED', f'자동 위치 찾기 응답을 확인하지 못했습니다: {error}')

    def on_goal_response(self, request_id, future):
        if self.goal_id != request_id:
            return
        try:
            handle = future.result()
            if not handle.accepted:
                self.goal_state(request_id, status='REJECTED')
                self.shared.complete(request_id, 'REJECTED', '목표가 거절되었습니다.')
                return
            if self.goal_id == request_id:
                self.goal_handle = handle
            self.goal_state(request_id, status='ACCEPTED')
            self.shared.complete(request_id, 'ACCEPTED', '목표가 접수되었습니다.')
            handle.get_result_async().add_done_callback(lambda f: self.on_result(request_id, f))
        except Exception as error:
            self.goal_state(request_id, status='FAILED')
            self.shared.complete(request_id, 'FAILED', f'목표 전송 실패: {error}')

    def on_feedback(self, request_id, msg):
        distance = msg.feedback.distance_remaining
        if math.isfinite(distance) and distance >= 0:
            self.goal_state(request_id, distance_remaining=distance, feedback_received_at=time.monotonic())

    def on_result(self, request_id, future):
        if self.goal_id != request_id:
            return
        try:
            response = future.result()
            status = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(
                response.status, 'FAILED')
            detail = response.result.error_msg
            self.goal_state(request_id, status=status, error_code=response.result.error_code,
                            error_message=detail)
            self.shared.complete(request_id, status, detail)
        except Exception as error:
            self.goal_state(request_id, status='FAILED')
            self.shared.complete(request_id, 'FAILED', str(error))

    def on_cancel_response(self, request_id, goal_id, future):
        if self.goal_id != goal_id:
            return
        try:
            if future.result().goals_canceling:
                self.shared.complete(request_id, 'CANCEL_REQUESTED', '취소 요청을 처리하고 있습니다.')
            else:
                self.shared.complete(request_id, 'FAILED', '취소가 거절되었거나 목표가 이미 종료되었습니다.')
                with self.shared.lock:
                    goal = self.shared.state.get('goal')
                    if goal and goal['request_id'] == goal_id and goal['status'] == 'CANCELING':
                        goal['status'] = 'ACCEPTED'
        except Exception as error:
            self.shared.complete(request_id, 'FAILED', str(error))
            self.goal_state(goal_id, status='ACCEPTED')

    def tick(self):
        if self.clock_monitor.observe(self.get_clock().now().nanoseconds/1e9, time.monotonic(),
                                      self.get_parameter('use_sim_time').value):
            if not self.reset_requested:
                self.request_clock_reset()
        if self.reset_requested or self.seen_epoch != self.shared.connection_epoch:
            self.reset_transport()
        self.set_state(action_ready=self.client.server_is_ready())
        self.update_robot()
        self.update_vectors()
        self.update_scan()
        self.update_grid_origins()
        if self.pending_relocalize and time.monotonic()-self.pending_relocalize[2] > 5:
            future, request_id, _received = self.pending_relocalize
            self.pending_relocalize = None
            future.cancel()
            self.shared.complete(request_id, 'FAILED', '자동 위치 찾기 응답이 지연됩니다. 상태를 확인해 주세요. 자동 재전송하지 않습니다.')
        try:
            self.process_command(self.shared.commands.get_nowait())
        except queue.Empty:
            pass
