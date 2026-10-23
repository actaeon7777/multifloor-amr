"""Run public navigation goals only in AUTO; gate all Nav2 velocity output.

MANUAL (an LB takeover) rejects new goals and aborts the active goal as
OVERRIDDEN. A goal is never resumed after AUTO returns; a new goal is required.
An E-stop (/estop/status from the CAN bridge) aborts the active goal as ESTOP;
after the operator approves a restart a new goal is required as well.

All state is owned by a SingleThreadedExecutor. Action execution awaits an rclpy
Future rather than blocking the executor. An old internal goal must reach a
terminal result before any replacement/resumed goal can be dispatched.
"""
from copy import deepcopy
from dataclasses import dataclass
import json
from math import hypot, isfinite
from time import monotonic, time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy._rclpy_pybind11 import RCLError
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import SetBool
from amr_control.time_guard import HealthLease, SessionOrder

OVERRIDE_DETAIL = ('수동 조작(LB)으로 자율주행을 중단했습니다. '
                   'LB를 놓은 뒤 목적지를 다시 지정하세요.')
ESTOP_DETAIL = ('E-stop으로 자율주행을 중단했습니다. '
                '버튼을 원위치로 돌리고 재개를 승인한 뒤 목적지를 다시 지정하세요.')


def valid_goal(goal):
    p, q = goal.pose.pose.position, goal.pose.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    norm = hypot(q.z, q.w)
    return (goal.pose.header.frame_id == 'map' and all(map(isfinite, values))
            and abs(p.z) < 1e-6 and abs(q.x) < 1e-6 and abs(q.y) < 1e-6
            and isfinite(norm) and norm > 1e-6)


def valid_command(message, max_linear, max_angular):
    v, w = message.linear, message.angular
    return (all(map(isfinite, (v.x, v.y, v.z, w.x, w.y, w.z)))
            and v.y == v.z == w.x == w.y == 0.0
            and abs(v.x) <= max_linear + 1e-9
            and abs(w.z) <= max_angular + 1e-9)


@dataclass
class Request:
    handle: object
    completion: object
    finished: bool = False


@dataclass
class Run:
    request: Request
    sent_at: float
    handle: object = None
    stopping: bool = False
    cancel_future: object = None
    last_cancel: float = 0.0
    response_failed: bool = False


class NavigationManager(Node):
    def __init__(self, **kwargs):
        super().__init__('navigation_manager', **kwargs)
        defaults = {'command_timeout': .20, 'publish_rate': 20.0,
                    'max_linear_speed': 1.0, 'max_angular_speed': .8,
                    'backend_response_timeout': 5.0, 'restart_quiet_period': .10,
                    'localization_status_timeout': 2.0, 'maneuver_lease_timeout': 30.0}
        self.settings = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if any(not isfinite(v) or v <= 0 for v in self.settings.values()):
            raise ValueError('Navigation manager timing and limits must be positive and finite')
        self.require_localization_ready = self.declare_parameter(
            'require_localization_ready', True).value
        self.require_time_health = self.declare_parameter('require_time_health', False).value
        self.time_health = HealthLease()
        self.require_estop = self.declare_parameter('require_estop', False).value
        self.estop = HealthLease()
        self.localization_order = SessionOrder()
        self.localization_is_ready = False
        self.localization_detail = '위치 준비 상태를 기다립니다.'
        self.localization_status_time = 0.0
        self.localization_generation = -1
        self.last_ready_generation = -1
        self.initial_pose_generation_barrier = -1
        self.mode = 'MANUAL'
        self.desired = None
        self.run = None
        self.requests = {}
        self.last_command = None
        self.command_time = 0.0
        self.quiet_until = 0.0
        self.terminal_status = 'IDLE'
        self.last_status = None
        # Elevator boarding/alighting: a leased, odometry-based command path that
        # does not need localization READY. Never shared with a Nav2 goal.
        self.lease_until = 0.0
        self.maneuver_command = None
        self.maneuver_time = 0.0
        self.group = ReentrantCallbackGroup()
        self.output = self.create_publisher(Twist, '/cmd_vel_auto', 1)
        self.status_output = self.create_publisher(
            String, '/navigation/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.backend = ActionClient(self, NavigateToPose, '/navigation/navigate_to_pose',
                                    callback_group=self.group)
        self.server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose', self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            handle_accepted_callback=self.on_accepted, callback_group=self.group)
        self.topic_client = ActionClient(self, NavigateToPose, '/navigate_to_pose',
                                         callback_group=self.group)
        self.create_subscription(
            String, '/control_mux/mode', self.on_mode,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.group)
        self.create_subscription(
            String, '/localization/status', self.on_localization_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.group)
        self.create_subscription(String, '/system/time_health', self.on_time_health,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL), callback_group=self.group)
        self.create_subscription(String, '/estop/status', self.on_estop_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL), callback_group=self.group)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, 1,
            callback_group=self.group)
        self.create_subscription(Twist, '/cmd_vel_navigation', self.on_command, 1,
                                 callback_group=self.group)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_topic_goal, 1,
                                 callback_group=self.group)
        self.create_subscription(Twist, '/cmd_vel_maneuver', self.on_maneuver_command, 1,
                                 callback_group=self.group)
        self.create_service(SetBool, '/navigation/maneuver_lease', self.on_maneuver_lease,
                            callback_group=self.group)
        self.timer = self.create_timer(
            1.0/self.settings['publish_rate'], self.tick, callback_group=self.group,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status()

    @staticmethod
    def key(handle):
        return bytes(handle.goal_id.uuid)

    def localization_ready(self):
        return ((not self.require_time_health or self.time_health.healthy())
            and (not self.require_localization_ready or (
            self.localization_is_ready
            and monotonic() - self.localization_status_time
            <= self.settings['localization_status_timeout'])))

    def on_time_health(self, message):
        previous = self.time_health.identity
        self.time_health.receive(message.data)
        current = self.time_health.status
        changed = previous is not None and previous != self.time_health.identity
        if self.require_time_health and (current.get('latched') or changed):
            self.initial_pose_generation_barrier = max(
                self.initial_pose_generation_barrier, self.last_ready_generation)
            self.localization_is_ready = False
            self.enforce_localization_readiness()
        self.reconcile()

    def estop_ready(self):
        return not self.require_estop or self.estop.healthy()

    def estop_stopped(self):
        """An E-stop, or no fresh status. ARMING after a closed drive gate only holds."""
        if not self.require_estop:
            return False
        fresh = self.estop.status and 0 <= monotonic()-self.estop.received < self.estop.timeout
        return not fresh or self.estop.status.get('state') not in ('READY', 'ARMING', 'BYPASS')

    def on_estop_status(self, message):
        previous = self.estop.identity
        self.estop.receive(message.data)
        changed = previous is not None and previous != self.estop.identity
        if self.require_estop and changed:
            self.enforce_estop(force=True)
        self.reconcile()

    def enforce_estop(self, force=False):
        if not (force or self.estop_stopped()):
            return
        self.lease_until = 0.0
        self.maneuver_command = None
        request = self.desired
        if self.run is not None:
            self.run.stopping = True
        if self.run is not None or request is not None:
            self.stop_output()
        if request is not None:
            self.get_logger().warning('E-stop observed; aborting navigation goal')
            self.finish(request, 'ESTOP', detail=ESTOP_DETAIL)

    def on_localization_status(self, message):
        try:
            status = json.loads(message.data)
            generation = status['generation']
            stamp = status['stamp']
            if (type(generation) is not int or generation < 0
                    or type(stamp) not in (int, float) or not isfinite(stamp)):
                raise ValueError('invalid status generation or timestamp')
            age = time() - stamp
            session = status['session']
            previous_session = self.localization_order.session
            if not -1.0 <= age <= self.settings['localization_status_timeout']:
                raise ValueError('stale localization status')
            if not self.localization_order.accept(session, generation, stamp):
                return
            if previous_session is not None and session != previous_session:
                self.localization_detail = '위치 감독기 세션이 변경되었습니다.'
                self.localization_is_ready = False
                self.enforce_localization_readiness()
                self.localization_generation = self.last_ready_generation = -1
                self.initial_pose_generation_barrier = -1
            # A delayed status from a previous initialization cannot reopen the gate.
            if generation < self.localization_generation:
                return
            if (self.last_ready_generation >= 0 and generation > self.last_ready_generation
                    and self.desired is not None):
                # Do not retain a goal across a new pose estimate even if an
                # intermediate NOT_READY heartbeat was lost.
                self.localization_detail = '초기 위치 또는 위치 검증 세대가 변경되었습니다.'
                self.localization_is_ready = False
                self.enforce_localization_readiness()
            detail = status.get('detail', '')
            self.localization_detail = detail[:1000] if isinstance(detail, str) else ''
            self.localization_generation = generation
            self.localization_status_time = monotonic() - max(0.0, age)
            self.localization_is_ready = (
                status.get('state') == 'READY' and status.get('ready') is True
                and -1.0 <= age <= self.settings['localization_status_timeout']
                and generation > self.initial_pose_generation_barrier)
            if self.localization_is_ready:
                self.last_ready_generation = generation
        except (ValueError, TypeError, KeyError):
            self.localization_detail = '위치 상태 메시지가 유효하지 않습니다.'
            self.localization_is_ready = False
        self.reconcile()

    def on_initial_pose(self, message):
        # /initialpose and status use independent DDS streams. Clear readiness here
        # immediately; a READY heartbeat for the previous pose must not reopen it.
        # Use the last READY generation, since the new unready status may arrive first.
        self.initial_pose_generation_barrier = max(
            self.initial_pose_generation_barrier, self.last_ready_generation)
        self.localization_detail = '새 초기 위치의 검증 결과를 기다립니다.'
        self.localization_is_ready = False
        self.reconcile()

    def enforce_localization_readiness(self):
        if self.localization_ready():
            return True
        request = self.desired
        if self.run is not None:
            self.run.stopping = True
        if self.run is not None or request is not None:
            self.stop_output()
        if request is not None:
            self.get_logger().warning('Localization readiness lost; aborting navigation goal')
            detail = (self.time_health.status.get('detail', '시간 상태 수신을 기다립니다.')
                      if self.require_time_health and not self.time_health.healthy() else
                      '위치 상태 수신 시간이 초과되었습니다.'
                      if monotonic()-self.localization_status_time > self.settings['localization_status_timeout']
                      else self.localization_detail or '위치 준비 조건을 충족하지 못했습니다.')
            self.finish(request, 'FAILED', detail='위치 준비 조건 해제로 주행을 중단했습니다. ' + detail)
        return False

    def lease_active(self):
        return monotonic() < self.lease_until

    def on_maneuver_lease(self, request, response):
        if not request.data:
            self.lease_until = 0.0
            self.maneuver_command = None
            self.stop_output()
            response.success, response.message = True, 'maneuver lease released'
        elif self.mode != 'AUTO':
            response.success, response.message = False, 'manual takeover is active (MANUAL)'
        elif not self.estop_ready():
            response.success, response.message = False, 'E-stop interlock is not READY'
        elif self.desired is not None or self.run is not None:
            response.success, response.message = False, 'a navigation goal is active'
        else:
            self.lease_until = monotonic() + self.settings['maneuver_lease_timeout']
            response.success, response.message = True, 'maneuver lease granted'
        self.publish_status()
        return response

    def on_maneuver_command(self, message):
        if self.lease_active() and valid_command(
                message, self.settings['max_linear_speed'], self.settings['max_angular_speed']):
            self.maneuver_command = deepcopy(message)
            self.maneuver_time = monotonic()
        else:
            self.maneuver_command = None

    def on_goal(self, goal):
        if self.mode != 'AUTO':
            self.get_logger().warning('Rejecting goal: manual takeover is active (MANUAL)')
            return GoalResponse.REJECT
        if self.lease_active():
            self.get_logger().warning('Rejecting goal: an elevator maneuver holds the command lease')
            return GoalResponse.REJECT
        if not self.estop_ready():
            self.get_logger().warning('Rejecting goal: E-stop interlock is not READY (approve restart)')
            return GoalResponse.REJECT
        if not self.enforce_localization_readiness():
            self.get_logger().warning('Rejecting goal: localization is not ready or status is stale')
            return GoalResponse.REJECT
        if not valid_goal(goal):
            self.get_logger().warning('Rejecting goal: expected a finite planar pose in map')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_accepted(self, handle):
        previous = self.desired
        self.stop_output()
        if self.run is not None:
            self.run.stopping = True
        if previous is not None:
            self.finish(previous, 'CANCELED' if previous.handle.is_cancel_requested else 'FAILED')
        request = Request(handle, Future())
        self.requests[self.key(handle)] = request
        self.desired = request
        handle.execute()
        self.reconcile()

    async def execute(self, handle):
        request = self.requests[self.key(handle)]
        result = await request.completion
        self.requests.pop(self.key(handle), None)
        return result

    def on_cancel(self, handle):
        request = self.requests.get(self.key(handle))
        if request is None or request.finished:
            return CancelResponse.REJECT
        # rclpy enters CANCELING after this callback returns. Finish on the next tick.
        if self.desired is request:
            self.desired = None
        if self.run is not None and self.run.request is request:
            self.run.stopping = True
        self.stop_output()
        return CancelResponse.ACCEPT

    def on_topic_goal(self, pose):
        if self.mode != 'AUTO':
            self.get_logger().warning('Ignoring /goal_pose: manual takeover is active (MANUAL)')
            return
        if not self.estop_ready():
            self.get_logger().warning('Ignoring /goal_pose: E-stop interlock is not READY')
            return
        if not self.enforce_localization_readiness():
            self.get_logger().warning('Ignoring /goal_pose: localization is not ready or status is stale')
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        if not valid_goal(goal):
            self.get_logger().warning('Ignoring invalid /goal_pose')
            return
        self.topic_client.send_goal_async(goal)

    def on_mode(self, message):
        mode = 'AUTO' if message.data == 'AUTO' else 'MANUAL'
        if mode != self.mode:
            self.mode = mode
            if mode != 'AUTO':
                self.lease_until = 0.0
                self.maneuver_command = None
            self.stop_output()
            if mode != 'AUTO' and self.run is not None:
                self.run.stopping = True
            self.reconcile()

    def permitted(self):
        run = self.run
        return (self.localization_ready() and self.mode == 'AUTO' and self.estop_ready()
                and run is not None and run.handle is not None
                and not run.stopping and self.desired is run.request
                and not run.request.finished and not run.request.handle.is_cancel_requested)

    def on_command(self, message):
        if (self.permitted() and valid_command(
                message, self.settings['max_linear_speed'], self.settings['max_angular_speed'])):
            self.last_command = deepcopy(message)
            self.command_time = monotonic()
        else:
            self.stop_output()

    def stop_output(self):
        self.last_command = None
        self.command_time = 0.0
        self.output.publish(Twist())

    def finish(self, request, status, detail='', backend_result=None):
        if request.finished:
            return
        request.finished = True
        if request.handle.is_cancel_requested:
            status = 'CANCELED'
        if status == 'SUCCEEDED':
            request.handle.succeed()
        elif status == 'CANCELED':
            request.handle.canceled()
        else:
            request.handle.abort()
        result = deepcopy(backend_result) if backend_result is not None else NavigateToPose.Result()
        if detail:
            result.error_msg = detail
        request.completion.set_result(result)
        if self.desired is request:
            self.desired = None
        self.terminal_status = status

    def send(self, request):
        if not self.enforce_localization_readiness():
            return
        goal = deepcopy(request.handle.request)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        q = goal.pose.pose.orientation
        norm = hypot(q.z, q.w)
        q.z, q.w = q.z/norm, q.w/norm
        run = Run(request, monotonic())
        self.run = run
        self.stop_output()
        future = self.backend.send_goal_async(
            goal, feedback_callback=lambda msg: self.on_feedback(run, msg))
        future.add_done_callback(lambda f: self.on_response(run, f))

    def on_feedback(self, run, message):
        if self.run is run and self.permitted() and run.request.handle.is_active:
            run.request.handle.publish_feedback(message.feedback)

    def on_response(self, run, future):
        try:
            run.handle = future.result()
        except Exception as error:
            self.get_logger().error(f'Nav2 goal response failed: {error}')
            self.complete_run(run, GoalStatus.STATUS_ABORTED)
            return
        if not run.handle.accepted:
            self.complete_run(run, GoalStatus.STATUS_ABORTED)
            return
        result = run.handle.get_result_async()
        result.add_done_callback(lambda f: self.on_result(run, f))
        self.reconcile()

    def on_result(self, run, future):
        try:
            response = future.result()
            status, result = response.status, response.result
        except Exception as error:
            self.get_logger().error(f'Nav2 result failed: {error}')
            status, result = GoalStatus.STATUS_ABORTED, NavigateToPose.Result(error_msg=str(error))
        self.complete_run(run, status, result)

    def complete_run(self, run, status, backend_result=None):
        if self.run is not run:
            return
        self.enforce_localization_readiness()
        self.stop_output()
        self.run = None
        self.quiet_until = monotonic() + self.settings['restart_quiet_period']
        request = run.request
        if not request.finished:
            if request.handle.is_cancel_requested:
                self.finish(request, 'CANCELED')
            elif status == GoalStatus.STATUS_SUCCEEDED:
                self.finish(request, 'SUCCEEDED')
            elif not run.stopping:
                self.finish(request, 'FAILED',
                    detail='내부 Nav2 목표가 거절되었거나 종료되었습니다.' if backend_result is None else '',
                    backend_result=backend_result)
        self.publish_status()

    def enforce_auto_mode(self):
        request = self.desired
        if self.mode == 'AUTO' or request is None:
            return
        if self.run is not None:
            self.run.stopping = True
        self.stop_output()
        self.get_logger().warning('Manual takeover (LB); aborting navigation goal')
        self.finish(request, 'OVERRIDDEN', detail=OVERRIDE_DETAIL)

    def reconcile(self):
        self.enforce_auto_mode()
        self.enforce_estop()
        ready = self.enforce_localization_readiness()
        now = monotonic()
        run = self.run
        if run is not None:
            if (self.mode != 'AUTO' or self.desired is not run.request
                    or run.request.handle.is_cancel_requested):
                run.stopping = True
            if run.handle is None and now-run.sent_at > self.settings['backend_response_timeout']:
                if not run.response_failed:
                    run.response_failed = True
                    run.stopping = True
                    self.finish(run.request, 'FAILED')
                    self.get_logger().error('Nav2 goal response timed out; blocking dispatch until resolved')
            if run.stopping:
                self.stop_output()
                if (run.handle is not None and run.handle.accepted
                        and (run.cancel_future is None or run.cancel_future.done())
                        and now-run.last_cancel >= .25):
                    run.last_cancel = now
                    run.cancel_future = run.handle.cancel_goal_async()
            # In particular, a fast OFF/ON never starts a second backend goal here.
        elif (ready and self.desired is not None and self.mode == 'AUTO' and now >= self.quiet_until
              and self.estop_ready()
              and not self.desired.handle.is_cancel_requested and self.backend.server_is_ready()):
            self.send(self.desired)
        self.publish_status()

    def publish_status(self):
        if self.lease_active():
            status = 'MANEUVER'
        elif self.desired is None:
            if self.run is not None and self.run.stopping:
                status = 'STOPPING'
            else:
                status = 'MANUAL' if self.mode != 'AUTO' else self.terminal_status
        elif self.run is not None and self.run.stopping:
            status = 'STOPPING'
        elif self.permitted():
            status = 'RUNNING'
        else:
            status = 'WAITING_NAV2'
        if status != self.last_status:
            self.status_output.publish(String(data=status))
            self.get_logger().info(f'Navigation: {status}')
            self.last_status = status

    def tick(self):
        for request in list(self.requests.values()):
            if request.handle.is_cancel_requested and not request.finished:
                # Public cancellation completes only after its backend has terminated.
                if self.run is None or self.run.request is not request:
                    self.finish(request, 'CANCELED')
        self.reconcile()
        if self.lease_active():
            fresh = (self.maneuver_command is not None
                     and monotonic()-self.maneuver_time < self.settings['command_timeout'])
            healthy = ((not self.require_time_health or self.time_health.healthy())
                       and self.estop_ready())
            self.output.publish(self.maneuver_command if fresh and healthy and self.mode == 'AUTO'
                                else Twist())
            return
        if (self.permitted() and self.last_command is not None
                and monotonic()-self.command_time < self.settings['command_timeout']):
            self.output.publish(self.last_command)
        else:
            self.output.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            try:
                node.stop_output()
            except RCLError:
                # SIGINT can invalidate the context between the check and publish.
                if rclpy.ok():
                    raise
        node.server.destroy()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
