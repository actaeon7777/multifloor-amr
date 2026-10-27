"""ROS adapter for Patrol; all motion goes through navigation_manager."""
from dataclasses import dataclass
import json
import math
import signal
from time import monotonic
from uuid import uuid4

from action_msgs.msg import GoalStatus
from amr_elevator_interfaces.msg import FloorState
from amr_vehicle_interfaces.msg import VehicleStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from amr_control.operator_hold import held_value
from amr_control.time_guard import HealthLease
from amr_stack.floor_config import load_floor_config
from amr_patrol.patrol import Patrol, valid_request_id, validate_settings
from amr_patrol.routes import default_config_path, load_patrol_config


@dataclass
class Navigation:
    serial: int
    pose: tuple
    handle: object = None
    obsolete: bool = False
    cancel_sent: bool = False
    cancel_sent_at: object = None
    feedback_xy: object = None


class PatrolManager(Node):
    def __init__(self, building=None, **kwargs):
        super().__init__('patrol_manager', **kwargs)
        self.p = {key: self.declare_parameter(key, value).value for key, value in {
            'config': str(default_config_path()), 'tick_period': .2,
            'nav_timeout_s': 180.0, 'max_retries': 1, 'max_consecutive_failures': 3,
            'min_battery_voltage': 26.1, 'goal_xy_tolerance': .3,
            'retry_delay_s': 3.0, 'battery_low_s': 10.0, 'require_estop': True,
            'cancel_result_timeout_s': 10.0,
        }.items()}
        validate_settings(self.p)
        if not isinstance(self.p['config'], str) or not self.p['config']:
            raise ValueError('config must be a non-empty path')
        self.building = building or load_floor_config()
        self.patrol = Patrol(load_patrol_config(self.p['config'], self.building), self.ros_now,
                             **{k: self.p[k] for k in ('nav_timeout_s', 'max_retries',
                                'max_consecutive_failures', 'min_battery_voltage',
                                'retry_delay_s', 'battery_low_s')})
        self.floor_state = None
        self.held = False
        self.estop = HealthLease()
        self.battery_v = None
        self.battery_received = -math.inf
        self.nav_state = ''
        self.run = None
        self.pending_pose = None
        self.failed_run = False
        self.shutting_down = False
        self.last_status = None
        self.last_status_time = -math.inf
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/patrol/status', retained)
        self.observation_pub = self.create_publisher(String, '/patrol/observation', 10)
        self.event_pub = self.create_publisher(String, '/robot/event', 10)
        self.create_subscription(String, '/patrol/command', self.on_command, 10)
        self.create_subscription(FloorState, '/floor/state', self.on_floor, retained)
        self.create_subscription(String, '/operator/hold', self.on_hold, retained)
        self.create_subscription(String, '/estop/status', self.on_estop, retained)
        self.create_subscription(VehicleStatus, '/vehicle/status', self.on_vehicle, 10)
        self.create_subscription(String, '/navigation/status', self.on_navigation, retained)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_timer(min(self.p['tick_period'], 1.0), self.tick)
        self.publish_status(force=True)

    def ros_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _publish(publisher, payload):
        publisher.publish(String(data=json.dumps(payload, ensure_ascii=False, allow_nan=False)))

    def on_floor(self, message):
        self.floor_state = message
        if self.patrol.active and (message.floor != self.patrol.route.floor
                or message.confidence not in ('CONFIRMED', 'BEACON', 'ASSUMED')
                or message.zone == FloorState.ZONE_IN_ELEVATOR):
            self.abort('현재 층을 확인할 수 없어 순찰 중단')

    def on_hold(self, message):
        held = held_value(message.data)
        if held is not None:
            self.held = held
            if held:
                self.abort('관리자 정지')

    def on_estop(self, message):
        identity = self.estop.identity
        accepted = self.estop.receive(message.data)
        if self.p['require_estop'] and self.patrol.active and (not self.estop.healthy() or (
                accepted and identity is not None and identity != self.estop.identity)):
            self.abort('E-stop')

    def on_vehicle(self, message):
        if monotonic() - self.battery_received > .5:
            self.patrol.battery(None)  # A telemetry gap breaks continuous low voltage.
        voltage = message.battery_voltage_v
        self.battery_v = voltage if message.battery_voltage_available and math.isfinite(voltage) else None
        self.battery_received = monotonic()
        self.patrol.battery(self.battery_v)
        self.drive()

    def on_navigation(self, message):
        # navigation_manager publishes a plain state string, not JSON.
        self.nav_state = message.data
        self.drive()

    def estop_healthy(self):
        return not self.p['require_estop'] or self.estop.healthy()

    def on_command(self, message):
        request_id = None
        try:
            command = json.loads(message.data)
            if not isinstance(command, dict):
                raise ValueError('잘못된 순찰 명령')
            request_id = command.get('request_id')
            if not valid_request_id(request_id):
                request_id = request_id if isinstance(request_id, str) else None
                raise ValueError('잘못된 요청 ID')
            if command.get('command') == 'stop':
                self.patrol.stop()
                self.patrol.ack(request_id, True, '순찰 정지 요청')
            elif command.get('command') == 'start':
                if self.shutting_down:
                    raise ValueError('순찰 노드 종료 중')
                self._release_stuck_run()
                floor = self.floor_state
                battery = self.battery_v if monotonic() - self.battery_received <= .5 else None
                accepted = self.patrol.start(command.get('route'), request_id,
                                  floor.floor if floor else None, battery,
                                  self.nav_state in ('RUNNING', 'MANEUVER', 'STOPPING')
                                  or self.run is not None or self.pending_pose is not None,
                                  held=self.held, estop_healthy=self.estop_healthy(),
                                  confidence=floor.confidence if floor else 'UNKNOWN',
                                  in_elevator=bool(floor and floor.zone == FloorState.ZONE_IN_ELEVATOR))
                if accepted:
                    self.failed_run = False
            else:
                raise ValueError('알 수 없는 순찰 명령')
        except (ValueError, TypeError) as error:
            self.patrol.ack(request_id, False, '잘못된 JSON 명령' if isinstance(
                error, json.JSONDecodeError) else str(error))
        self.drive()
        self.publish_status(force=True)

    def abort(self, reason):
        self.patrol.stop(reason)
        self.drive()

    def _cancel_obsolete(self, run=None):
        run = self.run if run is None else run
        if run is None:
            return
        if run is not self.run or not self.patrol.active or run.serial != self.patrol.goal_serial:
            run.obsolete = True
        if run.obsolete and run.handle is not None and not run.cancel_sent:
            run.cancel_sent = True
            run.cancel_sent_at = monotonic()
            try:
                future = run.handle.cancel_goal_async()
                future.add_done_callback(lambda f: self._cancel_response(run, f))
            except Exception as error:
                if run is self.run:
                    self.patrol.stop(f'목표 취소 실패: {error}')
                self.get_logger().error(str(error))

    def _release_stuck_run(self):
        run = self.run
        if run is None or self.patrol.active or not run.obsolete:
            return
        expired = (run.cancel_sent_at is not None
                   and monotonic() - run.cancel_sent_at >= self.p['cancel_result_timeout_s'])
        if expired or not self.client.server_is_ready():
            self.get_logger().warning('종료 결과가 없는 취소 목표 해제: '
                                      + ('취소 결과 시간 초과' if expired else '주행 서버 연결 끊김'))
            self.run = None

    def _cancel_response(self, run, future):
        try:
            response = future.result()
            # A terminal result may race with a rejected cancel. Keep the slot
            # occupied until that result arrives; never send overlapping goals.
            if not response.goals_canceling and self.run is run:
                self.get_logger().warning('목표 취소 확인 대기: 결과 수신 전 새 목표를 보내지 않습니다')
        except Exception as error:
            if self.run is run:
                self.patrol.stop(f'목표 취소 실패: {error}')
        self.drive()

    def tick(self):
        if self.patrol.active:
            if self.held:
                self.patrol.stop('관리자 정지')
            elif not self.estop_healthy():
                self.patrol.stop('E-stop')
        if monotonic() - self.battery_received > .5:
            self.patrol.battery(None)
        self.drive()

    def drive(self):
        """Drain immediate actions; keep a canceled/pending goal until terminal."""
        self._cancel_obsolete()
        self._release_stuck_run()
        self.patrol.navigation_state(
            self.nav_state, self.held,
            other_navigation=self.run is None and self.failed_run
            and self.nav_state in ('RUNNING', 'WAITING_NAV2', 'MANEUVER'))
        if not self.patrol.active:
            self.pending_pose = None
        blocked = self.run is not None and self.run.obsolete
        # A replacement waits for cancellation, without consuming its timeout.
        if not blocked or not self.patrol.active:
            for _ in range(16):
                action = self.patrol.tick(self.ros_now())
                self._cancel_obsolete()
                if action is None or action[0] == 'wait':
                    break
                kind, value = action
                if kind == 'navigate':
                    self.pending_pose = (self.patrol.goal_serial, value)
                    break
                if kind == 'publish_observation':
                    self._publish(self.observation_pub, value)
        if self.pending_pose is not None:
            serial, pose = self.pending_pose
            if serial != self.patrol.goal_serial or not self.patrol.active:
                self.pending_pose = None
            elif (self.run is None and self.client.server_is_ready()
                  and not self.patrol.manual_wait):
                self.pending_pose = None
                self._send_goal(serial, pose)
        while self.patrol.events:
            self._publish(self.event_pub, dict(id=uuid4().hex, **self.patrol.events.popleft()))
        self.publish_status()

    def _send_goal(self, serial, pose):
        self.failed_run = False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(pose[0]), float(pose[1])
        goal.pose.pose.orientation.z = math.sin(pose[2] / 2)
        goal.pose.pose.orientation.w = math.cos(pose[2] / 2)
        run = self.run = Navigation(serial, pose)
        self.patrol.navigation_dispatched(self.ros_now())
        try:
            future = self.client.send_goal_async(goal, feedback_callback=lambda m: self._feedback(run, m))
            future.add_done_callback(lambda f: self._accepted(run, f))
        except Exception as error:
            self.get_logger().error(f'목표 전송 실패: {error}')
            self._complete(run, False)

    def _feedback(self, run, message):
        pose = message.feedback.current_pose
        if pose.header.frame_id == 'map':
            run.feedback_xy = (pose.pose.position.x, pose.pose.position.y)

    def _accepted(self, run, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self._complete(run, False)
                return
            run.handle = handle
            handle.get_result_async().add_done_callback(lambda f: self._result(run, f))
            self._cancel_obsolete(run)
        except Exception as error:
            self.get_logger().error(f'목표 응답 실패: {error}')
            # If acceptance is known but the result channel failed, retain the
            # slot and request cancellation instead of starting another goal.
            if run is not self.run:
                self.get_logger().warning('이미 해제한 목표의 늦은 응답 오류')
            elif run.handle is not None:
                self.abort('주행 결과 수신 실패')
            else:
                self._complete(run, False)

    def _result(self, run, future):
        try:
            result = future.result()
            ok = result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code == 0
            if run.feedback_xy is not None:
                distance = math.hypot(run.feedback_xy[0] - run.pose[0], run.feedback_xy[1] - run.pose[1])
                ok = ok and math.isfinite(distance) and distance <= self.p['goal_xy_tolerance']
        except Exception:
            ok = False
        self._complete(run, ok)

    def _complete(self, run, ok):
        if self.run is not run:
            return
        self.run = None
        if not run.obsolete and run.serial == self.patrol.goal_serial:
            self.failed_run = not ok and run.handle is not None
            self.patrol.on_nav_result(
                ok, nav_state=self.nav_state, held=self.held,
                other_navigation=self.failed_run
                and self.nav_state in ('RUNNING', 'WAITING_NAV2', 'MANEUVER'))
        elif self.pending_pose is not None:
            # Cancellation time belongs to the old goal. A queued replacement
            # has not had a chance to reach the server yet.
            self.patrol.navigation_dispatched(self.ros_now())
        self.drive()

    def publish_status(self, force=False):
        now = self.ros_now()
        status = self.patrol.status(now)
        changed = {key: value for key, value in status.items() if key not in ('stamp', 'elapsed_s')}
        if force or changed != self.last_status or (
                self.patrol.active and monotonic() - self.last_status_time >= 1.0):
            self._publish(self.status_pub, status)
            self.last_status, self.last_status_time = changed, monotonic()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = executor = None
    previous = {}
    try:
        try:
            from rclpy.experimental import EventsExecutor
            executor = EventsExecutor()
        except ImportError:
            from rclpy.executors import SingleThreadedExecutor
            executor = SingleThreadedExecutor()
        stopped = executor.create_future()

        def request_stop(*_):
            if not stopped.done():
                stopped.set_result(None)
                executor.wake()

        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, request_stop)
        node = PatrolManager()
        executor.add_node(node)
        while rclpy.ok() and not stopped.done():
            executor.spin_until_future_complete(stopped, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None and rclpy.ok():
                node.shutting_down = True
                node.abort('순찰 노드 종료')
                deadline = monotonic() + .9
                while rclpy.ok() and node.run is not None and monotonic() < deadline:
                    executor.spin_once(timeout_sec=min(.05, max(0., deadline - monotonic())))
        finally:
            try:
                if executor is not None:
                    executor.shutdown()
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                for signum, handler in previous.items():
                    signal.signal(signum, handler)
