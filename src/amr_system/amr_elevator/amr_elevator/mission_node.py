"""floor_mission_manager: navigate to a pose on any floor, riding the elevator.

Real vehicle and simulation share this node. Flow (docs/ELEVATOR.md 4.4):
NAV_TO_HALL -> CALL_ELEVATOR -> WAIT_DOOR_OPEN -> BOARDING -> IN_CAR_ALIGN ->
SELECT_FLOOR -> RIDING -> SWITCH_MAP -> ALIGHTING -> RELOCALIZE -> NAV_TO_GOAL.

Floor legs use the existing /navigate_to_pose (navigation_manager).  Door
crossings use a leased odometry maneuver (/navigation/maneuver_lease) while
localization is suspended (/localization/suspend).  After the ride the target
floor map is loaded and the existing manual /initialpose verification restores
READY.  Any unrecoverable condition stops in FAILED_SAFE for the operator.
"""

from dataclasses import dataclass, field
import json
import math
from pathlib import Path

from action_msgs.msg import GoalStatus
from amr_elevator_interfaces.action import NavigateToFloorPose
from amr_elevator_interfaces.msg import (
    ElevatorBeacon, ElevatorCommand, ElevatorDoorState, FloorEvent, FloorState,
)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from amr_elevator.geometry import depth_beyond, wrap
from amr_elevator.maneuver import DONE, TURN, LineManeuver, ManeuverLimits, travel_clearance

ACTIVE = ('NAV_TO_HALL', 'CALL_ELEVATOR', 'WAIT_DOOR_OPEN', 'BOARDING', 'BACKING_OUT',
          'IN_CAR_ALIGN', 'SELECT_FLOOR', 'RIDING', 'SWITCH_MAP', 'ALIGHTING', 'RETURN_TO_CAR',
          'RELOCALIZE', 'NAV_TO_GOAL')
HALF_LENGTH = .32


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


@dataclass
class PendingAck:
    sequence: int
    location: int
    command: tuple
    sent: float
    attempts: int = 1
    acked: bool = False
    # A late acknowledgement of an earlier press of the same button still counts.
    sequences: set = field(default_factory=set)


@dataclass
class Mission:
    handle: object
    completion: Future
    floor: str
    pose: tuple
    room: str
    start_floor: str
    phase: str = 'IDLE'
    phase_since: float = 0.
    started: float = 0.
    detail: str = ''
    counters: dict = field(default_factory=dict)
    timings: dict = field(default_factory=dict)
    finished: bool = False


class FloorMissionManager(Node):
    def __init__(self, building=None, **kwargs):
        super().__init__('floor_mission_manager', **kwargs)
        from amr_stack.floor_config import load_floor_config
        p = {k: self.declare_parameter(k, v).value for k, v in {
            'floors_config': '', 'elevator_id': 'E1', 'simulation': False,
            'use_door_open_button': True, 'ack_timeout': 3., 'ack_attempts': 3,
            'door_wait_timeout': 120., 'ride_timeout': 180., 'relocalize_timeout': 30.,
            'boarding_timeout': 30., 'max_boarding_attempts': 3, 'lease_renew_period': 10.,
            'initial_pose_xy_variance': .0225, 'initial_pose_yaw_variance': .0076,
            'maneuver_speed': .35, 'tick_period': .05, 'floor_state_timeout': 3.,
            'localization_recovery_timeout': 45.,
        }.items()}
        self.p = p
        self.building = building or load_floor_config(p['floors_config'] or None)
        self.elevator_id = p['elevator_id']
        self.group = ReentrantCallbackGroup()
        self.mission = None
        self.floor_state = None
        self.floor_state_time = -math.inf
        self.door = ElevatorDoorState()
        self.pose = None
        self.scan = None
        self.mode = 'MANUAL'
        self.localization = {}
        self.sequence = 0
        self.pending = None
        self.maneuver = None
        self.lease_renewed = -math.inf
        self.nav = None            # {'future', 'handle', 'status'}
        self.calls = {}            # name -> Future
        self.last_status = None
        self.seed_after_resume = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'floor_mission/status', latched)
        self.command_pub = self.create_publisher(ElevatorCommand, 'elevator/command', 10)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel_maneuver', 1)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.create_subscription(FloorState, 'floor/state', self.on_floor_state, latched)
        self.create_subscription(FloorEvent, 'floor/events', self.on_floor_event, 20)
        self.create_subscription(ElevatorDoorState, 'elevator/door', self.on_door, 10)
        self.create_subscription(ElevatorBeacon, 'elevator/beacon', self.on_beacon, 50)
        self.create_subscription(PoseStamped, 'floor/pose_estimate', self.on_pose, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, 'control_mux/mode', self.on_mode, latched)
        self.create_subscription(String, 'localization/status', self.on_localization, latched)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.group)
        self.load_map_client = self.create_client(LoadMap, 'map_server/load_map', callback_group=self.group)
        self.suspend_client = self.create_client(SetBool, 'localization/suspend', callback_group=self.group)
        self.lease_client = self.create_client(SetBool, 'navigation/maneuver_lease', callback_group=self.group)
        self.relocalize_client = self.create_client(Trigger, 'localization/relocalize', callback_group=self.group)
        self.server = ActionServer(
            self, NavigateToFloorPose, 'floor_mission/navigate', self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            handle_accepted_callback=self.on_accepted, callback_group=self.group)
        self.create_timer(p['tick_period'], self.tick, callback_group=self.group)
        self.publish_status(force=True)

    # Helpers ------------------------------------------------------------------

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stop_geometry(self, floor):
        return self.building.floor(floor).elevators[self.elevator_id]

    def geometry(self, floor):
        return self.building.elevator_geometry(floor, self.elevator_id)

    def current_floor(self):
        s = self.floor_state
        return '' if s is None else s.floor

    def set_phase(self, phase, detail=''):
        m = self.mission
        now = self.now()
        if m.phase in ACTIVE:
            m.timings[m.phase] = m.timings.get(m.phase, 0.) + now - m.phase_since
        m.phase, m.phase_since, m.detail = phase, now, detail
        self.maneuver = None
        self.get_logger().info(f'Mission {phase}: {detail}'.rstrip(': '))
        self.publish_status(force=True)

    def elapsed(self):
        return self.now() - self.mission.phase_since

    def count(self, key):
        counters = self.mission.counters
        counters[key] = counters.get(key, 0) + 1
        return counters[key]

    def call(self, name, client, request):
        if not client.service_is_ready():
            return False
        self.calls[name] = client.call_async(request)
        return True

    def result(self, name):
        """None while pending; the response (or False on error) once done."""
        future = self.calls.get(name)
        if future is None or not future.done():
            return None
        self.calls.pop(name)
        try:
            return future.result()
        except Exception:  # noqa: BLE001
            return False

    def send_elevator(self, command, floor_code=0, direction=ElevatorCommand.DIR_NONE, attempts=1,
                      earlier=()):
        self.sequence = (self.sequence + 1) & 0xff
        message = ElevatorCommand(command=command, floor_code=floor_code, direction=direction,
                                  sequence=self.sequence)
        message.header.stamp = self.get_clock().now().to_msg()
        self.command_pub.publish(message)
        location = (ElevatorBeacon.LOCATION_HALL if command == ElevatorCommand.CMD_HALL_CALL
                    else ElevatorBeacon.LOCATION_CAR)
        self.pending = PendingAck(self.sequence, location, (command, floor_code, direction), self.now(), attempts,
                                  sequences={self.sequence, *earlier})

    def retry_pending(self, failure_code):
        """Resend an unacknowledged command; True while still trying."""
        pending = self.pending
        if pending is None or pending.acked:
            return True
        if self.now() - pending.sent < self.p['ack_timeout']:
            return True
        if pending.attempts >= self.p['ack_attempts']:
            self.fail(failure_code, f'no acknowledgement after {pending.attempts} presses')
            return False
        self.count('button_retries')
        self.send_elevator(*pending.command, attempts=pending.attempts + 1, earlier=pending.sequences)
        return True

    def publish_twist(self, linear=0., angular=0.):
        message = Twist()
        message.linear.x, message.angular.z = float(linear), float(angular)
        self.twist_pub.publish(message)

    def renew_lease(self):
        if self.now() - self.lease_renewed >= self.p['lease_renew_period'] and 'lease' not in self.calls:
            if self.call('lease', self.lease_client, SetBool.Request(data=True)):
                self.lease_renewed = self.now()
        response = self.result('lease')
        if response is False or (response is not None and not response.success):
            self.lease_renewed = -math.inf  # retry next tick (e.g. after MANUAL -> AUTO)

    def release_lease(self):
        self.publish_twist()
        if self.lease_client.service_is_ready():
            self.lease_client.call_async(SetBool.Request(data=False))
        self.lease_renewed = -math.inf

    def door_state(self):
        return self.door.state

    def depth(self, floor, line):
        stop = self.stop_geometry(floor)
        return depth_beyond(self.pose, line, stop.car_inward_yaw)

    def clearance(self, reverse):
        if self.scan is None:
            return math.inf
        return travel_clearance(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                                reverse, HALF_LENGTH)

    def drive(self):
        """Advance the current maneuver; True once done."""
        if self.maneuver is None or self.pose is None:
            self.publish_twist()
            return False
        if self.mode != 'AUTO':
            self.publish_twist()
            return False
        self.renew_lease()
        linear, angular = self.maneuver.command(
            self.pose, self.p['tick_period'], self.clearance(self.maneuver.reverse))
        self.publish_twist(linear, angular)
        return self.maneuver.phase == DONE

    def limits(self):
        return ManeuverLimits(max_speed=self.p['maneuver_speed'])

    # Nav2 legs -----------------------------------------------------------------

    def start_nav(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose[0], pose[1]
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        self.nav = {'status': 'SENDING', 'handle': None}
        if not self.nav_client.server_is_ready():
            self.nav['status'] = 'UNAVAILABLE'
            return
        future = self.nav_client.send_goal_async(goal)
        nav = self.nav

        def accepted(f):
            try:
                handle = f.result()
            except Exception:  # noqa: BLE001
                nav['status'] = 'FAILED'
                return
            if not handle.accepted:
                nav['status'] = 'REJECTED'
                return
            nav['handle'] = handle
            nav['status'] = 'RUNNING'
            handle.get_result_async().add_done_callback(done)

        def done(f):
            try:
                status = f.result().status
            except Exception:  # noqa: BLE001
                status = GoalStatus.STATUS_ABORTED
            nav['status'] = 'SUCCEEDED' if status == GoalStatus.STATUS_SUCCEEDED else 'FAILED'
        future.add_done_callback(accepted)

    def cancel_nav(self):
        if self.nav and self.nav.get('handle') is not None and self.nav['status'] == 'RUNNING':
            self.nav['handle'].cancel_goal_async()
        self.nav = None

    def nav_leg(self, pose, next_phase, failure):
        """Drive a Nav2 leg; a localization drop pauses the leg instead of using up retries.

        navigation_startup withdraws READY on tracking faults and re-verifies once the
        robot stops (existing behaviour); navigation_manager then aborts the goal.
        """
        m = self.mission
        waiting = m.counters.get('_localization_wait')
        if waiting is not None:
            if self.ready():
                m.counters.pop('_localization_wait')
            elif self.now() - waiting > self.p['localization_recovery_timeout']:
                self.fail('LOCALIZATION_LOST', 'localization did not recover during a navigation leg')
            return
        if self.nav is None:
            self.start_nav(pose)
            return
        status = self.nav['status']
        if status == 'SUCCEEDED':
            self.nav = None
            self.set_phase(next_phase)
            return True
        if status in ('FAILED', 'REJECTED', 'UNAVAILABLE'):
            # navigation_manager aborts on a localization drop; its status may arrive a moment later.
            failed_at = m.counters.setdefault('_nav_failed_at', self.now())
            if self.ready() and self.now() - failed_at < 1.:
                return
            m.counters.pop('_nav_failed_at')
            self.nav = None
            if not self.ready():
                self.count('localization_waits')
                m.counters['_localization_wait'] = self.now()
            elif self.count(failure) > 2:
                self.fail(failure, 'navigation leg failed')

    # Inputs ---------------------------------------------------------------------

    def on_floor_state(self, message):
        self.floor_state = message
        self.floor_state_time = self.now()

    def on_floor_event(self, message):
        if self.mission is not None and self.mission.phase in ACTIVE:
            events = self.mission.counters.setdefault('floor_events', [])
            events.append(message.event)
            if message.event == 'RULE_VIOLATION':
                self.fail('FLOOR_RULE_VIOLATION', message.detail)

    def on_door(self, message):
        self.door = message

    def on_pose(self, message):
        self.pose = (message.pose.position.x, message.pose.position.y, yaw_of(message.pose.orientation))

    def on_scan(self, message):
        self.scan = message

    def on_mode(self, message):
        self.mode = 'AUTO' if message.data == 'AUTO' else 'MANUAL'

    def on_localization(self, message):
        try:
            self.localization = json.loads(message.data)
        except ValueError:
            self.localization = {}

    def on_beacon(self, message):
        pending = self.pending
        if (pending is not None and message.event == ElevatorBeacon.EVENT_BUTTON_ACK
                and message.location == pending.location and message.ack_sequence in pending.sequences):
            pending.acked = True

    def ready(self):
        return self.localization.get('state') == 'READY' and self.localization.get('ready') is True

    # Action ---------------------------------------------------------------------

    def on_goal(self, request):
        if self.mission is not None and not self.mission.finished:
            self.get_logger().warning('Rejecting floor mission: another mission is active')
            return GoalResponse.REJECT
        try:
            floor = self.building.floor(request.floor)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return GoalResponse.REJECT
        if not self.p['simulation'] and not floor.real_enabled:
            self.get_logger().warning(f'Rejecting {request.floor}: real_enabled is false (provisional map)')
            return GoalResponse.REJECT
        state = self.floor_state
        if (state is None or state.zone != FloorState.ZONE_OUTSIDE
                or state.confidence not in ('BEACON', 'CONFIRMED') or not state.floor):
            self.get_logger().warning('Rejecting floor mission: current floor is not confirmed')
            return GoalResponse.REJECT
        pose = request.pose.pose
        values = (pose.position.x, pose.position.y, pose.orientation.z, pose.orientation.w)
        if request.pose.header.frame_id != 'map' or not all(math.isfinite(v) for v in values):
            return GoalResponse.REJECT
        if not self.ready():
            self.get_logger().warning('Rejecting floor mission: localization is not READY')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_accepted(self, handle):
        request = handle.request
        pose = request.pose.pose
        self.mission = Mission(
            handle, Future(), request.floor,
            (pose.position.x, pose.position.y, yaw_of(pose.orientation)),
            request.destination_id, self.current_floor(), started=self.now())
        handle.execute()
        self.set_phase('NAV_TO_GOAL' if request.floor == self.current_floor() else 'NAV_TO_HALL',
                       f'{self.mission.start_floor} -> {request.floor} {request.destination_id}'.strip())

    async def execute(self, handle):
        return await self.mission.completion

    def on_cancel(self, handle):
        return CancelResponse.ACCEPT

    def finish(self, outcome, error_code='', message=''):
        m = self.mission
        if m is None or m.finished:
            return
        now = self.now()
        if m.phase in ACTIVE:
            m.timings[m.phase] = m.timings.get(m.phase, 0.) + now - m.phase_since
        m.finished = True
        self.cancel_nav()
        self.maneuver = None
        self.pending = None
        result = NavigateToFloorPose.Result(success=outcome == 'SUCCEEDED', error_code=error_code,
                                            message=message or json.dumps({'timings': m.timings}))
        if outcome == 'SUCCEEDED':
            m.handle.succeed()
        elif outcome == 'CANCELED':
            m.handle.canceled()
        else:
            m.handle.abort()
        m.phase, m.detail, m.phase_since = outcome if outcome != 'FAILED' else 'FAILED_SAFE', \
            f'{error_code} {message}'.strip(), now
        m.completion.set_result(result)
        self.publish_status(force=True)

    def fail(self, code, detail=''):
        if self.mission is None or self.mission.finished:
            return
        self.get_logger().error(f'Mission failed safe: {code} {detail}')
        self.release_lease()
        self.finish('FAILED', code, detail)

    def publish_status(self, force=False):
        m = self.mission
        state = self.floor_state
        status = {
            'phase': 'IDLE' if m is None else m.phase,
            'detail': '' if m is None else m.detail,
            'goal_floor': '' if m is None else m.floor,
            'goal_room': '' if m is None else m.room,
            'start_floor': '' if m is None else m.start_floor,
            'current_floor': '' if state is None else state.floor,
            'zone': 0 if state is None else state.zone,
            'confidence': '' if state is None else state.confidence,
            'door': self.door.state,
            'counters': {} if m is None else {k: v for k, v in m.counters.items()
                                              if k != 'floor_events' and not k.startswith('_')},
            'timings': {} if m is None else m.timings,
            'elapsed': 0. if m is None else self.now() - m.started,
        }
        key = {k: v for k, v in status.items() if k != 'elapsed'}
        if force or key != self.last_status:
            self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
            self.last_status = key
        if m is not None and not m.finished:
            feedback = NavigateToFloorPose.Feedback(phase=m.phase, current_floor=status['current_floor'],
                                                    zone=status['zone'], detail=m.detail)
            m.handle.publish_feedback(feedback)

    # State machine ------------------------------------------------------------------

    def tick(self):
        m = self.mission
        if self.seed_after_resume:
            response = self.result('suspend')
            if response is not None:
                self.seed_after_resume = False
                if response and response.success:
                    self.publish_initial_pose()
        if m is None or m.finished:
            return
        if m.handle.is_cancel_requested:
            self.cancel()
            return
        if self.now() - self.floor_state_time > self.p['floor_state_timeout']:
            # elevator_monitor publishes at least every second; without it nothing is safe.
            self.fail('FLOOR_STATE_LOST', 'no /floor/state from elevator_monitor')
            self.publish_status()
            return
        try:
            getattr(self, 'phase_' + m.phase.lower())()
        except Exception as error:  # noqa: BLE001 - never leave the robot moving
            self.get_logger().error(f'Mission error in {m.phase}: {error!r}')
            self.fail('INTERNAL_ERROR', repr(error))
        self.publish_status()

    def cancel(self):
        m = self.mission
        inside = self.floor_state is not None and self.floor_state.zone == FloorState.ZONE_IN_ELEVATOR
        self.release_lease()
        if not inside and m.phase in ('BOARDING', 'BACKING_OUT', 'WAIT_DOOR_OPEN', 'ALIGHTING', 'RELOCALIZE'):
            # Outside the car again: give localization back, seeded at the carried pose.
            self.seed_after_resume = self.call('suspend', self.suspend_client, SetBool.Request(data=False))
        self.finish('CANCELED', 'CANCELED',
                    'canceled inside the car: localization stays suspended' if inside else 'canceled')

    def phase_nav_to_hall(self):
        stop = self.stop_geometry(self.mission.start_floor if not self.current_floor() else self.current_floor())
        self.nav_leg(stop.hall_wait_pose, 'CALL_ELEVATOR', 'NAV_TO_HALL_FAILED')

    def phase_call_elevator(self):
        m = self.mission
        here = self.building.floor(self.current_floor() or m.start_floor)
        if self.door_state() == ElevatorDoorState.OPEN:
            self.pending = None
            self.set_phase('WAIT_DOOR_OPEN', 'car already here')
            return
        if self.pending is None or self.pending.command[0] != ElevatorCommand.CMD_HALL_CALL:
            up = self.building.floor(m.floor).level > here.level
            self.send_elevator(ElevatorCommand.CMD_HALL_CALL, here.beacon_floor_code,
                               ElevatorCommand.DIR_UP if up else ElevatorCommand.DIR_DOWN)
            return
        if self.pending.acked:
            self.set_phase('WAIT_DOOR_OPEN')
            return
        self.retry_pending('NO_HALL_ACK')

    def phase_wait_door_open(self):
        m = self.mission
        if self.door_state() == ElevatorDoorState.OPEN:
            prepared = m.counters.get('_prepared')
            if not prepared:
                if 'suspend' not in self.calls and 'lease' not in self.calls:
                    self.call('suspend', self.suspend_client, SetBool.Request(data=True))
                    self.call('lease', self.lease_client, SetBool.Request(data=True))
                    m.counters['_prepared'] = 'pending'
                return
            if prepared == 'pending':
                if 'suspend' in self.calls or 'lease' in self.calls:
                    suspend, lease = self.result('suspend'), self.result('lease')
                    if suspend is not None:
                        m.counters['_suspend_ok'] = bool(suspend and suspend.success)
                    if lease is not None:
                        m.counters['_lease_ok'] = bool(lease and lease.success)
                    return
                if not (m.counters.pop('_suspend_ok', False) and m.counters.pop('_lease_ok', False)):
                    m.counters.pop('_prepared', None)
                    self.fail('PREPARE_FAILED', 'localization suspend or maneuver lease refused')
                    return
                m.counters.pop('_prepared', None)
                self.lease_renewed = self.now()
                if self.p['use_door_open_button']:
                    self.send_elevator(ElevatorCommand.CMD_DOOR_OPEN)
                floor = self.current_floor()
                geometry = self.geometry(floor)
                self.set_phase('BOARDING')
                self.maneuver = LineManeuver(geometry.car_pose, final_yaw=geometry.car_pose[2],
                                             limits=self.limits())
            return
        if self.elapsed() > self.p['door_wait_timeout']:
            if self.count('recalls') > 1:
                self.fail('DOOR_NOT_OPENED', 'elevator door did not open')
            else:
                self.pending = None
                self.set_phase('CALL_ELEVATOR', 'calling again')

    def phase_boarding(self):
        m = self.mission
        floor = self.current_floor() or m.start_floor
        geometry = self.geometry(floor)
        front = None if self.pose is None else self.depth(floor, geometry.hall_door) + HALF_LENGTH
        closing = self.door_state() in (ElevatorDoorState.CLOSING, ElevatorDoorState.CLOSED)
        if closing and front is not None and front < 0.:
            self.count('boarding_aborts')
            stop = self.stop_geometry(floor)
            self.set_phase('BACKING_OUT', 'door closing before the threshold')
            self.maneuver = LineManeuver(stop.hall_wait_pose, reverse=True, limits=self.limits())
            return
        self.drive()
        inside = self.floor_state is not None and self.floor_state.zone == FloorState.ZONE_IN_ELEVATOR
        if self.maneuver is not None and self.maneuver.phase in (TURN, DONE) and inside:
            maneuver = self.maneuver
            self.set_phase('IN_CAR_ALIGN')
            self.maneuver = maneuver
            return
        if self.elapsed() > self.p['boarding_timeout']:
            self.fail('BOARDING_TIMEOUT', 'could not enter the car')

    def phase_backing_out(self):
        if not self.drive():
            if self.elapsed() > self.p['boarding_timeout']:
                self.fail('BACK_OUT_TIMEOUT', 'could not return to the hall')
            return
        if self.count('boarding_attempts') >= self.p['max_boarding_attempts']:
            self.fail('BOARDING_FAILED', 'door closed before boarding repeatedly')
            return
        self.release_lease()
        self.mission.counters['_after_relocalize'] = 'CALL_ELEVATOR'
        self.set_phase('RELOCALIZE', 'back in the hall')

    def phase_in_car_align(self):
        if self.maneuver is None:
            geometry = self.geometry(self.current_floor() or self.mission.start_floor)
            self.maneuver = LineManeuver(geometry.car_pose, final_yaw=geometry.car_pose[2], limits=self.limits())
        if self.drive():
            self.publish_twist()
            self.set_phase('SELECT_FLOOR')
        elif self.elapsed() > 20.:
            self.fail('ALIGN_TIMEOUT', 'could not turn in the car')

    def phase_select_floor(self):
        self.renew_lease()
        self.publish_twist()
        target = self.building.floor(self.mission.floor)
        if self.pending is None or self.pending.command[0] != ElevatorCommand.CMD_CAR_FLOOR:
            self.send_elevator(ElevatorCommand.CMD_CAR_FLOOR, target.beacon_floor_code)
            return
        if self.pending.acked:
            self.set_phase('RIDING')
            return
        self.retry_pending('NO_CAR_ACK')

    def phase_riding(self):
        m = self.mission
        self.renew_lease()
        self.publish_twist()
        state = self.floor_state
        if state is None:
            return
        door_open = self.door_state() == ElevatorDoorState.OPEN
        if state.floor == m.floor and door_open:
            if state.confidence in ('BEACON', 'CONFIRMED'):
                if self.p['use_door_open_button']:
                    self.send_elevator(ElevatorCommand.CMD_DOOR_OPEN)
                self.set_phase('SWITCH_MAP')
                return
            if state.confidence == 'ASSUMED':
                waited = m.counters.setdefault('_assumed_since', self.now())
                if self.now() - waited > 6.:
                    self.fail('ARRIVAL_UNCONFIRMED', 'no hall beacon at the arrival floor')
                elif m.counters.get('_assumed_hold') is None and self.p['use_door_open_button']:
                    m.counters['_assumed_hold'] = True
                    self.send_elevator(ElevatorCommand.CMD_DOOR_OPEN)
                return
        if state.confidence == 'TRANSIT':
            m.counters['_last_transit'] = self.now()
        last_motion = m.counters.get('_last_transit', m.phase_since)
        if state.floor and state.floor != m.floor and door_open and self.now() - last_motion > 20.:
            # Stopped at another floor and the car is not moving on: press the target again.
            if self.count('car_repress') > 2:
                self.fail('RIDE_STALLED', f'car stays at {state.floor}')
                return
            m.counters['_last_transit'] = self.now()
            self.pending = None
            self.send_elevator(ElevatorCommand.CMD_CAR_FLOOR, self.building.floor(m.floor).beacon_floor_code)
        if self.elapsed() > self.p['ride_timeout']:
            self.fail('RIDE_TIMEOUT', 'did not reach the target floor')

    def phase_switch_map(self):
        m = self.mission
        self.renew_lease()
        self.publish_twist()
        if 'load_map' not in self.calls and not m.counters.get('_map_requested'):
            url = str(self.building.map_path(m.floor, self.p['simulation']))
            if self.call('load_map', self.load_map_client, LoadMap.Request(map_url=url)):
                m.counters['_map_requested'] = True
            elif self.elapsed() > 5.:
                self.fail('MAP_SWITCH_FAILED', 'map_server/load_map unavailable')
            return
        response = self.result('load_map')
        if response is None:
            return
        m.counters.pop('_map_requested', None)
        if response is False or response.result != LoadMap.Response.RESULT_SUCCESS:
            self.fail('MAP_SWITCH_FAILED', f'load_map failed for {m.floor}')
            return
        stop = self.stop_geometry(m.floor)
        self.set_phase('ALIGHTING')
        self.maneuver = LineManeuver(stop.hall_exit_pose, final_yaw=stop.hall_exit_pose[2], limits=self.limits())

    def phase_alighting(self):
        m = self.mission
        geometry = self.geometry(m.floor)
        if self.pose is not None:
            front = -self.depth(m.floor, geometry.car_door) + HALF_LENGTH  # past the car door, outward
            if self.door_state() in (ElevatorDoorState.CLOSING, ElevatorDoorState.CLOSED) and front < 0.:
                self.count('alighting_aborts')
                if self.count('alighting_attempts') > 2:
                    self.fail('ALIGHTING_FAILED', 'door closed before leaving repeatedly')
                    return
                if self.p['use_door_open_button']:
                    self.send_elevator(ElevatorCommand.CMD_DOOR_OPEN)
                self.set_phase('RETURN_TO_CAR', 'door closing before the threshold')
                self.maneuver = LineManeuver(geometry.car_pose, reverse=True, limits=self.limits())
                return
        done = self.drive()
        outside = self.floor_state is not None and self.floor_state.zone == FloorState.ZONE_OUTSIDE
        if done and outside:
            self.release_lease()
            self.set_phase('RELOCALIZE')
        elif self.elapsed() > self.p['boarding_timeout']:
            self.fail('ALIGHTING_TIMEOUT', 'could not leave the car')

    def phase_return_to_car(self):
        if self.drive():
            if self.door_state() == ElevatorDoorState.OPEN:
                stop = self.stop_geometry(self.mission.floor)
                self.set_phase('ALIGHTING', 'door open again')
                self.maneuver = LineManeuver(stop.hall_exit_pose, final_yaw=stop.hall_exit_pose[2],
                                             limits=self.limits())
            elif self.elapsed() > 15.:
                if self.p['use_door_open_button'] and self.count('return_open_press') <= 2:
                    self.send_elevator(ElevatorCommand.CMD_DOOR_OPEN)
                    self.mission.phase_since = self.now()
                else:
                    self.fail('DOOR_NOT_REOPENED', 'waiting in the car')

    def publish_initial_pose(self):
        if self.pose is None:
            return False
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x, message.pose.pose.position.y = self.pose[0], self.pose[1]
        message.pose.pose.orientation.z = math.sin(self.pose[2] / 2)
        message.pose.pose.orientation.w = math.cos(self.pose[2] / 2)
        covariance = [0.] * 36
        covariance[0] = covariance[7] = self.p['initial_pose_xy_variance']
        covariance[35] = self.p['initial_pose_yaw_variance']
        message.pose.covariance = covariance
        self.initial_pub.publish(message)
        return True

    def phase_relocalize(self):
        m = self.mission
        step = m.counters.get('_relocalize', 'resume')
        if step == 'resume':
            if 'suspend' not in self.calls:
                self.call('suspend', self.suspend_client, SetBool.Request(data=False))
                return
            response = self.result('suspend')
            if response is None:
                return
            if response is False or not response.success:
                self.fail('RESUME_FAILED', 'localization did not resume')
                return
            m.counters['_relocalize'] = 'seed'
        elif step == 'seed':
            if self.localization.get('state') == 'WAITING_INITIAL_POSE' and self.publish_initial_pose():
                m.counters['_relocalize'] = 'verify'
                m.counters['_seeded_at'] = self.now()
        elif step == 'verify':
            if self.ready():
                m.counters.pop('_relocalize', None)
                m.counters.pop('_seeded_at', None)
                self.set_phase(m.counters.pop('_after_relocalize', 'NAV_TO_GOAL'))
                return
            if self.now() - m.counters['_seeded_at'] > self.p['relocalize_timeout']:
                if self.count('tim_relocalize') > 1:
                    self.fail('RELOCALIZE_FAILED', 'localization did not verify after the ride')
                    return
                self.call('relocalize', self.relocalize_client, Trigger.Request())
                m.counters['_seeded_at'] = self.now() + 30.  # TIM search gets extra time
        if self.elapsed() > 4 * self.p['relocalize_timeout'] + 60.:
            self.fail('RELOCALIZE_TIMEOUT', 'localization did not become READY')

    def phase_nav_to_goal(self):
        if self.nav_leg(self.mission.pose, 'SUCCEEDED', 'NAV_TO_GOAL_FAILED'):
            self.finish('SUCCEEDED')


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = FloorMissionManager()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.publish_twist()
            except Exception:  # noqa: BLE001
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
