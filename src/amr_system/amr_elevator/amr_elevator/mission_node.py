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
from time import monotonic

import numpy as np

from action_msgs.msg import GoalStatus
from amr_elevator_interfaces.action import NavigateToFloorPose
from amr_elevator_interfaces.msg import (
    ElevatorBeacon, ElevatorCommand, ElevatorCommandStatus, ElevatorDoorState, FloorEvent,
    FloorState,
)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from rclpy.time import Time


def spin_executor():
    """Spin with rclpy's EventsExecutor when available.

    The default executor rebuilds its wait set in Python on every cycle; on
    the Jetson that was about half of each Python node's CPU (2026-10-23).
    """
    try:
        from rclpy.experimental import EventsExecutor
        return EventsExecutor()
    except ImportError:  # older rclpy
        from rclpy.executors import SingleThreadedExecutor
        return SingleThreadedExecutor()
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from amr_control.time_guard import HealthLease
from amr_elevator.announcements import ANNOUNCEMENTS
from amr_elevator.clearance import (
    boarding_region, cloud_points_xy, exit_region, scan_points, strip_clearance, sweep_hits,
)
from amr_elevator.geometry import depth_beyond, wrap
from amr_elevator.maneuver import DONE, TURN, LineManeuver, ManeuverLimits, travel_clearances

ACTIVE = ('NAV_TO_HALL', 'CALL_ELEVATOR', 'WAIT_DOOR_OPEN', 'BOARDING', 'BACKING_OUT',
          'IN_CAR_ALIGN', 'SELECT_FLOOR', 'RIDING', 'SWITCH_MAP', 'ALIGHTING', 'RETURN_TO_CAR',
          'RELOCALIZE', 'NAV_TO_GOAL')
# Phases where the robot may stand in the doorway; stopping there holds the door open.
DOORWAY = ('BOARDING', 'BACKING_OUT', 'ALIGHTING', 'RETURN_TO_CAR')
DEPTH_PHASES = ('WAIT_DOOR_OPEN', 'BOARDING', 'BACKING_OUT', 'IN_CAR_ALIGN',
                'ALIGHTING', 'RETURN_TO_CAR')
HALF_LENGTH = .32
STATUS_NAMES = {
    ElevatorCommandStatus.STATUS_DELIVERED: 'DELIVERED',
    ElevatorCommandStatus.STATUS_COMPLETED: 'COMPLETED',
    ElevatorCommandStatus.STATUS_DEVICE_REJECTED: 'DEVICE_REJECTED',
    ElevatorCommandStatus.STATUS_RF_TIMEOUT: 'RF_TIMEOUT',
    ElevatorCommandStatus.STATUS_NO_RESULT: 'NO_RESULT',
    ElevatorCommandStatus.STATUS_NO_ROUTE: 'NO_ROUTE',
    ElevatorCommandStatus.STATUS_BAD_FRAME: 'BAD_FRAME',
    ElevatorCommandStatus.STATUS_REPLACED: 'REPLACED',
}
RESULT_NAMES = {
    ElevatorCommandStatus.RESULT_WRONG_DEVICE: 'WRONG_DEVICE',
    ElevatorCommandStatus.RESULT_UNSUPPORTED: 'UNSUPPORTED',
    ElevatorCommandStatus.RESULT_BUTTON_FAULT: 'BUTTON_FAULT',
    ElevatorCommandStatus.RESULT_BAD_REQUEST: 'BAD_REQUEST',
}
# The press can never succeed as sent: retrying with a new sequence would not help.
FATAL_STATUSES = (ElevatorCommandStatus.STATUS_DEVICE_REJECTED,
                  ElevatorCommandStatus.STATUS_NO_ROUTE, ElevatorCommandStatus.STATUS_BAD_FRAME)


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
    repeat: int = 0
    acked_at: float = None
    rejected: str = ''


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
            'use_door_open_button': False, 'ack_timeout': 3., 'ack_attempts': 3,
            'door_wait_timeout': 120., 'ride_timeout': 180., 'relocalize_timeout': 30.,
            'boarding_timeout': 30., 'max_boarding_attempts': 3, 'lease_renew_period': 10.,
            # 0.3 m: the hall pose was 0.37 m off after a ride (2026-10-26); AMCL
            # must be able to reach the true pose when refinement is skipped.
            'initial_pose_xy_variance': .09, 'initial_pose_yaw_variance': .0076,
            'maneuver_speed': .35, 'tick_period': .05, 'floor_state_timeout': 3.,
            'localization_recovery_timeout': 45., 'require_estop': False, 'base_frame': 'base_footprint',
            # Door-open presses about 1 s apart, repeated by the car beacon itself.
            'door_open_repeat_board': 6, 'door_open_repeat_exit': 10, 'door_open_repeat_hold': 15,
            'door_open_rearm_s': 8., 'door_open_rearm_max': 2,
            'use_depth': True, 'depth_topic': '/camera/depth/points_filtered', 'depth_max_age': .5,
            'space_margin': .10, 'wall_margin': .12, 'turn_margin': .05, 'clear_confirm_s': .5,
            'announce_period': 5., 'board_block_timeout': 15., 'alight_block_timeout': 20.,
            'turn_block_timeout': 30., 'max_alighting_attempts': 3,
            'depth_min_height': .06, 'depth_max_height': 1., 'depth_max_range': 3.,
            'door_cancel_grace': 1.5,
            'scan_max_age': .5,
        }.items()}
        for key in ('depth_max_age', 'space_margin', 'wall_margin', 'turn_margin', 'clear_confirm_s',
                    'announce_period', 'board_block_timeout', 'alight_block_timeout', 'turn_block_timeout',
                    'max_alighting_attempts', 'depth_min_height', 'depth_max_height',
                    'depth_max_range', 'door_cancel_grace', 'scan_max_age'):
            if not math.isfinite(p[key]) or p[key] <= 0.:
                raise ValueError(f'{key} must be positive and finite')
        if type(p['max_alighting_attempts']) is not int:
            raise ValueError('max_alighting_attempts must be an integer')
        if p['depth_min_height'] >= p['depth_max_height']:
            raise ValueError('depth_min_height must be less than depth_max_height')
        for key in ('door_open_repeat_board', 'door_open_repeat_exit', 'door_open_repeat_hold'):
            if type(p[key]) is not int or not 0 <= p[key] <= ElevatorCommand.DOOR_OPEN_REPEAT_MAX:
                raise ValueError(f'{key} must be an integer in [0, {ElevatorCommand.DOOR_OPEN_REPEAT_MAX}]')
        self.p = p
        self.building = building or load_floor_config(p['floors_config'] or None)
        self.elevator_id = p['elevator_id']
        self._geometry_cache = {}
        self.group = ReentrantCallbackGroup()
        self.mission = None
        self.floor_state = None
        self.floor_state_time = -math.inf
        self.door = ElevatorDoorState()
        self.pose = None
        self.scan = None
        self.scan_received = -math.inf  # monotonic receive time, independent of ROS clock jumps
        self._clearance_scan = None
        self._clearances = (math.inf, math.inf)
        self.laser_pose = None     # laser (x, y, yaw) in base_frame, from TF once
        self._scan_points = np.empty((0, 2))
        self._depth_points = np.empty((0, 2))
        self._depth_clearances = (math.inf, math.inf)
        self.depth_received = -math.inf
        self.depth_transform = None
        self._announced = {}
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self, spin_thread=False)
        self.mode = 'MANUAL'
        self.estop = HealthLease()
        self.estop_event = False
        self.localization = {}
        self.sequence = 0
        self.pending = None        # HALL_CALL / CAR_FLOOR awaiting BUTTON_ACK
        self.door_pending = None   # DOOR_OPEN awaiting BUTTON_ACK (repeats started)
        self.maneuver = None
        self.lease_renewed = -math.inf
        self.nav = None            # {'future', 'handle', 'status'}
        self.calls = {}            # name -> Future
        self.last_status = None
        self.last_feedback_time = -math.inf
        self.seed_after_resume = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'floor_mission/status', latched)
        self.announcement_pub = self.create_publisher(String, 'robot/announcement', 10)
        self.command_pub = self.create_publisher(ElevatorCommand, 'elevator/command', 10)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel_maneuver', 1)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.create_subscription(FloorState, 'floor/state', self.on_floor_state, latched)
        self.create_subscription(FloorEvent, 'floor/events', self.on_floor_event, 20)
        self.create_subscription(ElevatorDoorState, 'elevator/door', self.on_door, 10)
        self.create_subscription(ElevatorBeacon, 'elevator/beacon', self.on_beacon, 50)
        self.create_subscription(ElevatorCommandStatus, 'elevator/command_status',
                                 self.on_command_status, 20)
        self.create_subscription(PoseStamped, 'floor/pose_estimate', self.on_pose, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        if p['use_depth']:
            self.create_subscription(PointCloud2, p['depth_topic'], self.on_depth, qos_profile_sensor_data)
        self.create_subscription(String, 'control_mux/mode', self.on_mode, latched)
        self.create_subscription(String, 'estop/status', self.on_estop, latched)
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
        key = (floor, self.elevator_id)
        if key not in self._geometry_cache:
            self._geometry_cache[key] = self.building.elevator_geometry(*key)
        return self._geometry_cache[key]

    def current_floor(self):
        s = self.floor_state
        return '' if s is None else s.floor

    def set_phase(self, phase, detail=''):
        m = self.mission
        now = self.now()
        if m.phase in ACTIVE:
            m.timings[m.phase] = m.timings.get(m.phase, 0.) + now - m.phase_since
        m.phase, m.phase_since, m.detail = phase, now, detail
        for key in ('_clear_since', '_blocked_since', '_align_active_since', '_exit_started',
                    '_exit_started_at', '_return_wait_since', '_return_arrived', '_prepared',
                    '_alight_announced',
                    '_door_not_open_since'):
            m.counters.pop(key, None)
        if phase == 'RIDING':
            for key in ('_last_transit', '_assumed_since', '_assumed_hold'):
                m.counters.pop(key, None)
        if phase == 'RETURN_TO_CAR':
            m.counters.pop('return_open_press', None)
        self.maneuver = None
        for key in {'BOARDING': ('board_start',), 'SWITCH_MAP': ('arrival',)}.get(phase, ()):
            self.announce(key)
        if phase == 'SELECT_FLOOR' and not m.counters.get('_boarding_announced'):
            self.announce('board_done')
            self.announce('alight_floor')
            m.counters['_boarding_announced'] = True
        self.get_logger().info(f'Mission {phase}: {detail}'.rstrip(': '))
        self.publish_status(force=True)

    def elapsed(self):
        return self.now() - self.mission.phase_since

    def count(self, key):
        counters = self.mission.counters
        counters[key] = counters.get(key, 0) + 1
        return counters[key]

    def announce(self, key):
        now = self.now()
        if now - self._announced.get(key, -math.inf) < self.p['announce_period']:
            return
        self._announced[key] = now
        floor = self.mission.floor if key == 'alight_floor' else self.current_floor()
        self.announcement_pub.publish(String(data=json.dumps({
            'key': key, 'text': ANNOUNCEMENTS[key].format(floor=floor),
            'phase': self.mission.phase, 'floor': floor, 'stamp': now,
        }, ensure_ascii=False)))

    def depth_fresh(self):
        return (self.p['use_depth'] and self.depth_transform is not None
                and 0. <= self.now() - self.depth_received <= self.p['depth_max_age'])

    def scan_fresh(self):
        return (self.scan is not None and self.laser_pose is not None
                and 0. <= monotonic() - self.scan_received <= self.p['scan_max_age'])

    def obstacle_points(self):
        if self.depth_fresh():
            return np.concatenate((self._scan_points, self._depth_points))
        return self._scan_points

    def space_clear(self, region):
        """Require a continuous clear interval with a known pose and laser frame."""
        clear = False
        if self.pose is not None and self.scan_fresh():
            x, y, yaw = self.pose
            c, s = math.cos(yaw), math.sin(yaw)
            points = self.obstacle_points() @ np.array(((c, s), (-s, c))) + (x, y)
            clear = not np.any(region.contains(points))
        if not clear:
            self.mission.counters.pop('_clear_since', None)
            return False
        since = self.mission.counters.setdefault('_clear_since', self.now())
        return self.now() - since >= self.p['clear_confirm_s']

    def blocked_for(self, blocked):
        if not blocked:
            self.mission.counters.pop('_blocked_since', None)
            return 0.
        since = self.mission.counters.setdefault('_blocked_since', self.now())
        return self.now() - since

    def call(self, name, client, request):
        if not client.service_is_ready():
            return False
        self.calls[name] = client.call_async(request)
        if name == 'suspend' and request.data and self.mission is not None:
            mission = self.mission
            mission.counters['_suspend_requested'] = True

            def suspended(future):
                try:
                    response = future.result()
                except Exception:  # a failed RPC never granted suspension
                    return
                if response and response.success:
                    mission.counters['_suspended'] = True
                    if (self.mission is mission
                            and mission.counters.pop('_resume_on_suspend_reply', False)):
                        # A delayed True reply may follow the failure's first False
                        # request. Send False again so the final state is resumed.
                        self.seed_after_resume = self.call(
                            'suspend', self.suspend_client, SetBool.Request(data=False))
            self.calls[name].add_done_callback(suspended)
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
                      earlier=(), repeat=0):
        self.sequence = (self.sequence + 1) & 0xff
        message = ElevatorCommand(command=command, floor_code=floor_code, direction=direction,
                                  sequence=self.sequence, repeat=repeat)
        message.header.stamp = self.get_clock().now().to_msg()
        self.command_pub.publish(message)
        location = (ElevatorBeacon.LOCATION_HALL if command == ElevatorCommand.CMD_HALL_CALL
                    else ElevatorBeacon.LOCATION_CAR)
        pending = PendingAck(self.sequence, location, (command, floor_code, direction), self.now(), attempts,
                             sequences={self.sequence, *earlier}, repeat=repeat)
        if command == ElevatorCommand.CMD_DOOR_OPEN:
            self.door_pending = pending  # Never replaces a pending hall or car-floor press.
        else:
            self.pending = pending

    def hold_door(self, repeat):
        """Ask the car beacon to press door-open ``repeat`` times, about 1 s apart.

        One command covers the whole hold; BUTTON_ACK only means the presses started,
        and the door state still comes from the LiDAR.
        """
        if self.p['use_door_open_button']:
            self.send_elevator(ElevatorCommand.CMD_DOOR_OPEN, repeat=repeat)

    def retry_door_open(self):
        """Resend an unacknowledged door-open with a new sequence; giving up is not fatal."""
        pending = self.door_pending
        if pending is None or pending.acked or self.now() - pending.sent < self.p['ack_timeout']:
            return
        if pending.attempts >= self.p['ack_attempts']:
            self.count('door_open_unacked')
            self.get_logger().warning(
                f'Door-open not acknowledged after {pending.attempts} presses; relying on the LiDAR')
            self.door_pending = None
            return
        self.send_elevator(ElevatorCommand.CMD_DOOR_OPEN, attempts=pending.attempts + 1,
                           earlier=pending.sequences, repeat=pending.repeat)

    def retry_pending(self, failure_code):
        """Resend an unacknowledged command; True while still trying."""
        pending = self.pending
        if pending is None or pending.acked:
            return True
        if pending.rejected:
            code = ('HALL_REJECTED' if pending.command[0] == ElevatorCommand.CMD_HALL_CALL
                    else 'CAR_REJECTED')
            self.fail(code, pending.rejected)
            return False
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
            self.lease_renewed = -math.inf  # retry next tick

    def release_lease(self):
        self.publish_twist()
        self.calls.pop('lease', None)  # an old renewal must not block the next preparation
        if self.lease_client.service_is_ready():
            self.lease_client.call_async(SetBool.Request(data=False))
        self.lease_renewed = -math.inf

    def door_state(self):
        return self.door.state

    def depth(self, floor, line):
        stop = self.stop_geometry(floor)
        return depth_beyond(self.pose, line, stop.car_inward_yaw)

    def clearance(self, reverse):
        # Received scans always carry a timestamp. Preserve the legacy lazy
        # projection path for callers that directly supply an unreceived scan.
        if hasattr(self, 'scan_received') and not FloorMissionManager.scan_fresh(self):
            return 0.
        if self.scan is None:
            if getattr(self, 'depth_transform', None) is not None and self.depth_fresh():
                return self._depth_clearances[bool(reverse)]
            return math.inf
        if self.laser_pose is None:
            # Without the laser frame the scan direction is unknown: hold rather than drive blind.
            self.get_logger().warning('laser transform unknown; maneuver held', throttle_duration_sec=5.)
            return 0.
        scan = self.scan
        if scan is not self._clearance_scan:
            # Cache both directions so a phase change never reprojects the same scan.
            self._clearances = travel_clearances(
                scan.ranges, scan.angle_min, scan.angle_increment,
                HALF_LENGTH, laser_pose=self.laser_pose)
            self._clearance_scan = scan
        depth = math.inf
        if getattr(self, 'depth_transform', None) is not None and self.depth_fresh():
            depth = self._depth_clearances[bool(reverse)]
        return min(self._clearances[bool(reverse)], depth)

    def drive(self):
        """Advance the current maneuver; True once done."""
        if self.maneuver is None or self.pose is None:
            self.publish_twist()
            return False
        if self.mode != 'AUTO' or not self.estop_ready():
            self.publish_twist()
            return False
        self.renew_lease()
        pending = self.maneuver.pending_turn(self.pose)
        # Round the safety envelope outward to centimetres (default .40 x .36),
        # conservatively including the specified 0.40 m near-front contact.
        turn_blocked = pending is not None and (not self.scan_fresh() or sweep_hits(
            self.obstacle_points(), pending,
            half_length=math.ceil((.328 + .02 + self.p['turn_margin']) * 100) / 100,
            half_width=math.ceil((.287 + .02 + self.p['turn_margin']) * 100) / 100,
            body_half_length=.328, body_half_width=.287))
        linear, angular = self.maneuver.command(
            self.pose, self.p['tick_period'], self.clearance(self.maneuver.reverse),
            turn_blocked=turn_blocked)
        if (self.mission.phase == 'ALIGHTING' and (linear or angular)
                and not self.mission.counters.get('_alight_announced')):
            self.announce('alight_start')
            self.mission.counters['_alight_announced'] = True
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
        self.scan_received = monotonic()
        if self.laser_pose is None:
            try:
                t = self.tf.lookup_transform(self.p['base_frame'], message.header.frame_id, Time()).transform
            except TransformException:
                return
            self.laser_pose = (t.translation.x, t.translation.y, yaw_of(t.rotation))
            self.destroy_subscription(self.tf_listener.tf_sub)
            del self.tf_listener.tf_sub  # TransformListener.unregister must not destroy it twice.
        self._scan_points = scan_points(message.ranges, message.angle_min, message.angle_increment,
                                        message.range_min, message.range_max, self.laser_pose)
        # Keep the established bumper convention and libm boundary handling.
        ranges = np.asarray(message.ranges, dtype=float)
        valid = np.isfinite(ranges) & (ranges >= message.range_min) & (ranges <= message.range_max)
        self._clearances = travel_clearances(np.where(valid, ranges, np.inf),
                                            message.angle_min, message.angle_increment,
                                            HALF_LENGTH, half_width=.307, laser_pose=self.laser_pose)
        self._clearance_scan = message

    def on_depth(self, message):
        if (not self.p['use_depth'] or self.mission is None
                or self.mission.phase not in DEPTH_PHASES):
            return
        if self.depth_transform is None:
            try:
                t = self.tf.lookup_transform(self.p['base_frame'], message.header.frame_id, Time()).transform
            except TransformException:
                return
            q = t.rotation
            x, y, z, w = q.x, q.y, q.z, q.w
            rotation = np.array(((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                                 (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                                 (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y))))
            self.depth_transform = ((t.translation.x, t.translation.y, t.translation.z), rotation)
        # Read float32 XYZ directly: preserve field offsets, endian and row padding
        # without adding a sensor_msgs_py runtime dependency to this package.
        fields = {f.name: f for f in message.fields}
        if any(name not in fields or fields[name].datatype != PointField.FLOAT32
               or fields[name].count != 1 for name in ('x', 'y', 'z')):
            return
        dtype = np.dtype({'names': ['x', 'y', 'z'],
                          'formats': [('>' if message.is_bigendian else '<') + 'f4'] * 3,
                          'offsets': [fields[name].offset for name in ('x', 'y', 'z')],
                          'itemsize': message.point_step})
        cloud = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                           strides=(message.row_step, message.point_step))
        points = np.column_stack([cloud[name].ravel() for name in ('x', 'y', 'z')])
        self._depth_points = cloud_points_xy(
            points, self.depth_transform, self.p['depth_min_height'],
            self.p['depth_max_height'], self.p['depth_max_range'])
        self._depth_clearances = tuple(strip_clearance(self._depth_points, reverse, HALF_LENGTH)
                                       for reverse in (False, True))
        self.depth_received = self.now()

    def on_mode(self, message):
        self.mode = 'AUTO' if message.data == 'AUTO' else 'MANUAL'

    def on_estop(self, message):
        previous = self.estop.identity
        self.estop.receive(message.data)
        if previous is not None and previous != self.estop.identity:
            self.estop_event = True

    def estop_ready(self):
        return not self.p['require_estop'] or self.estop.healthy()

    def estop_stopped(self):
        """An E-stop, or no fresh status. ARMING only holds; every E-stop changes the epoch."""
        if not self.p['require_estop']:
            return False
        fresh = self.estop.status and 0 <= monotonic()-self.estop.received < self.estop.timeout
        return (self.estop_event or not fresh
                or self.estop.status.get('state') not in ('READY', 'ARMING', 'BYPASS'))

    def on_localization(self, message):
        try:
            self.localization = json.loads(message.data)
        except ValueError:
            self.localization = {}

    def on_beacon(self, message):
        for pending in (self.pending, self.door_pending):
            if (pending is not None and not pending.acked
                    and message.event == ElevatorBeacon.EVENT_BUTTON_ACK
                    and message.location == pending.location
                    and message.ack_sequence in pending.sequences):
                pending.acked, pending.acked_at = True, self.now()

    def on_command_status(self, message):
        """STM32 RF relay progress (0x311): fail fast when a press can never succeed."""
        status = STATUS_NAMES.get(message.status, str(message.status))
        result = RESULT_NAMES.get(message.device_result, f'0x{message.device_result:02X}')
        for pending, door in ((self.pending, False), (self.door_pending, True)):
            if pending is None or pending.acked or message.cmd_seq not in pending.sequences:
                continue
            if message.status in FATAL_STATUSES:
                detail = f'{status} {result} (target 0x{message.target:02X}, seq {message.cmd_seq})'
                if door:
                    self.count('door_open_rejected')
                    self.get_logger().warning(f'Door-open rejected: {detail}')
                    self.door_pending = None
                else:
                    pending.rejected = detail
            elif message.status in (ElevatorCommandStatus.STATUS_RF_TIMEOUT,
                                    ElevatorCommandStatus.STATUS_NO_RESULT):
                if self.mission is not None and not self.mission.finished:
                    self.count('rf_timeouts')
                self.get_logger().info(f'Elevator press seq {message.cmd_seq}: {status}; '
                                       'the mission retries with a new sequence')

    def ready(self):
        return self.localization.get('state') == 'READY' and self.localization.get('ready') is True

    # Action ---------------------------------------------------------------------

    def on_goal(self, request):
        if self.mode != 'AUTO':
            self.get_logger().warning('Rejecting floor mission: manual takeover is active (MANUAL)')
            return GoalResponse.REJECT
        if not self.estop_ready():
            self.get_logger().warning('Rejecting floor mission: E-stop interlock is not READY')
            return GoalResponse.REJECT
        if self.mission is not None and not self.mission.finished:
            self.get_logger().warning('Rejecting floor mission: another mission is active')
            return GoalResponse.REJECT
        try:
            floor = self.building.floor(request.floor)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return GoalResponse.REJECT
        if not self.p['simulation'] and not floor.real_enabled:
            self.get_logger().warning(f'Rejecting {request.floor}: real_enabled is false')
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
        self._announced.clear()
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
        if outcome != 'SUCCEEDED' and m.phase in DOORWAY and self.in_doorway():
            # The robot stops where the door would close on it: keep the door open.
            self.hold_door(self.p['door_open_repeat_hold'])
        m.finished = True
        self.cancel_nav()
        self.maneuver = None
        self.pending = None
        self.door_pending = None
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

    def in_doorway(self):
        """True unless the pose shows the whole body clear of the hall-car threshold."""
        m = self.mission
        floor = m.floor if m.phase in ('ALIGHTING', 'RETURN_TO_CAR') else (
            self.current_floor() or m.start_floor)
        if self.pose is None:
            return True
        try:
            geometry = self.geometry(floor)
            past_hall = self.depth(floor, geometry.hall_door)   # inward of the hall door plane
            past_car = self.depth(floor, geometry.car_door)     # inward of the car door plane
        except ValueError:
            return True
        return past_hall + HALF_LENGTH > 0. and past_car - HALF_LENGTH < 0.

    def fail(self, code, detail=''):
        if self.mission is None or self.mission.finished:
            return
        self.get_logger().error(f'Mission failed safe: {code} {detail}')
        self.release_lease()
        inside = self.floor_state is not None and self.floor_state.zone == FloorState.ZONE_IN_ELEVATOR
        if not inside and (self.mission.counters.get('_suspended')
                           or self.mission.counters.get('_suspend_requested')):
            self.mission.counters['_resume_on_suspend_reply'] = True
            self.seed_after_resume = self.call('suspend', self.suspend_client, SetBool.Request(data=False))
        self.finish('FAILED', code, detail)
        try:
            self.announce('robot_error')
        except Exception as error:  # announcement transport must never prevent stopping
            self.get_logger().warning(f'Failure announcement unavailable: {error!r}')

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
        changed = key != self.last_status
        if force or changed:
            self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
            self.last_status = key
        now = self.now()
        if m is not None and not m.finished and (changed or now - self.last_feedback_time >= 1.):
            feedback = NavigateToFloorPose.Feedback(phase=m.phase, current_floor=status['current_floor'],
                                                    zone=status['zone'], detail=m.detail)
            m.handle.publish_feedback(feedback)
            self.last_feedback_time = now

    # State machine ------------------------------------------------------------------

    def tick(self):
        m = self.mission
        if self.seed_after_resume:
            response = self.result('suspend')
            if response is not None:
                self.seed_after_resume = False
                if response and response.success:
                    if m is not None:
                        m.counters.pop('_suspended', None)
                        m.counters.pop('_suspend_requested', None)
                    self.publish_initial_pose()
        if m is None or m.finished:
            self.estop_event = False
            return
        self.retry_door_open()
        if m.handle.is_cancel_requested:
            self.cancel()
            return
        if self.mode != 'AUTO':
            self.get_logger().warning('Manual takeover (LB): aborting floor mission')
            self.abandon('FAILED', 'MANUAL_OVERRIDE', 'manual takeover (LB)')
            return
        if self.estop_stopped():
            self.estop_event = False
            self.get_logger().warning('E-stop: aborting floor mission')
            self.abandon('FAILED', 'ESTOP', 'E-stop; send a new mission after release')
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
        self.abandon('CANCELED', 'CANCELED', 'canceled')

    def abandon(self, outcome, code, detail):
        m = self.mission
        inside = self.floor_state is not None and self.floor_state.zone == FloorState.ZONE_IN_ELEVATOR
        self.release_lease()
        if not inside and m.phase in ('BOARDING', 'BACKING_OUT', 'WAIT_DOOR_OPEN', 'ALIGHTING', 'RELOCALIZE'):
            # Outside the car again: give localization back, seeded at the carried pose.
            self.seed_after_resume = self.call('suspend', self.suspend_client, SetBool.Request(data=False))
        self.finish(outcome, code,
                    f'{detail} inside the car: localization stays suspended' if inside else detail)

    def phase_nav_to_hall(self):
        stop = self.stop_geometry(self.mission.start_floor if not self.current_floor() else self.current_floor())
        self.nav_leg(stop.hall_wait_pose, 'CALL_ELEVATOR', 'NAV_TO_HALL_FAILED')

    def phase_call_elevator(self):
        m = self.mission
        here = self.building.floor(self.current_floor() or m.start_floor)
        if (self.pending is not None and self.pending.acked
                and self.pending.command[0] == ElevatorCommand.CMD_HALL_CALL):
            self.announce('call')
            self.announce('waiting')
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
        opened = self.door_state() == ElevatorDoorState.OPEN
        prepared = m.counters.get('_prepared')
        if opened:
            m.counters.pop('_door_not_open_since', None)
        elif prepared:
            m.counters.setdefault('_door_not_open_since', self.now())
            m.counters.pop('_clear_since', None)
        # Drain both preparation replies even if the door closes during the RPCs.
        # RELOCALIZE must not mistake the old suspend(True) reply for resume(False).
        if prepared == 'pending':
            for name in ('suspend', 'lease'):
                response = self.result(name)
                if response is not None:
                    m.counters['_' + name + '_ok'] = bool(response and response.success)
                    if name == 'suspend' and response and response.success:
                        m.counters['_suspended'] = True
            if 'suspend' in self.calls or 'lease' in self.calls:
                self.publish_twist()
                if self.elapsed() > self.p['door_wait_timeout']:
                    self.announce('board_cancel')
                    self.fail('PREPARE_FAILED', 'preparation did not complete')
                return
            suspend = m.counters.pop('_suspend_ok', False)
            lease = m.counters.pop('_lease_ok', False)
            if not (suspend and lease):
                self.fail('PREPARE_FAILED', 'localization suspend or maneuver lease refused')
                return
            m.counters['_prepared'] = prepared = True
            self.lease_renewed = self.now()
        if prepared and not opened:
            self.publish_twist()
            self.renew_lease()
            since = m.counters['_door_not_open_since']
            if self.now() - since >= self.p['door_cancel_grace']:
                self.cancel_wait_boarding('door stayed non-OPEN before boarding')
            return
        if self.elapsed() > self.p['door_wait_timeout']:
            if self.count('recalls') > 1:
                self.announce('board_cancel')
                self.fail('DOOR_NOT_OPENED', 'elevator door or boarding space unavailable')
            elif prepared:
                self.cancel_wait_boarding('boarding space wait expired')
            else:
                self.pending = None
                self.set_phase('CALL_ELEVATOR', 'calling again')
            return
        if not opened:
            return
        if not prepared:
            if ('suspend' not in self.calls and 'lease' not in self.calls
                    and self.suspend_client.service_is_ready() and self.lease_client.service_is_ready()):
                self.call('suspend', self.suspend_client, SetBool.Request(data=True))
                self.call('lease', self.lease_client, SetBool.Request(data=True))
                m.counters['_prepared'] = 'pending'
            self.publish_twist()
            return
        self.renew_lease()
        geometry = self.geometry(self.current_floor() or m.start_floor)
        # The approach from the wait pose counts too: people leaving the car go first.
        region = boarding_region(geometry, self.p['space_margin'], self.p['wall_margin'],
                                 approach_from=self.pose)
        if not self.space_clear(region):
            self.publish_twist()
            self.announce('board_space')
            return
        self.hold_door(self.p['door_open_repeat_board'])
        self.set_phase('BOARDING')
        self.maneuver = LineManeuver(geometry.car_pose, final_yaw=geometry.car_pose[2],
                                     limits=self.limits())

    def cancel_wait_boarding(self, reason):
        self.announce('board_cancel')
        self.release_lease()
        if self.count('boarding_attempts') >= self.p['max_boarding_attempts']:
            self.fail('BOARDING_FAILED', reason)
            return
        self.pending = None
        self.mission.counters['_after_relocalize'] = 'CALL_ELEVATOR'
        self.set_phase('RELOCALIZE', reason)

    def back_out(self, reason, require_open=True):
        self.publish_twist()
        if require_open and self.door_state() != ElevatorDoorState.OPEN:
            if self.mission.phase == 'BOARDING':
                inside = self.floor_state is not None and self.floor_state.zone == FloorState.ZONE_IN_ELEVATOR
                if inside:
                    self.set_phase('IN_CAR_ALIGN', 'door not open for retreat; staying in the car')
                elif self.elapsed() > self.p['boarding_timeout']:
                    self.fail('BOARDING_FAILED', 'door not open for safe retreat')
            return False
        self.announce('board_cancel')
        self.hold_door(self.p['door_open_repeat_board'])
        self.count('boarding_aborts')
        self.pending = None  # a previous hall ACK cannot acknowledge the next call
        stop = self.stop_geometry(self.mission.start_floor)
        # After a partial 180-degree turn, ALIGN first faces into the car so that
        # reverse travel still leads to the hall. Its sweep is checked as usual.
        self.set_phase('BACKING_OUT', reason)
        self.maneuver = LineManeuver(stop.hall_wait_pose, reverse=True, limits=self.limits())
        return True

    def phase_boarding(self):
        m = self.mission
        floor = self.current_floor() or m.start_floor
        geometry = self.geometry(floor)
        front = None if self.pose is None else self.depth(floor, geometry.hall_door) + HALF_LENGTH
        closing = self.door_state() in (ElevatorDoorState.CLOSING, ElevatorDoorState.CLOSED)
        if closing and front is not None and front < 0.:
            self.back_out('door closing before the threshold', require_open=False)
            return
        self.drive()
        blocked = self.maneuver is not None and self.maneuver.blocked
        duration = self.blocked_for(blocked)
        if blocked:
            self.announce('obstacle')
            if duration >= self.p['board_block_timeout']:
                self.back_out('boarding continuously blocked')
                return
        inside = self.floor_state is not None and self.floor_state.zone == FloorState.ZONE_IN_ELEVATOR
        if self.maneuver is not None and self.maneuver.phase in (TURN, DONE) and inside:
            maneuver = self.maneuver
            self.set_phase('IN_CAR_ALIGN')
            self.maneuver = maneuver
            return
        if self.elapsed() > self.p['boarding_timeout']:
            self.back_out('boarding timeout')

    def phase_backing_out(self):
        if self.elapsed() > self.p['boarding_timeout']:
            self.fail('BACK_OUT_TIMEOUT', 'could not return to the hall')
            return
        floor = self.mission.start_floor
        geometry = self.geometry(floor)
        outside = (self.pose is not None
                   and self.depth(floor, geometry.hall_door) + HALF_LENGTH <= 0.)
        if self.door_state() != ElevatorDoorState.OPEN and not outside:
            self.publish_twist()
            self.renew_lease()
            if self.maneuver is not None:
                self.maneuver.speed = 0.
            if self.pose is not None and self.depth(floor, geometry.car_door) + HALF_LENGTH > 0.:
                self.set_phase('IN_CAR_ALIGN', 'door closed during retreat; staying in the car')
            return
        if not self.drive():
            return
        if self.count('boarding_attempts') >= self.p['max_boarding_attempts']:
            self.fail('BOARDING_FAILED', 'door closed before boarding repeatedly')
            return
        self.release_lease()
        self.mission.counters['_after_relocalize'] = 'CALL_ELEVATOR'
        self.set_phase('RELOCALIZE', 'back in the hall')

    def phase_in_car_align(self):
        if self.elapsed() > 2 * self.p['turn_block_timeout'] + 20.:
            self.publish_twist()
            if (self.door_state() == ElevatorDoorState.OPEN
                    and self.current_floor() == self.mission.start_floor):
                self.back_out('in-car alignment total time exceeded')
            else:
                self.fail('ALIGN_TIMEOUT', 'in-car alignment total time exceeded')
            return
        if self.maneuver is None:
            geometry = self.geometry(self.current_floor() or self.mission.start_floor)
            self.maneuver = LineManeuver(geometry.car_pose, final_yaw=geometry.car_pose[2], limits=self.limits())
        if self.drive():
            self.publish_twist()
            self.set_phase('SELECT_FLOOR')
            return
        blocked = self.maneuver.blocked
        duration = self.blocked_for(blocked)
        if blocked:
            self.mission.counters.pop('_align_active_since', None)
            self.announce('board_space')
            if duration >= self.p['turn_block_timeout']:
                if (self.door_state() == ElevatorDoorState.OPEN
                        and self.current_floor() == self.mission.start_floor):
                    self.back_out('in-car turn continuously blocked')
                elif duration >= 2 * self.p['turn_block_timeout']:
                    self.fail('ALIGN_BLOCKED', 'turn obstructed with no safe boarding cancellation')
            return
        since = self.mission.counters.setdefault('_align_active_since', self.now())
        if self.now() - since > 20.:
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
                # One command holds the door for the whole exit; never one per second.
                self.hold_door(self.p['door_open_repeat_exit'])
                m.counters['_door_rearms'] = 0
                self.set_phase('SWITCH_MAP')
                return
            if state.confidence == 'ASSUMED':
                waited = m.counters.setdefault('_assumed_since', self.now())
                if self.now() - waited > 6.:
                    self.fail('ARRIVAL_UNCONFIRMED', 'no hall beacon at the arrival floor')
                elif m.counters.get('_assumed_hold') is None:
                    m.counters['_assumed_hold'] = True
                    self.hold_door(self.p['door_open_repeat_exit'])
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
        if m.counters.get('_loaded_map') == m.floor:
            self.start_alighting()
            return
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
        m.counters['_loaded_map'] = m.floor
        self.start_alighting()

    def start_alighting(self):
        self.set_phase('ALIGHTING')
        # Create the maneuver only after the exit corridor is confirmed clear.

    def return_to_car(self, reason, already_in_car=False):
        self.publish_twist()
        self.announce('alight_cancel')
        self.count('alighting_aborts')
        if self.count('alighting_attempts') > self.p['max_alighting_attempts']:
            self.fail('ALIGHTING_FAILED', reason)
            return
        geometry = self.geometry(self.mission.floor)
        if not already_in_car:
            # Only a body in the doorway needs the door held; from the car centre the
            # door must be allowed to close so the target floor can be pressed again.
            self.hold_door(self.p['door_open_repeat_exit'])
        self.set_phase('RETURN_TO_CAR', reason)
        if already_in_car:
            self.mission.counters['_return_arrived'] = True
            self.mission.counters['_return_wait_since'] = self.now()
        else:
            self.maneuver = LineManeuver(geometry.car_pose, reverse=True, limits=self.limits())

    def phase_alighting(self):
        m = self.mission
        geometry = self.geometry(m.floor)
        if self.pose is not None:
            front = -self.depth(m.floor, geometry.car_door) + HALF_LENGTH
            if self.door_state() in (ElevatorDoorState.CLOSING, ElevatorDoorState.CLOSED) and front < 0.:
                self.return_to_car('door closing before the threshold',
                                   already_in_car=not m.counters.get('_exit_started'))
                return
            self.rearm_door_open(geometry)
        self.renew_lease()
        if not m.counters.get('_exit_started'):
            if self.elapsed() >= self.p['alight_block_timeout']:
                self.return_to_car('exit space total wait expired', already_in_car=True)
                return
            stop = self.stop_geometry(m.floor)
            region = exit_region(geometry, stop.hall_exit_pose, self.p['space_margin'], self.p['wall_margin'])
            clear = self.space_clear(region)
            # A clear-confirmation interval is still a hold, but not an obstacle.
            blocked = not clear and '_clear_since' not in m.counters
            self.blocked_for(blocked)
            if not clear:
                self.publish_twist()
                if blocked:
                    self.announce('obstacle')
                return
            m.counters['_exit_started'] = True
            m.counters['_exit_started_at'] = self.now()
            self.maneuver = LineManeuver(stop.hall_exit_pose, final_yaw=stop.hall_exit_pose[2],
                                         limits=self.limits())
        done = self.drive()
        blocked = self.maneuver is not None and self.maneuver.blocked
        duration = self.blocked_for(blocked)
        if blocked:
            self.announce('obstacle')
            if duration >= self.p['alight_block_timeout']:
                self.return_to_car('alighting continuously blocked')
                return
        outside = self.floor_state is not None and self.floor_state.zone == FloorState.ZONE_OUTSIDE
        if done and outside:
            self.announce('alight_done')
            self.release_lease()
            self.set_phase('RELOCALIZE')
        else:
            self.check_alighting_timeout()

    def check_alighting_timeout(self):
        if not self.mission.counters.get('_exit_started'):
            if self.elapsed() >= self.p['alight_block_timeout']:
                self.return_to_car('exit space total wait expired', already_in_car=True)
            return
        # The drive bound starts with the drive, not with the wait for a clear exit.
        if self.now() - self.mission.counters['_exit_started_at'] > self.p['boarding_timeout']:
            floor = self.mission.floor
            hall = self.geometry(floor).hall_door
            if self.pose is not None and self.depth(floor, hall) + HALF_LENGTH <= 0.:
                self.fail('ALIGHTING_TIMEOUT', 'could not leave the car')
            else:
                self.return_to_car('alighting timeout before clearing the hall door')

    def phase_return_to_car(self):
        m = self.mission
        arrived = m.counters.get('_return_arrived') or (
            self.maneuver is not None and self.maneuver.phase == DONE)
        if not arrived:
            if self.elapsed() > self.p['boarding_timeout']:
                self.fail('RETURN_BLOCKED', 'return-to-car drive timeout')
                return
            done = self.drive()
            blocked = self.maneuver is not None and self.maneuver.blocked
            duration = self.blocked_for(blocked)
            if blocked:
                self.announce('obstacle')
                if duration >= self.p['alight_block_timeout']:
                    self.fail('RETURN_BLOCKED', 'return to car continuously blocked')
                    return
            if not done:
                return
        m.counters['_return_arrived'] = True
        self.publish_twist()
        self.renew_lease()
        since = m.counters.setdefault('_return_wait_since', self.now())
        if self.door_state() == ElevatorDoorState.OPEN:
            self.start_alighting()
        elif self.now() - since > 15.:
            if self.p['use_door_open_button'] and self.count('return_open_press') <= 2:
                self.hold_door(self.p['door_open_repeat_exit'])
                m.counters['_return_wait_since'] = self.now()
            else:
                self.pending = None
                self.set_phase('SELECT_FLOOR', 'requesting target floor again after canceled exit')

    def rearm_door_open(self, geometry):
        """A slow exit outlasts the repeats: extend them once more, not every second."""
        m = self.mission
        pending = self.door_pending
        rear_in_doorway = self.depth(m.floor, geometry.hall_door) + HALF_LENGTH > 0.
        if (not rear_in_doorway or pending is None or not pending.acked
                or self.now() - pending.acked_at < self.p['door_open_rearm_s']
                or m.counters.get('_door_rearms', 0) >= self.p['door_open_rearm_max']):
            return
        m.counters['_door_rearms'] = m.counters.get('_door_rearms', 0) + 1
        self.count('door_open_rearms')
        self.hold_door(self.p['door_open_repeat_exit'])

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
            m.counters.pop('_suspended', None)
            m.counters.pop('_suspend_requested', None)
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
    executor = spin_executor()
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
