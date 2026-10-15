"""Verify localization before activating Nav2; never commands vehicle motion.

Global initialization is a hypothesis search, not a known pose. Repeated
corridors may remain ambiguous; in that case an operator must supply a prior.
All watchdogs use monotonic time, independently of paused simulation clocks.
"""
from concurrent.futures import ThreadPoolExecutor
from collections import deque
import copy
import hashlib
import json
import math
import time
import uuid

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import ParticleCloud
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from amr_localization_interfaces.srv import GlobalLocalize

from .localization_quality import MapQuality, covariance_is_bounded, particle_concentration


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def planar(pose):
    return (pose.position.x, pose.position.y, yaw(pose.orientation))


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def compose_se2(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    angle = a[2] + b[2]
    return (a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1],
            math.atan2(math.sin(angle), math.cos(angle)))


def inverse_se2(pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (-c*pose[0]-s*pose[1], s*pose[0]-c*pose[1], -pose[2])


def valid_planar_pose(position, rotation):
    values = (position.x, position.y, position.z, rotation.x, rotation.y, rotation.z, rotation.w)
    return (all(math.isfinite(v) for v in values)
            and abs(rotation.x) <= .01 and abs(rotation.y) <= .01
            and abs(sum(v*v for v in values[3:])-1) <= .01)


class NavigationStartup(Node):
    def __init__(self, **kwargs):
        super().__init__('navigation_startup', **kwargs)
        defaults = dict(
            automatic_localization=True, global_frame='map', odom_frame='odom',
            global_localization_backend='tim', tim_service='/localization/global_localize',
            tim_timeout_sec=30.0, tim_max_candidates=5,
            tim_max_compensation_translation=0.02, tim_max_compensation_yaw=0.01,
            base_frame='base_footprint', data_timeout=1.5, pose_timeout=3.0,
            search_timeout=60.0, verification_duration=2.5, verification_samples=6,
            manual_verification_duration=1.0, manual_verification_samples=4,
            initialization_update_interval=0.25, tracking_update_interval=1.0,
            tracking_verification_duration=1.0, tracking_verification_samples=4,
            max_xy_std=0.35, max_yaw_std=0.25, min_particle_concentration=0.90,
            min_match_ratio=0.70, min_known_ratio=0.85, min_valid_beams=30,
            initialization_max_beams=1080, tracking_max_beams=60,
            quality_max_beams=360, quality_max_range=30.0, tracking_quality_max_range=30.0,
            match_tolerance=0.20,
            ambiguity_margin=0.10, rival_position_step=0.5,
            rival_yaw_step=math.pi/12, rival_max_candidates=200000,
            service_timeout=5.0, navigation_start_timeout=90.0,
            stationary_linear_speed=0.02, stationary_angular_speed=0.04,
            stationary_duration=1.0,
        )
        self.cfg = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        for key, value in self.cfg.items():
            if isinstance(value, (float, int)) and not isinstance(value, bool):
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f'{key} must be positive and finite')
        for key in ('min_match_ratio', 'min_known_ratio', 'min_particle_concentration', 'ambiguity_margin'):
            if self.cfg[key] > 1:
                raise ValueError(f'{key} must be at most 1')
        for key in ('initialization_max_beams', 'tracking_max_beams', 'quality_max_beams'):
            if type(self.cfg[key]) is not int or self.cfg[key] < 2:
                raise ValueError(f'{key} must be an integer >= 2')
        for key in ('verification_samples', 'manual_verification_samples',
                    'tracking_verification_samples'):
            if type(self.cfg[key]) is not int or self.cfg[key] < 1:
                raise ValueError(f'{key} must be a positive integer')
        if self.cfg['global_localization_backend'] not in ('tim', 'amcl'):
            raise ValueError('global_localization_backend must be tim or amcl')
        if type(self.cfg['tim_max_candidates']) is not int or self.cfg['tim_max_candidates'] < 1:
            raise ValueError('tim_max_candidates must be a positive integer')
        self.session = str(uuid.uuid4())
        self.generation = 0  # Ordered within this UUID session, independent of wall-clock corrections.
        self.state, self.detail, self.source = 'WAITING_DATA', '지도·스캔·오도메트리·AMCL을 기다립니다.', 'unknown'
        self.last_logged_detail = ''
        self.last_state_log = -math.inf
        self.map_quality = None
        self.map_key = None
        self.map_snapshot = None
        self.map_revision = 0
        self.scan = self.odom = self.pose = self.cloud = None
        self.scan_samples = deque(maxlen=30)
        self.pose_samples = deque(maxlen=30)
        self.received = {}
        # Keep actual callback history separate from the transformable scan
        # selected by data_problem(). Recovery cannot erase an inference gap.
        self.sensor_history = {}
        self.odom_samples = deque(maxlen=3000)
        self.nav_status = 'IDLE'
        self.last_motion = time.monotonic()
        self.last_motion_stamp = self.get_clock().now().nanoseconds
        self.motion_revision = 0
        self.goal_revision = 0
        self.initialized = False
        # A tracking interruption does not erase the verified global prior.
        # Only an explicit new initialization generation can do that.
        self.prior_verified = False
        self.tracking_bad_since = None
        self.tracking_bad_count = 0
        self.tracking_bad_last_scan = 0
        self.tracking_warning = ''
        self.evidence_issue = None
        self.auto_attempted = False
        self.attempt_started = 0.0
        self.attempt_stamp = 0
        self.good_since = None
        self.good_count = 0
        self.last_checked_scan = 0
        self.last_checked_pose = 0
        self.last_candidate = None
        self.metrics = {}
        self.tim_metrics = {'tim_status': 'idle'}
        self.tim_operation = None
        self.auto_seed_echoes = deque(maxlen=8)
        self.beam_phase = 'initialization'
        self.beam_revision = 0
        self.beam_confirmed_revision = -1
        self.applied_beams = None
        self.beam_operation = None
        self.beam_failed = False
        self.evidence_epoch = 0
        self.global_pending = False
        self.global_future = self.start_future = self.nomotion_future = None
        self.reset_uncertain = False
        self.global_requested = self.start_requested = self.nomotion_requested = 0.0
        self.start_sent = False
        # Elevator car: verification paused; resume always waits for a new /initialpose.
        self.suspended = False
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='localization_rival')
        self.rival_future = None
        self.rival_result = None
        self.rival_generation = None
        self.rival_evidence_epoch = None
        self.rival_pose = None
        self.rival_map_key = None
        self.rival_submitted_score = 0.0
        self.rival_submitted_at = 0.0
        self.rival_completed_at = 0.0
        self.rival_motion_revision = 0
        self.lifecycle_names = ('amcl', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator')
        self.lifecycle_clients = {n: self.create_client(GetState, f'/{n}/get_state') for n in self.lifecycle_names}
        self.lifecycle_pending = {}
        self.lifecycle_states = {}
        self.lifecycle_poll = 0.0
        self.global_client = self.create_client(Empty, '/reinitialize_global_localization')
        self.tim_client = self.create_client(GlobalLocalize, self.cfg['tim_service'])
        self.nomotion_client = self.create_client(Empty, '/request_nomotion_update')
        self.beam_set_client = self.create_client(SetParametersAtomically, '/amcl/set_parameters_atomically')
        self.beam_get_client = self.create_client(GetParameters, '/amcl/get_parameters')
        self.start_client = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/localization/status', latched)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: self.receive('pose', m), latched)
        self.create_subscription(ParticleCloud, '/particle_cloud', lambda m: self.receive('cloud', m), qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose_message, 10)
        self.create_subscription(String, '/navigation/status', self.on_navigation_status, latched)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_service(Trigger, '/localization/relocalize', self.on_relocalize)
        self.create_service(SetBool, '/localization/suspend', self.on_suspend)
        # Wall timer continues to revoke readiness when /clock is paused.
        from rclpy.clock import Clock, ClockType
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self.tick, clock=self.wall_clock)
        self.publish_status()

    def receive(self, name, message):
        setattr(self, name, message)
        self.received[name] = time.monotonic()
        if name == 'pose':
            self.pose_samples.append((message, self.received[name]))

    def on_odom(self, message):
        self.observe_sensor('odom', message)
        if not self.valid_odom(message):
            self.latch_tim_failure('invalid_odom', 'TIM 계산 중 오도메트리 좌표계 또는 값이 유효하지 않았습니다.')
            self.record_motion()
        else:
            if self.tim_operation is not None:
                self.observe_tim_excursion(planar(message.pose.pose))
            elif self.odom is not None and self.valid_odom(self.odom):
                # Outside inference retain the quiet-period pose check. During
                # inference every sample instead uses the bounded t0 excursion.
                delta = compose_se2(inverse_se2(planar(self.odom.pose.pose)), planar(message.pose.pose))
                if math.hypot(*delta[:2]) > .001 or abs(delta[2]) > .001:
                    self.record_motion()
            twist = message.twist.twist
            speed, angular = math.hypot(twist.linear.x, twist.linear.y), abs(twist.angular.z)
            if self.tim_operation is not None:
                self.tim_metrics['tim_max_linear_speed'] = max(self.tim_metrics['tim_max_linear_speed'], speed)
                self.tim_metrics['tim_max_angular_speed'] = max(self.tim_metrics['tim_max_angular_speed'], angular)
            if speed > self.cfg['stationary_linear_speed'] or angular > self.cfg['stationary_angular_speed']:
                self.latch_tim_failure('discarded_motion', 'TIM 계산 중 측정된 속도가 정지 기준을 초과했습니다.')
                self.record_motion()
        self.receive('odom', message)
        self.odom_samples.append(message)

    def on_scan(self, message):
        self.observe_sensor('scan', message)
        if not self.valid_scan(message):
            self.latch_tim_failure('invalid_scan', 'TIM 계산 중 해석 가능한 유효 스캔이 부족했습니다.')
        self.receive('scan', message)
        self.scan_samples.append((message, self.received['scan']))

    def record_motion(self):
        self.last_motion = time.monotonic()
        self.last_motion_stamp = self.get_clock().now().nanoseconds
        self.motion_revision += 1

    def on_command(self, twist):
        values = (twist.linear.x, twist.linear.y, twist.angular.z)
        # A command is movement intent, even below the measured quiet-speed
        # threshold. Zero commands must not repeatedly restart the quiet timer.
        if not all(math.isfinite(v) for v in values) or any(abs(v) > 1e-6 for v in values):
            self.latch_tim_failure('discarded_command', 'TIM 계산 중 이동 명령이 발생했습니다.')
            self.record_motion()

    def on_navigation_status(self, message):
        self.nav_status = message.data
        if self.goal_in_progress():
            self.goal_revision += 1
            self.latch_tim_failure('discarded_goal', 'TIM 계산 중 주행 목표가 시작되었습니다.')

    def valid_odom(self, message):
        pose, twist = message.pose.pose, message.twist.twist
        return (message.header.frame_id == self.cfg['odom_frame']
                and message.child_frame_id == self.cfg['base_frame']
                and valid_planar_pose(pose.position, pose.orientation)
                and all(math.isfinite(v) for v in (twist.linear.x, twist.linear.y, twist.linear.z,
                                                  twist.angular.x, twist.angular.y, twist.angular.z))
                and math.isfinite(math.hypot(twist.linear.x, twist.linear.y)))

    def valid_scan(self, scan):
        metadata = (scan.angle_min, scan.angle_max, scan.angle_increment, scan.range_min,
                    scan.range_max, scan.time_increment)
        return (all(math.isfinite(v) for v in metadata) and scan.angle_increment != 0
                and 0 <= scan.time_increment * max(0, len(scan.ranges)-1) <= self.cfg['data_timeout']
                and 0 <= scan.range_min < scan.range_max
                and sum(math.isfinite(r) and scan.range_min <= r <= scan.range_max
                        for r in scan.ranges) >= self.cfg['min_valid_beams'])

    def latch_tim_failure(self, status, detail):
        if self.tim_operation is not None and self.tim_operation['failure'] is None:
            self.tim_operation['failure'] = (status, detail)

    def observe_sensor(self, name, message):
        now = time.monotonic()
        previous = self.sensor_history.get(name)
        self.sensor_history[name] = (message, now)
        if self.tim_operation is None:
            return
        if previous is not None:
            gap = now-previous[1]
            key = f'tim_max_{name}_receive_gap_sec'
            self.tim_metrics[key] = max(self.tim_metrics.get(key, 0.), gap)
            if gap > self.cfg['data_timeout']:
                self.latch_tim_failure(f'discarded_{name}_gap', f'TIM 계산 중 {name} 수신이 중단되었습니다.')
            if stamp_ns(message.header.stamp) < stamp_ns(previous[0].header.stamp):
                self.latch_tim_failure(f'invalid_{name}_stamp', f'TIM 계산 중 {name} 시각이 역행했습니다.')
        if name == 'scan' and message.header.frame_id != self.tim_operation['scan'].header.frame_id:
            self.latch_tim_failure('discarded_sensor', 'TIM 계산 중 LiDAR 좌표계가 변경되었습니다.')
        self.check_tim_sensor_age(name, message, now)

    def check_tim_sensor_age(self, name, message, received):
        wall_age = time.monotonic()-received
        age = (self.get_clock().now().nanoseconds-stamp_ns(message.header.stamp))/1e9
        self.tim_metrics[f'tim_{name}_receive_age_sec'] = wall_age
        self.tim_metrics[f'tim_{name}_stamp_age_sec'] = age
        key = f'tim_max_{name}_receive_gap_sec'
        self.tim_metrics[key] = max(self.tim_metrics.get(key, 0.), wall_age)
        key = f'tim_max_{name}_stamp_age_sec'
        self.tim_metrics[key] = max(self.tim_metrics.get(key, 0.), age)
        if wall_age > self.cfg['data_timeout']:
            self.latch_tim_failure(f'discarded_{name}_gap', f'TIM 계산 중 {name} 수신이 중단되었습니다.')
        elif stamp_ns(message.header.stamp) == 0 or not -.1 <= age <= self.cfg['data_timeout']:
            self.latch_tim_failure(f'invalid_{name}_stamp', f'TIM 계산 중 {name} 시간 정보가 최신이 아니었습니다.')

    def observe_tim_excursion(self, pose):
        delta = compose_se2(inverse_se2(self.tim_operation['odom_pose']), pose)
        distance, angle = math.hypot(*delta[:2]), abs(delta[2])
        if not math.isfinite(distance) or not math.isfinite(angle):
            self.latch_tim_failure('invalid_odom', 'TIM 계산 중 오도메트리 편차를 계산할 수 없었습니다.')
            self.record_motion()
            return
        self.tim_metrics['tim_max_translation'] = max(self.tim_metrics['tim_max_translation'], distance)
        self.tim_metrics['tim_max_yaw'] = max(self.tim_metrics['tim_max_yaw'], angle)
        if (distance > self.cfg['tim_max_compensation_translation']
                or angle > self.cfg['tim_max_compensation_yaw']):
            self.latch_tim_failure('discarded_excursion', 'TIM 계산 중 오도메트리 최대 편차가 보상 한도를 초과했습니다.')
            self.record_motion()

    def checked_transform(self, target, source, when):
        transform = self.tf_buffer.lookup_transform(target, source, when)
        t = transform.transform
        if (transform.header.frame_id != target or transform.child_frame_id != source
                or not valid_planar_pose(t.translation, t.rotation)):
            raise ValueError('Invalid planar transform or frame')
        return (t.translation.x, t.translation.y, yaw(t.rotation))

    def on_map(self, message):
        if message.header.frame_id != self.cfg['global_frame']:
            self.map_quality = None
            self.map_snapshot = None
            self.map_revision += 1
            self.set_state('WAITING_DATA', '지도 좌표계가 설정과 다릅니다.')
            return
        info = message.info
        raw = np.asarray(message.data, dtype=np.int16)
        key = (info.width, info.height, info.resolution, planar(info.origin), hashlib.sha256(raw.tobytes()).digest())
        if key == self.map_key and self.map_quality is not None:
            return
        try:
            quality = MapQuality(info.width, info.height, info.resolution, *planar(info.origin),
                                 raw, match_tolerance=self.cfg['match_tolerance'])
        except (ValueError, TypeError) as exc:
            self.map_quality = None
            self.map_snapshot = None
            self.map_revision += 1
            self.set_state('WAITING_DATA', f'지도를 읽을 수 없습니다: {exc}')
            return
        changed = self.map_key is not None
        self.map_key, self.map_quality = key, quality
        self.map_snapshot = copy.deepcopy(message)
        self.map_revision += 1
        if changed:
            self.cancel_tim('discarded_map')
            self.new_generation('unknown')
            self.initialized = False
            # A running system must never silently reset onto a changed map.
            self.auto_attempted = True
            if self.suspended:
                return  # The elevator mission switches maps while suspended.
            self.set_state('WAITING_INITIAL_POSE', '지도가 변경되었습니다. 정지 후 초기 위치를 다시 지정하거나 자동 탐색을 재시도하세요.')

    def new_generation(self, source):
        self.cancel_tim('discarded_manual' if source == 'manual' else 'superseded')
        self.generation += 1
        self.source = source
        self.prior_verified = False
        self.clear_tracking_warning()
        self.beam_phase = 'initialization'
        self.beam_revision += 1
        self.global_pending = False
        # Keep an older in-flight parameter operation until it completes. A
        # local Future cancellation would not retract the remote AMCL write.
        self.clear_evidence()

    def clear_evidence(self):
        """Require new observations after a seed, beam change or tracking fault."""
        self.attempt_started = time.monotonic()
        self.attempt_stamp = self.get_clock().now().nanoseconds
        self.evidence_epoch += 1
        self.pose = self.cloud = None
        self.pose_samples.clear()
        self.good_since = None
        self.good_count = 0
        self.last_checked_scan = 0
        self.last_checked_pose = 0
        self.last_candidate = None
        self.rival_result = None
        self.metrics = {}

    def on_initialpose_message(self, message, info):
        # The initializer publishes a normal /initialpose for AMCL, but its own
        # DDS echo must not turn an automatic hypothesis into a human prior.
        publisher_gid = bytes(info.get('publisher_gid', b''))
        if publisher_gid and any(
                endpoint.node_name == self.get_name()
                and endpoint.node_namespace == self.get_namespace()
                and bytes(endpoint.endpoint_gid) == publisher_gid
                for endpoint in self.get_publishers_info_by_topic('/initialpose')):
            return
        # Jazzy's rclpy MessageInfo omits publisher_gid. Match and consume the
        # exact, freshly timestamped message we just published in that case.
        # Keep superseded echoes too: a delayed local echo is never a manual
        # command and must not erase a newer operator request's generation.
        fingerprint = hashlib.sha256(serialize_message(message)).digest()
        if not publisher_gid and fingerprint in self.auto_seed_echoes:
            self.auto_seed_echoes.remove(fingerprint)
            return
        self.on_initialpose(message)

    def on_initialpose(self, message):
        if self.suspended:
            self.get_logger().warning('엘리베이터 구간에서는 초기 위치를 받지 않습니다. 재개 후 다시 지정하세요.')
            return
        if self.beam_failed:
            self.fail_beams('이전 빔 설정 변경이 완료되지 않았습니다.')
            return
        if self.reset_uncertain:
            self.set_state('FAILED', 'AMCL 초기화 완료 여부가 불명확합니다. localization과 navigation을 재시작하세요.')
            return
        self.new_generation('manual')
        self.auto_attempted = True
        q = message.pose.pose.orientation
        values = (*planar(message.pose.pose), q.x, q.y, q.z, q.w, *message.pose.covariance)
        valid = (message.header.frame_id == self.cfg['global_frame']
                 and all(math.isfinite(v) for v in values)
                 and abs(sum(v*v for v in (q.x, q.y, q.z, q.w))-1) < .01)
        if self.global_future is not None:
            # AMCL services and subscriptions need not execute in DDS delivery
            # order. An in-flight global reset could overwrite this manual pose.
            self.initialized = False
            self.set_state('WAITING_INITIAL_POSE', '자동 초기화 응답을 기다린 뒤 초기 위치를 다시 지정하세요.')
            return
        self.initialized = bool(valid)
        if valid:
            x, y, heading = planar(message.pose.pose)
            self.get_logger().info(
                f'수동 초기 위치 수신: ({x:.3f}, {y:.3f}, {heading:.3f}rad). '
                '새 위치를 기준으로 검증을 시작합니다.')
        self.set_state('VERIFYING' if valid else 'WAITING_INITIAL_POSE',
                       '지정된 초기 위치에서 스캔·지도 정렬을 확인합니다.' if valid else '유효한 지도 좌표계 초기 위치를 지정하세요.')

    def stationary(self):
        return (self.odom is not None and self.fresh('odom', self.cfg['data_timeout'])
                and time.monotonic()-self.last_motion >= self.cfg['stationary_duration'])

    def goal_in_progress(self):
        return self.nav_status not in ('IDLE', 'SUCCEEDED', 'CANCELED', 'FAILED')

    def on_suspend(self, request, response):
        """Pause verification inside the elevator car; resuming waits for /initialpose.

        Inside the car the scan cannot match the floor map, so READY is withdrawn
        instead of being judged. Nav2 lifecycle is not touched.
        """
        if self.state == 'FAILED' or self.beam_failed or self.reset_uncertain:
            response.success, response.message = False, f'위치 감독기 상태가 {self.state}입니다.'
            return response
        if request.data == self.suspended:
            response.success, response.message = True, 'already ' + ('suspended' if self.suspended else 'active')
            return response
        self.cancel_tim('superseded')
        self.new_generation('unknown')
        self.initialized = False
        self.auto_attempted = True
        self.suspended = bool(request.data)
        if self.suspended:
            self.set_state('SUSPENDED', '엘리베이터 구간: 위치 검증을 일시 정지했습니다.')
        else:
            self.set_state('WAITING_INITIAL_POSE', '엘리베이터 하차 후 초기 위치를 기다립니다.')
        response.success, response.message = True, self.detail
        return response

    def on_relocalize(self, request, response):
        if self.suspended:
            response.success, response.message = False, '엘리베이터 구간에서는 위치 탐색을 시작하지 않습니다.'
            return response
        if self.beam_failed:
            response.success, response.message = False, 'AMCL 빔 설정 실패 상태입니다. localization과 navigation을 재시작하세요.'
        elif self.reset_uncertain:
            response.success, response.message = False, 'AMCL 초기화 완료 여부가 불명확합니다. localization과 navigation을 재시작하세요.'
        elif self.goal_in_progress() or not self.stationary():
            response.success, response.message = False, '목표를 취소하고 로봇을 정지한 뒤 다시 시도하세요.'
        elif (self.state == 'STARTING_NAVIGATION' or self.global_future is not None
              or self.tim_operation is not None or self.global_pending or self.beam_operation is not None):
            response.success, response.message = False, '진행 중인 초기화가 완료될 때까지 기다리세요.'
        else:
            problem = self.data_problem()
            if problem or not self.initializer_ready():
                response.success, response.message = False, problem or '선택한 전역 위치 탐색 서비스가 준비되지 않았습니다.'
            else:
                self.begin_global()
                response.success, response.message = True, '자동 위치 탐색을 시작했습니다. 위치 검증 완료 후 주행할 수 있습니다.'
        return response

    def begin_global(self):
        self.new_generation('auto')
        self.initialized = True
        self.auto_attempted = True
        self.global_pending = True
        self.set_state('SEARCHING', '자동 탐색용 LiDAR 빔 설정을 준비합니다. 로봇을 정지해 두세요.')

    def initializer_ready(self):
        client = self.tim_client if self.cfg['global_localization_backend'] == 'tim' else self.global_client
        return client.service_is_ready()

    def cancel_tim(self, status):
        if self.tim_operation is not None:
            # This service only computes candidates. Canceling our local
            # interest is safe; it never writes a remote AMCL pose.
            if self.tim_operation['future'] is not None:
                self.tim_operation['future'].cancel()
            self.tim_operation = None
            self.tim_metrics['tim_status'] = status
            self.tim_metrics['tim_rejection_reason'] = status

    def reject_tim(self, status, detail):
        self.cancel_tim(status)
        self.tim_metrics['tim_status'] = status
        self.tim_metrics['tim_rejection_reason'] = status
        self.tim_metrics['tim_motion_events'] = self.motion_revision - self.tim_metrics.get('tim_motion_revision', self.motion_revision)
        self.tim_metrics['tim_goal_events'] = self.goal_revision - self.tim_metrics.get('tim_goal_revision', self.goal_revision)
        self.initialized = False
        self.set_state('WAITING_INITIAL_POSE', detail + ' 초기 위치를 지정하거나 자동 탐색을 재시도하세요.')

    def request_tim(self):
        scan = copy.deepcopy(self.scan)
        when = Time.from_msg(scan.header.stamp)
        self.tim_metrics = dict(tim_status='running', tim_candidates=0, tim_rejection_reason=None,
                                tim_max_translation=0., tim_max_yaw=0.,
                                tim_max_linear_speed=0., tim_max_angular_speed=0.,
                                tim_motion_revision=self.motion_revision, tim_goal_revision=self.goal_revision,
                                tim_motion_events=0, tim_goal_events=0)
        if not self.valid_odom(self.odom) or not self.valid_scan(scan):
            self.reject_tim('invalid_input', '오도메트리 또는 유효 LiDAR 스캔이 부족합니다.')
            return
        if stamp_ns(scan.header.stamp) < self.last_motion_stamp:
            self.reject_tim('invalid_input', '이동이 끝난 뒤 관측된 LiDAR 스캔이 필요합니다.')
            return
        try:
            # Save t0 transforms now; a 30s inference can outlive the TF cache.
            extrinsic = self.checked_transform(self.cfg['base_frame'], scan.header.frame_id, when)
            odom_pose = self.checked_transform(self.cfg['odom_frame'], self.cfg['base_frame'], when)
        except (TransformException, ValueError):
            self.reject_tim('invalid_input_tf', 'TIM 입력 시각의 좌표 변환이 유효하지 않습니다.')
            return
        request = GlobalLocalize.Request(
            map=copy.deepcopy(self.map_snapshot), scan=scan,
            max_candidates=self.cfg['tim_max_candidates'], timeout_sec=self.cfg['tim_timeout_sec'])
        self.tim_operation = dict(
            future=None, started=time.monotonic(), failure=None,
            generation=self.generation, map_key=self.map_key, map_revision=self.map_revision,
            motion_revision=self.motion_revision, goal_revision=self.goal_revision,
            scan=scan, extrinsic=extrinsic, odom_pose=odom_pose)
        # Cover the short scan-to-submission interval as well as later samples.
        for sample in self.odom_samples:
            if stamp_ns(sample.header.stamp) >= stamp_ns(scan.header.stamp):
                if not self.valid_odom(sample):
                    self.latch_tim_failure('invalid_odom', 'TIM 입력 이후 유효하지 않은 오도메트리가 있습니다.')
                else:
                    self.observe_tim_excursion(planar(sample.pose.pose))
        for name, (message, received) in self.sensor_history.items():
            self.check_tim_sensor_age(name, message, received)
        if self.tim_operation['failure']:
            self.reject_tim(*self.tim_operation['failure'])
            return
        self.tim_operation['future'] = self.tim_client.call_async(request)
        self.set_state('SEARCHING', 'TIM으로 지도 전체의 스캔 일치 후보를 계산합니다. 로봇을 정지해 두세요.')

    def advance_tim(self):
        operation = self.tim_operation
        if operation is None:
            return False
        elapsed = time.monotonic()-operation['started']
        self.tim_metrics['tim_wall_elapsed_sec'] = elapsed
        for name, (message, received) in self.sensor_history.items():
            self.check_tim_sensor_age(name, message, received)
        self.tim_metrics['tim_motion_events'] = self.motion_revision-operation['motion_revision']
        self.tim_metrics['tim_goal_events'] = self.goal_revision-operation['goal_revision']
        if (operation['generation'] != self.generation or operation['map_key'] != self.map_key
                or operation['map_revision'] != self.map_revision or self.map_snapshot is None):
            self.reject_tim('discarded_map', 'TIM 계산 중 지도 또는 초기화 요청이 변경되었습니다.')
        elif operation['failure']:
            self.reject_tim(*operation['failure'])
        elif operation['goal_revision'] != self.goal_revision or self.goal_in_progress():
            self.reject_tim('discarded_goal', 'TIM 계산 중 주행 목표가 시작되었습니다.')
        elif operation['motion_revision'] != self.motion_revision or not self.stationary():
            self.reject_tim('discarded_motion', 'TIM 계산 중 로봇의 정지 상태가 해제되었습니다.')
        elif elapsed > self.cfg['tim_timeout_sec']:
            self.reject_tim('timeout', 'TIM 계산 응답 시간이 초과되었습니다.')
        elif operation['future'].done():
            try:
                response = operation['future'].result()
                if not response.success:
                    self.reject_tim('no_result', 'TIM이 신뢰할 수 있는 위치 후보를 찾지 못했습니다.')
                    return True
                problem = self.data_problem()
                if problem:
                    self.reject_tim('discarded_stale', problem)
                    return True
                if self.scan.header.frame_id != operation['scan'].header.frame_id:
                    self.reject_tim('discarded_sensor', 'TIM 계산 중 LiDAR 좌표계가 변경되었습니다.')
                    return True
                when = Time.from_msg(self.scan.header.stamp)
                if when.nanoseconds < stamp_ns(operation['scan'].header.stamp):
                    self.reject_tim('invalid_scan_stamp', 'TIM 결과 검증용 스캔 시각이 입력보다 이전입니다.')
                    return True
                try:
                    current_pose = self.checked_transform(self.cfg['odom_frame'], self.cfg['base_frame'], when)
                    current_extrinsic = self.checked_transform(self.cfg['base_frame'], self.scan.header.frame_id, when)
                except (TransformException, ValueError):
                    self.reject_tim('invalid_result_tf', 'TIM 결과 시각의 좌표 변환이 유효하지 않습니다.')
                    return True
                self.observe_tim_excursion(current_pose)
                if operation['failure']:
                    self.reject_tim(*operation['failure'])
                    return True
                # T_map_base(t1) = T_map_laser(t0) T_base_laser(t0)^-1
                #                  T_odom_base(t0)^-1 T_odom_base(t1).
                # Quality must use the transported laser pose and that same t1
                # observation. AMCL receives the base pose stamped at t1.
                delta = compose_se2(inverse_se2(operation['odom_pose']), current_pose)
                self.tim_metrics.update(tim_compensation_translation=math.hypot(*delta[:2]),
                                        tim_compensation_yaw=abs(delta[2]),
                                        tim_input_stamp_ns=stamp_ns(operation['scan'].header.stamp),
                                        tim_seed_stamp_ns=when.nanoseconds)
                if (not response.poses or len(response.poses) != len(response.scores)
                        or len(response.poses) > self.cfg['tim_max_candidates']
                        or not math.isfinite(response.elapsed_sec) or response.elapsed_sec < 0):
                    raise ValueError('Invalid TIM result shape or elapsed time')
                candidates = []
                for pose, score in zip(response.poses, response.scores):
                    p, q = pose.pose.position, pose.pose.orientation
                    if (pose.header.frame_id != self.cfg['global_frame']
                            or stamp_ns(pose.header.stamp) != stamp_ns(operation['scan'].header.stamp)
                            or not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w, score))
                            or not 0 <= score <= 1
                            or abs(q.x) > 1e-6 or abs(q.y) > 1e-6
                            or abs(sum(v*v for v in (q.x, q.y, q.z, q.w))-1) > .01):
                        raise ValueError('Invalid TIM laser pose, frame, stamp or score')
                    base_pose = compose_se2(compose_se2(planar(pose.pose), inverse_se2(operation['extrinsic'])), delta)
                    laser_pose = compose_se2(base_pose, current_extrinsic)
                    quality = self.map_quality.score(**self.scan_args(), laser_pose=laser_pose)
                    if (quality.origin_free and quality.valid_beams >= self.cfg['min_valid_beams']
                            and quality.match_ratio >= self.cfg['min_match_ratio']
                            and quality.known_ratio >= self.cfg['min_known_ratio']):
                        candidates.append((float(score), base_pose))
                if not candidates:
                    self.reject_tim('rejected_quality', 'TIM 후보와 최신 LiDAR 스캔의 지도 정렬이 부족합니다.')
                    return True
                score, base_pose = max(candidates, key=lambda candidate: candidate[0])
                seed = PoseWithCovarianceStamped()
                seed.header.frame_id = self.cfg['global_frame']
                seed.header.stamp = copy.deepcopy(self.scan.header.stamp)
                seed.pose.pose.position.x, seed.pose.pose.position.y = base_pose[:2]
                seed.pose.pose.orientation.z = math.sin(base_pose[2]/2)
                seed.pose.pose.orientation.w = math.cos(base_pose[2]/2)
                seed.pose.covariance[0] = seed.pose.covariance[7] = .04
                seed.pose.covariance[35] = .01
                self.tim_metrics.update(tim_status='accepted', tim_elapsed_sec=float(response.elapsed_sec),
                                        tim_candidates=len(response.poses), tim_score=score)
                self.tim_operation = None
                self.clear_evidence()
                self.auto_seed_echoes.append(hashlib.sha256(serialize_message(seed)).digest())
                self.initial_pose_pub.publish(seed)
                self.set_state('VERIFYING', 'TIM 위치 후보를 AMCL에 전달했습니다. 새 스캔과 전역 혼동 가능성을 검증합니다.')
            except Exception as exc:
                self.get_logger().warning(f'TIM initialization rejected: {type(exc).__name__}: {exc}')
                self.reject_tim('invalid_result', 'TIM 응답 또는 좌표 변환을 검증할 수 없습니다.')
        return True

    def fail_beams(self, reason):
        self.beam_failed = True
        self.good_since, self.good_count = None, 0
        self.set_state('FAILED', f'AMCL 빔 설정 실패: {reason} localization과 navigation을 재시작하세요.')

    def ensure_beams(self):
        """Serialize writes and confirm exact AMCL readback before using scans.

        Only max_beams is written. Changing particle counts here would reset
        the Jazzy particle filter and discard the location just established.
        A superseded request is drained, never cancelled and raced by a newer
        write. Timeouts latch a failure because remote execution is uncertain.
        """
        if self.beam_failed:
            self.fail_beams('설정 상태를 확인할 수 없습니다.')
            return False
        target = self.cfg[self.beam_phase + '_max_beams']
        if self.beam_confirmed_revision == self.beam_revision:
            return True
        now = time.monotonic()
        operation = self.beam_operation
        if operation is None:
            operation = dict(stage='waiting', target=target, revision=self.beam_revision,
                             started=now, future=None)
            self.beam_operation = operation
        if now-operation['started'] > self.cfg['service_timeout']:
            self.fail_beams('파라미터 서비스 응답 시간이 초과되었습니다.')
            return False
        try:
            if operation['stage'] == 'waiting':
                if self.beam_set_client.service_is_ready() and self.beam_get_client.service_is_ready():
                    request = SetParametersAtomically.Request(parameters=[Parameter(
                        name='max_beams', value=ParameterValue(
                            type=ParameterType.PARAMETER_INTEGER, integer_value=operation['target']))])
                    operation['future'] = self.beam_set_client.call_async(request)
                    operation['stage'] = 'setting'
            elif operation['future'].done():
                response = operation['future'].result()
                if operation['stage'] == 'setting':
                    if not response.result.successful:
                        self.fail_beams(response.result.reason or 'AMCL이 설정 변경을 거절했습니다.')
                        return False
                    operation['future'] = self.beam_get_client.call_async(GetParameters.Request(names=['max_beams']))
                    operation['stage'] = 'reading'
                else:
                    if (len(response.values) != 1
                            or response.values[0].type != ParameterType.PARAMETER_INTEGER
                            or response.values[0].integer_value != operation['target']):
                        self.fail_beams('AMCL에서 읽은 빔 수가 요청값과 다릅니다.')
                        return False
                    self.applied_beams = operation['target']
                    self.beam_operation = None
                    if operation['revision'] == self.beam_revision:
                        self.beam_confirmed_revision = self.beam_revision
                        self.clear_evidence()
                    # Even successful readback cannot reuse pre-change scans.
                    return False
        except Exception as exc:
            self.fail_beams(f'파라미터 서비스 오류: {type(exc).__name__}.')
            return False
        phase = '초기 탐색' if self.beam_phase == 'initialization' else '주행 추적'
        self.set_state('SEARCHING' if self.global_pending else 'VERIFYING',
                       f'{phase}용 AMCL {target}빔 설정과 실제 적용값을 확인합니다.')
        return False

    def fresh(self, name, timeout):
        return time.monotonic() - self.received.get(name, -math.inf) <= timeout

    def lifecycle_active(self, name):
        state, received = self.lifecycle_states.get(name, (0, -math.inf))
        return state == 3 and time.monotonic()-received <= 3.0

    def poll_lifecycle(self):
        now = time.monotonic()
        for name, (future, sent) in list(self.lifecycle_pending.items()):
            if future.done():
                try:
                    self.lifecycle_states[name] = (future.result().current_state.id, now)
                except Exception:
                    self.lifecycle_states.pop(name, None)
                del self.lifecycle_pending[name]
            elif now-sent > self.cfg['service_timeout']:
                future.cancel()
                del self.lifecycle_pending[name]
                self.lifecycle_states.pop(name, None)
        if now-self.lifecycle_poll >= 1.0:
            self.lifecycle_poll = now
            for name, client in self.lifecycle_clients.items():
                if name not in self.lifecycle_pending and client.service_is_ready():
                    self.lifecycle_pending[name] = (client.call_async(GetState.Request()), now)

    def data_problem(self):
        if self.map_quality is None:
            return '지도를 기다립니다.'
        if not self.lifecycle_active('amcl'):
            return 'AMCL 활성화를 기다립니다. localization을 실행하세요.'
        if self.scan_samples and not self.valid_scan(self.scan_samples[-1][0]):
            # History only tolerates TF delivery order, never a malformed or
            # unusable newly received sensor observation.
            return 'LiDAR 스캔 형식 또는 유효 측정 수가 올바르지 않습니다.'
        # DDS can deliver a scan before the odometry/TF covering its timestamp.
        # Use the newest transformable observation still inside the SAME
        # freshness bounds; never use extrapolation or silently latest TF.
        fallback = None
        for sample, received in reversed(self.scan_samples):
            age = (self.get_clock().now().nanoseconds-stamp_ns(sample.header.stamp))/1e9
            if (stamp_ns(sample.header.stamp) == 0 or not -.1 <= age <= self.cfg['data_timeout']
                    or time.monotonic()-received > self.cfg['data_timeout']
                    or not self.valid_scan(sample)):
                continue
            when = Time.from_msg(sample.header.stamp)
            if (self.tf_buffer.can_transform(self.cfg['odom_frame'], self.cfg['base_frame'], when)
                    and self.tf_buffer.can_transform(self.cfg['base_frame'], sample.header.frame_id, when)):
                if self.prior_verified and sample.time_increment > 0:
                    end = Time(nanoseconds=stamp_ns(sample.header.stamp) + round(
                        (len(sample.ranges)-1)*sample.time_increment*1e9))
                    # Rolling measurements need TF through the final beam, not
                    # just the first. DDS may deliver this scan before that TF.
                    if not self.tf_buffer.can_transform(self.cfg['odom_frame'], sample.header.frame_id, end):
                        continue
                if fallback is None:
                    fallback = (sample, received)
                if (not self.initialized or self.tf_buffer.can_transform(
                        self.cfg['global_frame'], sample.header.frame_id, when)):
                    fallback = (sample, received)
                    break
        if fallback is not None:
            self.scan, self.received['scan'] = fallback
        # /amcl_pose and /tf arrive on separate DDS streams. The latest pose
        # can precede the odometry TF needed at its stamp. Keep a complete,
        # transformable pair inside the original freshness/initialization bounds.
        # Malformed new poses still fail immediately rather than using history.
        newest_pose = self.pose_samples[-1][0] if self.pose_samples else self.pose
        for pose in (self.pose, newest_pose):
            if pose is not None and (pose.header.frame_id != self.cfg['global_frame']
                    or not valid_planar_pose(pose.pose.pose.position, pose.pose.pose.orientation)
                    or not np.isfinite(pose.pose.covariance).all()
                    or not covariance_is_bounded(pose.pose.covariance, math.inf, math.inf)):
                return 'AMCL 위치 또는 공분산이 유효하지 않습니다.'
        for pose, received in reversed(self.pose_samples):
            pose_stamp = stamp_ns(pose.header.stamp)
            age = (self.get_clock().now().nanoseconds-pose_stamp)/1e9
            if (pose_stamp < self.attempt_stamp or not -.1 <= age <= self.cfg['pose_timeout']
                    or time.monotonic()-received > self.cfg['pose_timeout']):
                continue
            if self.tf_buffer.can_transform(self.cfg['global_frame'], self.cfg['base_frame'],
                                            Time.from_msg(pose.header.stamp)):
                self.pose, self.received['pose'] = pose, received
                break
        for name, label in (('scan', 'LiDAR 스캔'), ('odom', '오도메트리')):
            message = getattr(self, name)
            if message is None or not self.fresh(name, self.cfg['data_timeout']):
                return f'{label} 수신을 기다립니다.'
            age = (self.get_clock().now().nanoseconds - stamp_ns(message.header.stamp))/1e9
            if stamp_ns(message.header.stamp) == 0 or not -.1 <= age <= self.cfg['data_timeout']:
                return f'{label} 시간 정보가 최신이 아닙니다.'
        if not self.valid_scan(self.scan):
            return 'LiDAR 스캔 형식 또는 유효 측정 수가 올바르지 않습니다.'
        if not self.valid_odom(self.odom):
            return '오도메트리 좌표계 또는 값이 유효하지 않습니다.'
        try:
            when = Time.from_msg(self.scan.header.stamp)
            self.tf_buffer.lookup_transform(self.cfg['odom_frame'], self.cfg['base_frame'], when)
            self.tf_buffer.lookup_transform(self.cfg['base_frame'], self.scan.header.frame_id, when)
        except TransformException:
            return 'odom·base·LiDAR 좌표 변환을 기다립니다.'
        return None

    def pose_evidence(self):
        # Initial confidence thresholds are admission tests, not a runtime
        # localization-loss detector. Still evaluate every validity check before
        # returning a confidence warning, so it cannot hide a real data fault.
        self.evidence_issue = 'hard'
        if stamp_ns(self.scan.header.stamp) < self.attempt_stamp:
            return None, '빔 설정·초기화 이후에 측정한 새 LiDAR 스캔을 기다립니다.'
        if (self.pose is None or not self.fresh('pose', self.cfg['pose_timeout'])
                or stamp_ns(self.pose.header.stamp) < self.attempt_stamp
                or self.pose.header.frame_id != self.cfg['global_frame']):
            return None, '초기화 이후 AMCL 위치 추정을 기다립니다.'
        covariance = np.asarray(self.pose.pose.covariance).reshape(6, 6)
        self.metrics.pop('xy_std', None)
        self.metrics.pop('yaw_std', None)
        if np.isfinite(covariance).all():
            xy_variance = float(np.max(np.linalg.eigvalsh(covariance[:2, :2])))
            for key, variance in (('xy_std', xy_variance), ('yaw_std', covariance[5, 5])):
                if math.isfinite(variance) and variance >= 0:
                    self.metrics[key] = math.sqrt(variance)
        if (not np.isfinite(covariance).all()
                or not covariance_is_bounded(self.pose.pose.covariance, math.inf, math.inf)):
            return None, 'AMCL 공분산이 유효하지 않습니다.'
        confidence_problem = None
        if not covariance_is_bounded(self.pose.pose.covariance, self.cfg['max_xy_std'], self.cfg['max_yaw_std']):
            xy, heading = self.metrics.get('xy_std'), self.metrics.get('yaw_std')
            xy_text = '알 수 없음' if xy is None else f'{xy:.3f}m/{self.cfg["max_xy_std"]:.3f}m'
            yaw_text = ('알 수 없음' if heading is None else
                        f'{math.degrees(heading):.2f}도/{math.degrees(self.cfg["max_yaw_std"]):.2f}도')
            confidence_problem = f'초기 위치 검증 기준보다 불확실성이 큽니다. 위치 표준편차 {xy_text}, 방향 표준편차 {yaw_text}.'
        try:
            when = Time.from_msg(self.scan.header.stamp)
            transform = self.tf_buffer.lookup_transform(self.cfg['global_frame'], self.scan.header.frame_id, when)
            base = self.tf_buffer.lookup_transform(self.cfg['global_frame'], self.cfg['base_frame'], when)
            laser_pose = (transform.transform.translation.x, transform.transform.translation.y, yaw(transform.transform.rotation))
            base_pose = (base.transform.translation.x, base.transform.translation.y, yaw(base.transform.rotation))
            pose_tf = self.tf_buffer.lookup_transform(self.cfg['global_frame'], self.cfg['base_frame'],
                                                     Time.from_msg(self.pose.header.stamp))
            reported = planar(self.pose.pose.pose)
            implied = (pose_tf.transform.translation.x, pose_tf.transform.translation.y,
                       yaw(pose_tf.transform.rotation))
            if (not all(math.isfinite(v) for v in reported)
                    or math.hypot(reported[0]-implied[0], reported[1]-implied[1]) > self.cfg['max_xy_std']
                    or abs(math.atan2(math.sin(reported[2]-implied[2]), math.cos(reported[2]-implied[2]))) > self.cfg['max_yaw_std']):
                return None, 'AMCL 추정 위치와 지도 좌표 변환이 일치하지 않습니다.'
            # AMCL must have actually processed a contemporary scan, rather
            # than borrowing an old covariance for a newly published TF.
            if abs(stamp_ns(self.scan.header.stamp)-stamp_ns(self.pose.header.stamp)) > self.cfg['pose_timeout']*1e9:
                return None, '스캔과 AMCL 추정 시각이 맞지 않습니다.'
            ray_poses = self.tracking_ray_poses(laser_pose)
            quality = self.map_quality.score(**self.scan_args(), laser_pose=laser_pose,
                                            ray_poses=ray_poses)
            raw_quality = (self.map_quality.score(**self.scan_args(), laser_pose=laser_pose)
                           if ray_poses is not None else quality)
        except (TransformException, ValueError) as exc:
            self.metrics['observation_error'] = str(exc)[:600]
            return None, '지도 좌표 변환 또는 유효한 스캔을 기다립니다. ' + str(exc)[:300]
        self.metrics.update(match_ratio=quality.match_ratio, known_ratio=quality.known_ratio,
                            obstruction_ratio=quality.obstruction_ratio,
                            uncompensated_match_ratio=raw_quality.match_ratio,
                            valid_beams=quality.valid_beams)
        if not quality.origin_free:
            return None, '추정된 LiDAR 위치가 지도의 이동 가능한 영역 밖에 있습니다.'
        if quality.valid_beams < self.cfg['min_valid_beams']:
            return None, '위치를 검증할 유효 LiDAR 측정이 부족합니다.'
        if confidence_problem:
            self.evidence_issue = 'confidence'
            return ((laser_pose, quality.match_ratio) if self.prior_verified else None), confidence_problem
        if (quality.match_ratio < self.cfg['min_match_ratio']
                or quality.known_ratio < self.cfg['min_known_ratio']):
            self.evidence_issue = 'scan_alignment'
            return ((laser_pose, quality.match_ratio) if self.prior_verified else None), (
                '지도와 LiDAR 스캔 정렬이 부족합니다. '
                f'일치율 {quality.match_ratio:.1%}/{self.cfg["min_match_ratio"]:.0%}, '
                f'회전·이동 보정 전 {raw_quality.match_ratio:.1%}, '
                f'지도 내 관측 {quality.known_ratio:.1%}/{self.cfg["min_known_ratio"]:.0%}, '
                f'벽 관통 {quality.obstruction_ratio:.1%}, '
                f'스캔 중 회전 {math.degrees(self.metrics.get("scan_motion_yaw", 0.0)):.2f}도.')
        if self.source == 'auto' and not self.prior_verified:
            if (self.cloud is None or not self.fresh('cloud', self.cfg['pose_timeout'])
                    or stamp_ns(self.cloud.header.stamp) < self.attempt_stamp
                    or self.cloud.header.frame_id != self.cfg['global_frame']):
                return None, '초기화 이후 AMCL 위치 후보 분포를 기다립니다.'
            mass = particle_concentration([planar(p.pose) for p in self.cloud.particles],
                                          base_pose, [p.weight for p in self.cloud.particles])
            self.metrics['particle_concentration'] = mass
            if mass < self.cfg['min_particle_concentration']:
                return None, '여러 위치·방향 후보가 남아 있습니다.'
        self.evidence_issue = None
        return (laser_pose, quality.match_ratio), None

    def clear_tracking_warning(self):
        self.tracking_bad_since = None
        self.tracking_bad_count = 0
        self.tracking_bad_last_scan = 0
        self.tracking_warning = ''

    def tracking_ray_poses(self, laser_pose):
        """Place each rolling-scan ray using odometry at its measurement time.

        Keep one map correction for the whole scan. Only odometry contributes
        intra-scan motion, so a changing AMCL correction cannot warp the rays.
        A missing exact-time transform fails the observation; never substitute
        latest TF, commanded velocity, or a guessed constant turn rate.
        Global initialization is stationary and keeps its original scoring.
        """
        self.metrics.update(scan_motion_compensated=False, scan_motion_translation=0.0,
                            scan_motion_yaw=0.0, scan_duration=0.0)
        scan = self.scan
        if not self.prior_verified or scan.time_increment == 0:
            return None
        if not self.valid_scan(scan):
            raise ValueError('Invalid rolling scan metadata')
        args = self.scan_args()
        indices, _, _ = self.map_quality.sampled_rays(**args)
        start_ns = stamp_ns(scan.header.stamp)
        start = self.checked_transform(self.cfg['odom_frame'], scan.header.frame_id,
                                       Time.from_msg(scan.header.stamp))
        correction = compose_se2(laser_pose, inverse_se2(start))
        poses = []
        duration = (len(scan.ranges)-1)*scan.time_increment
        # Check even an invalid/no-return final beam's time. Completeness must
        # not depend on which ranges happened to survive subsampling.
        end = self.checked_transform(self.cfg['odom_frame'], scan.header.frame_id,
                                     Time(nanoseconds=start_ns + round(duration*1e9)))
        inverse_start = inverse_se2(start)
        delta = compose_se2(inverse_start, end)
        translation, rotation = math.hypot(delta[0], delta[1]), abs(delta[2])
        for index in indices:
            odom_laser = self.checked_transform(
                self.cfg['odom_frame'], scan.header.frame_id,
                Time(nanoseconds=start_ns + round(int(index)*scan.time_increment*1e9)))
            delta = compose_se2(inverse_start, odom_laser)
            translation = max(translation, math.hypot(delta[0], delta[1]))
            rotation = max(rotation, abs(delta[2]))
            poses.append(compose_se2(correction, odom_laser))
        self.metrics.update(scan_motion_compensated=True, scan_duration=duration,
                            scan_motion_translation=translation, scan_motion_yaw=rotation)
        return np.asarray(poses, dtype=float).reshape(-1, 3)

    def record_tracking_warning(self, problem, now):
        """Report confidence changes without undoing completed initialization."""
        if self.tracking_bad_since is None:
            self.tracking_bad_since = now
            self.get_logger().warning(
                '위치 추적 품질 경고입니다. 초기화를 다시 실행하거나 목표를 취소하지 않습니다. ' + problem)
        self.tracking_warning = problem
        scan_stamp = stamp_ns(self.scan.header.stamp)
        if scan_stamp > self.tracking_bad_last_scan:
            self.tracking_bad_last_scan = scan_stamp
            self.tracking_bad_count += 1
        # Counts and duration are diagnostics, never a second startup deadline.
        # Frozen observations still expire through the data/pose watchdogs.

    def scan_args(self):
        s = self.scan
        return dict(ranges=list(s.ranges), angle_min=s.angle_min, angle_increment=s.angle_increment,
                    range_min=s.range_min, range_max=s.range_max,
                    max_beams=self.cfg['quality_max_beams'], max_range=self.quality_range())

    def quality_range(self):
        # Global uniqueness needs distant structure. Tracking has a verified
        # prior and must use the measurement range AMCL actually assimilates.
        return self.cfg['tracking_quality_max_range' if self.prior_verified else 'quality_max_range']

    @staticmethod
    def same_candidate(a, b):
        return (a is not None and b is not None and math.hypot(a[0]-b[0], a[1]-b[1]) <= .25
                and abs(math.atan2(math.sin(a[2]-b[2]), math.cos(a[2]-b[2]))) <= .15)

    def check_rivals(self, candidate, score):
        if self.rival_future is not None and self.rival_future.done():
            try:
                result = self.rival_future.result()
                if (self.rival_generation == self.generation and self.rival_map_key == self.map_key
                        and self.rival_evidence_epoch == self.evidence_epoch
                        and self.rival_motion_revision == self.motion_revision):
                    self.rival_result = result
                    self.rival_completed_at = time.monotonic()
                    self.metrics['rival_calculation_sec'] = self.rival_completed_at-self.rival_submitted_at
            except Exception as exc:
                self.get_logger().warning(f'Global ambiguity check failed: {exc}')
            self.rival_future = None
        if (not self.same_candidate(candidate, self.rival_pose)
                or self.rival_motion_revision != self.motion_revision
                or (self.rival_result is not None
                    and time.monotonic()-self.rival_completed_at > 5.0)):
            self.rival_result = None
        if self.rival_result is None:
            if self.rival_future is None:
                self.rival_pose = candidate
                self.rival_submitted_score = score
                self.rival_submitted_at = time.monotonic()
                self.rival_motion_revision = self.motion_revision
                self.rival_generation, self.rival_map_key = self.generation, self.map_key
                self.rival_evidence_epoch = self.evidence_epoch
                self.rival_future = self.worker.submit(
                    self.map_quality.find_rival, **self.scan_args(), laser_pose=candidate,
                    position_step=self.cfg['rival_position_step'], yaw_step=self.cfg['rival_yaw_step'],
                    max_candidates=self.cfg['rival_max_candidates'])
            return '다른 복도 위치와 혼동될 가능성을 확인합니다.'
        result = self.rival_result
        self.metrics.update(rival_score=result.best_score, rival_search_completed=result.completed,
                            rival_candidates=result.candidates_evaluated)
        if not result.completed:
            return '지도 후보 검사 한도를 초과했습니다. 초기 위치를 직접 지정하세요.'
        # Both scores must refer to the worker's same scan. A later scan with
        # fewer moving obstacles must not make an old ambiguous scan unique.
        if result.best_score >= min(score, self.rival_submitted_score)-self.cfg['ambiguity_margin']:
            return '유사한 스캔이 맞는 다른 위치가 있습니다. 초기 위치를 직접 지정하세요.'
        return None

    def request_update(self):
        now = time.monotonic()
        if self.nomotion_future is not None:
            if self.nomotion_future.done():
                self.nomotion_future = None
            elif now-self.nomotion_requested > self.cfg['service_timeout']:
                self.nomotion_future.cancel()
                self.nomotion_future = None
        interval = self.cfg['tracking_update_interval' if self.state == 'READY'
                            else 'initialization_update_interval']
        if (self.nomotion_future is None and now-self.nomotion_requested >= interval
                and self.nomotion_client.service_is_ready()):
            self.nomotion_requested = now
            self.nomotion_future = self.nomotion_client.call_async(Empty.Request())

    def set_state(self, state, detail):
        now = time.monotonic()
        if (state != self.state
                or (detail != self.last_logged_detail and now-self.last_state_log >= 1.0)):
            self.get_logger().info(f'{state}: {detail}')
            self.last_logged_detail, self.last_state_log = detail, now
        self.state, self.detail = state, detail
        self.publish_status()

    def verification_requirements(self):
        prefix = ('tracking_' if self.prior_verified else
                  'manual_' if self.source == 'manual' else '')
        return (self.cfg[prefix+'verification_duration'],
                self.cfg[prefix+'verification_samples'])

    def publish_status(self):
        duration, samples = self.verification_requirements()
        self.status_pub.publish(String(data=json.dumps(dict(
            state=self.state, ready=self.state == 'READY', detail=self.detail,
            source=self.source, generation=self.generation, session=self.session,
            stamp=time.time(), amcl_beam_phase=self.beam_phase,
            amcl_max_beams=self.applied_beams, quality_max_beams=self.cfg['quality_max_beams'],
            quality_max_range=self.quality_range(),
            initializer_backend=self.cfg['global_localization_backend'],
            prior_verified=self.prior_verified, tracking_warning=self.tracking_warning,
            tracking_bad_samples=self.tracking_bad_count,
            tracking_bad_duration=(0.0 if self.tracking_bad_since is None
                                   else max(0.0, time.monotonic()-self.tracking_bad_since)),
            verification_samples=self.good_count, verification_required_samples=samples,
            verification_elapsed=(0.0 if self.good_since is None
                                  else max(0.0, time.monotonic()-self.good_since)),
            verification_required_duration=duration,
            **self.tim_metrics, **self.metrics), ensure_ascii=False, allow_nan=False)))

    def tick(self):
        try:
            self.advance()
        except Exception as exc:
            self.good_since, self.good_count = None, 0
            self.set_state('FAILED', f'시작 상태 확인 오류: {type(exc).__name__}. navigation을 재시작하세요.')
            self.get_logger().error(str(exc))
        self.publish_status()

    def advance(self):
        now = time.monotonic()
        self.poll_lifecycle()
        if self.state == 'FAILED':
            return
        if self.suspended:
            return
        if self.beam_failed:
            self.fail_beams('설정 상태를 확인할 수 없습니다.')
            return
        if self.advance_tim():
            return
        if self.global_future is not None:
            if self.global_future.done():
                try:
                    self.global_future.result()
                    if self.global_generation == self.generation:
                        # Samples emitted while the service was queued can
                        # still belong to the previous particle distribution.
                        self.clear_evidence()
                except Exception:
                    self.reset_uncertain = True
                    self.initialized = False
                    self.set_state('FAILED', 'AMCL 초기화 완료 여부가 불명확합니다. localization과 navigation을 재시작하세요.')
                self.global_future = None
            elif now-self.global_requested > self.cfg['service_timeout']:
                self.global_future.cancel()
                self.global_future = None
                # Cancelling a local Future cannot retract an AMCL operation.
                # Never accept a later manual seed that a delayed reset could
                # overwrite. Restart both processes to clear this uncertainty.
                self.reset_uncertain = True
                self.initialized = False
                self.set_state('FAILED', 'AMCL 초기화 응답이 없습니다. localization과 navigation을 재시작하세요.')
            if self.reset_uncertain or self.global_future is not None:
                return
        problem = self.data_problem()
        if problem:
            self.clear_tracking_warning()
            self.good_since, self.good_count = None, 0
            if self.state == 'READY':
                # A resumed tracker must supply observations acquired after
                # the fault. Keep the verified prior and tracking beam profile.
                self.clear_evidence()
            if self.state != 'WAITING_INITIAL_POSE':
                self.set_state('WAITING_DATA', problem)
            return
        if not self.initialized:
            if (self.cfg['automatic_localization'] and not self.auto_attempted
                    and self.stationary() and not self.goal_in_progress()
                    and self.pose is not None
                    and self.tf_buffer.can_transform(self.cfg['global_frame'], self.cfg['base_frame'],
                                                     Time.from_msg(self.scan.header.stamp))):
                # Navigation can start after a human has already initialized
                # AMCL. Preserve that estimate rather than blindly replacing
                # it with uniform particles. Require NEW evidence and the full
                # automatic ambiguity check because the prior's source is not
                # known to this new supervisor process.
                self.new_generation('auto')
                self.initialized = self.auto_attempted = True
                self.set_state('VERIFYING', '기존 AMCL 위치 추정을 보존하고 새 스캔으로 검증합니다.')
            elif (self.cfg['automatic_localization'] and not self.auto_attempted
                    and self.stationary() and not self.goal_in_progress()
                    and self.initializer_ready()):
                self.begin_global()
            elif self.state != 'WAITING_INITIAL_POSE':
                self.set_state('WAITING_INITIAL_POSE', '정지 상태의 자동 탐색 또는 Web UI 초기 위치 지정을 기다립니다.')
            return
        if self.state == 'WAITING_INITIAL_POSE':
            return
        if not self.ensure_beams():
            return
        if self.global_pending:
            # Parameter handshakes take time; recheck admission immediately
            # before the reset rather than trusting its earlier request time.
            if not self.stationary() or self.goal_in_progress():
                self.set_state('SEARCHING', '자동 탐색을 시작하려면 목표를 취소하고 로봇을 정지해 주세요.')
                return
            if not self.initializer_ready():
                self.set_state('SEARCHING', '선택한 전역 위치 탐색 서비스를 기다립니다.')
                return
            self.global_pending = False
            if self.cfg['global_localization_backend'] == 'tim':
                self.request_tim()
                return
            self.set_state('SEARCHING', '지도 전체에 위치 후보를 배치해 자동 탐색합니다. 로봇을 정지해 두세요.')
            self.global_requested = now
            self.global_future = self.global_client.call_async(Empty.Request())
            self.global_generation = self.generation
            return
        self.request_update()
        # Confidence diagnostics must not mask a dead Nav2 server.
        if (self.prior_verified and self.state == 'READY'
                and not all(self.lifecycle_active(n) for n in self.lifecycle_names[1:])):
            self.clear_tracking_warning()
            self.set_state('FAILED', 'Nav2 활성 상태를 잃었습니다. navigation을 재시작하세요.')
            return
        evidence, problem = self.pose_evidence()
        if not problem and self.source == 'auto' and not self.prior_verified:
            problem = self.check_rivals(*evidence)
        soft_problem = None
        if (problem and self.prior_verified
                and self.evidence_issue in ('scan_alignment', 'confidence')):
            self.record_tracking_warning(problem, now)
            if self.state == 'READY':
                self.set_state('READY', '초기 위치 검증 완료. 위치 추적 품질 경고를 확인하세요.')
                return
            # Recovery needs fresh stopped observations and valid TF, not a
            # second global-confidence admission test for the retained prior.
            soft_problem, problem = problem, None
        if problem:
            self.clear_tracking_warning()
            self.good_since, self.good_count = None, 0
            if self.state == 'READY':
                self.clear_evidence()
            decided_rival = (self.source == 'auto' and not self.prior_verified
                              and self.evidence_issue is None and self.rival_result is not None)
            if decided_rival or now-self.attempt_started > self.cfg['search_timeout']:
                self.set_state('WAITING_INITIAL_POSE', problem + ' 초기 위치를 지정하거나 자동 탐색을 재시도하세요.')
            else:
                state = 'SEARCHING' if self.source == 'auto' and not self.prior_verified else 'VERIFYING'
                detail = '주행 위치 추적 검증 실패: ' + problem if self.prior_verified else problem
                self.set_state(state, detail)
            return
        if not soft_problem:
            if self.tracking_warning and self.state == 'READY':
                self.get_logger().info('위치 추적 품질이 초기 검증 기준을 다시 충족합니다. 기존 주행을 유지합니다.')
            self.clear_tracking_warning()
        if self.prior_verified and self.state != 'READY' and not self.stationary():
            self.good_since, self.good_count = None, 0
            self.set_state('VERIFYING', '주행 위치 추적을 재검증합니다. 로봇이 정지하기를 기다립니다.')
            return
        if (self.prior_verified and self.state != 'READY'
                and min(stamp_ns(self.scan.header.stamp), stamp_ns(self.pose.header.stamp))
                <= self.last_motion_stamp):
            self.good_since, self.good_count = None, 0
            self.set_state('VERIFYING', '정지 후 측정한 새 스캔과 AMCL 추정으로 위치 추적을 확인합니다.')
            return
        candidate, _ = evidence
        if self.state != 'READY':
            if self.good_since is None or not self.same_candidate(candidate, self.last_candidate):
                self.good_since, self.good_count = None, 0
                # Bound the whole observation window, not each small tick-to-tick step.
                self.last_candidate = candidate
            scan_stamp = stamp_ns(self.scan.header.stamp)
            pose_stamp = stamp_ns(self.pose.header.stamp)
            paired = self.source == 'manual' or self.prior_verified
            fresh_pair = (scan_stamp > self.last_checked_scan and pose_stamp > self.last_checked_pose)
            new_sample = fresh_pair if paired else scan_stamp != self.last_checked_scan
            if new_sample:
                self.last_checked_scan = scan_stamp
                self.last_checked_pose = pose_stamp
                if self.good_since is None:
                    self.good_since = now
                self.good_count += 1
            duration, samples = self.verification_requirements()
            if (self.good_since is None or self.good_count < samples
                    or now-self.good_since < duration):
                detail = ('기존 위치 추적의 회복을 확인합니다. 전역 초기화를 다시 실행하지 않습니다.'
                          if self.prior_verified else '스캔과 위치 추정이 안정적으로 유지되는지 확인합니다.')
                self.set_state('VERIFYING', detail)
                return
        if self.beam_phase == 'initialization':
            self.beam_phase = 'tracking'
            self.beam_revision += 1
            self.clear_evidence()
            self.set_state('VERIFYING', '초기 위치가 확인되었습니다. 주행용 빔 설정으로 전환 후 다시 검증합니다.')
            return
        nav_active = all(self.lifecycle_active(n) for n in self.lifecycle_names[1:])
        if nav_active:
            self.start_future = None
            self.prior_verified = True
            detail = ('기존 위치 추적과 Nav2 활성 상태가 회복되었습니다. 위치 추적 품질 경고를 확인하세요.'
                      if soft_problem else '위치 검증과 Nav2 활성화가 완료되었습니다. 목표를 지정할 수 있습니다.')
            self.set_state('READY', detail)
            return
        if self.state == 'READY':
            self.set_state('FAILED', 'Nav2 활성 상태를 잃었습니다. navigation을 재시작하세요.')
            return
        if self.start_future is not None:
            if self.start_future.done():
                try:
                    success = self.start_future.result().success
                except Exception:
                    success = False
                if not success:
                    self.set_state('FAILED', 'Nav2 활성화에 실패했습니다. 로그 확인 후 navigation만 재시작하세요.')
                    return
                # A successful service response is followed by lifecycle reads.
            if now-self.start_requested > self.cfg['navigation_start_timeout']:
                self.set_state('FAILED', 'Nav2 활성화 응답 시간이 초과되었습니다. navigation만 재시작하세요.')
                return
            self.set_state('STARTING_NAVIGATION', '위치가 확인되어 Nav2를 활성화하고 있습니다.')
            return
        if self.start_sent:
            self.set_state('FAILED', 'Nav2 상태가 일관되지 않습니다. navigation만 재시작하세요.')
            return
        if not self.start_client.service_is_ready():
            self.set_state('STARTING_NAVIGATION', 'Nav2 lifecycle 서비스를 기다립니다.')
            return
        self.start_sent = True
        self.start_requested = now
        self.set_state('STARTING_NAVIGATION', '위치가 확인되어 Nav2를 활성화하고 있습니다.')
        self.start_future = self.start_client.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP))

    def destroy_node(self):
        self.cancel_tim('shutdown')
        self.worker.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationStartup()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
