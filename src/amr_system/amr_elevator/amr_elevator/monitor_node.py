"""elevator_monitor: LiDAR door state + current floor / inside-car estimation.

Shared by the real vehicle and the simulation; it only reads the real AMR
interfaces (/scan, /odometry/filtered, TF, /localization/status) and the
elevator topics (/elevator/request, /floor/arrival_confirm).

Inside the car AMCL cannot be trusted, so the robot position is the last pose
verified by localization outside the car carried forward by odometry
(``anchor`` = map->odom at that time).  When the floor changes the anchor is
moved with the car: the pose relative to the car is kept on the new floor map.
"""

import json
import math

import numpy as np

from amr_elevator_interfaces.msg import (
    ElevatorRequest, FloorArrivalConfirm, ElevatorDoorState, FloorEvent, FloorState, FloorVisionGuess,
)
from amr_elevator_interfaces.srv import SetCurrentFloor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


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
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from amr_elevator.car_matcher import car_wall_segments, fit_car
from amr_elevator.door_detector import NAMES as DOOR_NAMES, DoorDetector, enclosure_ratio
from amr_elevator.floor_state import ZONE_IN_ELEVATOR, FloorStateMachine
from amr_elevator.geometry import compose, depth_beyond, inverse, point_in_polygon

HALF_BODY = .328


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose_of_transform(transform):
    t = transform.transform
    return (t.translation.x, t.translation.y, yaw_of(t.rotation))


class ElevatorMonitor(Node):
    def __init__(self, building=None, **kwargs):
        super().__init__('elevator_monitor', **kwargs)
        from amr_stack.floor_config import load_floor_config
        p = {k: self.declare_parameter(k, v).value for k, v in {
            'floors_config': '', 'elevator_id': 'E1', 'initial_floor': '',
            'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_footprint',
            'exit_margin': .37, 'fully_inside_margin': .05,
            'enclosure_near': 3., 'enclosure_ratio': .75,
            'stationary_speed': .05, 'publish_period': 1., 'arrival_settle_s': 1.,
            'vision_wait_s': 0., 'vision_auto': False,
            'vision_auto_min_score': .93, 'vision_auto_min_margin': .03,
            'car_wall_correction': True, 'car_correction_gain': .5,
            'door_open_ratio': .7, 'door_closed_ratio': .2, 'door_confirm_scans': 2,
        }.items()}
        self.p = p
        self.building = building or load_floor_config(p['floors_config'] or None)
        self.elevator_id = p['elevator_id']
        self._geometry_cache = {}
        self.machine = FloorStateMachine(
            self.building, p['initial_floor'] or None, arrival_settle_s=p['arrival_settle_s'],
            vision_wait_s=p['vision_wait_s'], vision_auto=p['vision_auto'],
            vision_auto_min_score=p['vision_auto_min_score'], vision_auto_min_margin=p['vision_auto_min_margin'])
        self.detector = DoorDetector(open_ratio=p['door_open_ratio'],
                                     closed_ratio=p['door_closed_ratio'],
                                     confirm_scans=p['door_confirm_scans'])
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self, spin_thread=False)
        self.anchor = None          # map->odom on the map of anchor_floor
        self.anchor_floor = None
        self.odom_pose = None
        self.speed = (0., 0.)
        self.laser_offset = None    # base->laser
        self.enclosed = False
        self.ready = False
        self.car_corrections = 0
        self.last_published = None
        self.next_publish = 0.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(FloorState, 'floor/state', latched)
        self.event_pub = self.create_publisher(FloorEvent, 'floor/events', 20)
        self.door_pub = self.create_publisher(ElevatorDoorState, 'elevator/door', 10)
        # Verified-localization anchor + odometry, on the current floor's map (valid inside the car).
        self.pose_pub = self.create_publisher(PoseStamped, 'floor/pose_estimate', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odometry/filtered', self.on_odometry, 20)
        self.create_subscription(String, 'localization/status', self.on_localization, latched)
        requests = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ElevatorRequest, '/elevator/request', self.on_request, requests)
        self.create_subscription(FloorArrivalConfirm, '/floor/arrival_confirm', self.on_arrival_confirm, 10)
        guesses = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(FloorVisionGuess, '/floor/vision_guess', self.on_vision_guess, guesses)
        self.create_service(SetCurrentFloor, 'floor/set_current', self.on_set_floor)
        self.create_timer(.1, self.tick)
        self.get_logger().info(f'Elevator monitor: {sorted(self.building.floors)}, '
                               f'start {self.machine.state.floor} (unconfirmed)')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # Geometry -------------------------------------------------------------------

    def current_floor(self):
        s = self.machine.state
        # Pass floors have no map: retain the last mapped car frame while riding.
        for floor in (s.floor, s.transit_from, self.anchor_floor):
            if floor in self.building.floors:
                return floor
        return self.machine.building.default_floor

    def geometry(self, floor=None):
        key = (floor or self.current_floor(), self.elevator_id)
        if key not in self._geometry_cache:
            self._geometry_cache[key] = self.building.elevator_geometry(*key)
        return self._geometry_cache[key]

    def inward_yaw(self, floor=None):
        return self.building.floor(floor or self.current_floor()).elevators[self.elevator_id].car_inward_yaw

    def estimated_pose(self):
        if self.anchor is None or self.odom_pose is None:
            return None
        return compose(self.anchor, self.odom_pose)

    def refresh_anchor(self):
        s = self.machine.state
        if not self.ready or s.zone == ZONE_IN_ELEVATOR or s.confidence == 'TRANSIT':
            return
        try:
            transform = self.tf.lookup_transform(self.p['map_frame'], self.p['odom_frame'], Time())
        except TransformException:
            return
        self.anchor = pose_of_transform(transform)
        self.anchor_floor = self.current_floor()

    def follow_car(self):
        """Keep the pose relative to the car when the car changes floor."""
        floor = self.machine.state.floor
        if self.anchor is None or floor not in self.building.floors or floor == self.anchor_floor:
            return
        old_car = self.geometry(self.anchor_floor).car_pose
        new_car = self.geometry(floor).car_pose
        self.anchor = compose(compose(new_car, inverse(old_car)), self.anchor)
        self.anchor_floor = floor

    # Inputs ---------------------------------------------------------------------

    def on_odometry(self, message):
        pose = message.pose.pose
        self.odom_pose = (pose.position.x, pose.position.y, yaw_of(pose.orientation))
        self.speed = (math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y),
                      abs(message.twist.twist.angular.z))

    def on_localization(self, message):
        try:
            status = json.loads(message.data)
        except ValueError:
            return
        self.ready = status.get('state') == 'READY' and status.get('ready') is True
        self.machine.on_localization(self.ready, self.now())

    def on_request(self, message):
        if message.kind != ElevatorRequest.KIND_CAR_FLOOR:
            return
        try:
            self.machine.on_car_target(message.target_floor)
        except ValueError as error:
            self.get_logger().warning(f'Ignoring elevator request: {error}')

    def on_arrival_confirm(self, message):
        verdict = self.machine.on_arrival_confirm(
            message.source, message.floor, message.candidate_id, message.accepted, self.now(),
            actual_floor=message.actual_floor)
        self.get_logger().info(f'Arrival confirmation {message.floor}: {verdict}')

    def on_vision_guess(self, message):
        # Frame time ties the evidence to the current opening, even after delivery delay.
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        verdict = self.machine.on_vision_guess(message.floor, stamp, message.score, message.margin,
                                               message.reference_floors)
        self.get_logger().info(f'Vision guess {message.floor}: {verdict} '
                               f'score={message.score:.3f} margin={message.margin:.3f}')

    def on_scan(self, message):
        if self.laser_offset is None:
            try:
                transform = self.tf.lookup_transform(self.p['base_frame'], message.header.frame_id,
                                                     Time(), Duration(seconds=0.))
            except TransformException:
                return
            self.laser_offset = pose_of_transform(transform)
        ranges = np.asarray(message.ranges, dtype=float)
        self.enclosed = enclosure_ratio(ranges, message.range_min, self.p['enclosure_near']) \
            >= self.p['enclosure_ratio']
        base = self.estimated_pose()
        if base is None:
            return
        geometry = self.geometry()
        inside = point_in_polygon(base[0], base[1], geometry.car_polygon)
        # Only with the whole body inside: in the doorway the hall walls confuse the fit.
        if (inside and self.p['car_wall_correction']
                and depth_beyond(base, geometry.car_door, self.inward_yaw()) >= .42):
            base = self.correct_in_car(base, message, ranges, geometry)
        door = geometry.car_door if (inside or self.machine.state.zone == ZONE_IN_ELEVATOR) \
            else geometry.hall_door
        reading = self.detector.update(compose(base, self.laser_offset), door, ranges,
                                       message.angle_min, message.angle_increment,
                                       message.range_min, message.range_max)
        self.machine.on_door(reading.state, self.now())
        out = ElevatorDoorState(elevator_id=self.elevator_id, state=reading.state,
                                open_ratio=float(reading.open_ratio), valid_beams=reading.valid_beams)
        out.header = message.header
        self.door_pub.publish(out)

    def correct_in_car(self, base, message, ranges, geometry):
        """Fit the scan to the car's interior walls and move the anchor accordingly."""
        laser = compose(base, self.laser_offset)
        ranges = np.asarray(ranges, dtype=float)
        indices = np.flatnonzero(np.isfinite(ranges) & (ranges >= message.range_min) & (ranges <= 4.))
        values = ranges[indices]
        angles = laser[2] + message.angle_min + indices * message.angle_increment
        points = np.column_stack((laser[0] + values * np.cos(angles),
                                  laser[1] + values * np.sin(angles)))
        fit = fit_car(points, car_wall_segments(geometry), base[:2], max_distance=.3)
        if fit is None or math.hypot(fit.dx, fit.dy) > .35 or abs(fit.dyaw) > .3:
            return base
        gain = self.p['car_correction_gain']
        angle = gain * fit.dyaw
        c, s = math.cos(angle), math.sin(angle)
        px, py = fit.pivot
        correction = (px + gain * fit.dx - (c * px - s * py), py + gain * fit.dy - (s * px + c * py), angle)
        self.anchor = compose(correction, self.anchor)
        self.car_corrections += 1
        if self.car_corrections % 50 == 1:
            self.get_logger().info(f'Car wall correction #{self.car_corrections}: '
                                   f'{fit.dx:+.3f} {fit.dy:+.3f} m {math.degrees(fit.dyaw):+.1f} deg '
                                   f'rms {fit.rms:.3f} ({fit.inliers} points)')
        return self.estimated_pose()

    def on_set_floor(self, request, response):
        stationary = self.speed[0] < self.p['stationary_speed'] and self.speed[1] < self.p['stationary_speed']
        try:
            self.machine.set_floor(request.floor, stationary)
            self.anchor = self.anchor_floor = None  # wait for localization on that floor's map
            response.success, response.message = True, f'floor set to {request.floor}'
        except Exception as error:  # noqa: BLE001 - reported to the operator
            response.success, response.message = False, str(error)
        return response

    # Output ---------------------------------------------------------------------

    def tick(self):
        now = self.now()
        self.follow_car()
        self.refresh_anchor()
        base = self.estimated_pose()
        if base is not None:
            estimate = PoseStamped()
            estimate.header.stamp = self.get_clock().now().to_msg()
            estimate.header.frame_id = self.p['map_frame']
            estimate.pose.position.x, estimate.pose.position.y = base[0], base[1]
            estimate.pose.orientation.z = math.sin(base[2] / 2)
            estimate.pose.orientation.w = math.cos(base[2] / 2)
            self.pose_pub.publish(estimate)
            geometry = self.geometry()
            inside = point_in_polygon(base[0], base[1], geometry.car_polygon)
            past_hall = depth_beyond(base, geometry.hall_door, self.inward_yaw()) < -self.p['exit_margin']
            fully_inside = depth_beyond(base, geometry.car_door, self.inward_yaw()) >= (
                HALF_BODY + self.p['fully_inside_margin'])
            self.machine.on_position(inside, past_hall, self.enclosed, now, fully_inside=fully_inside)
        state = self.machine.tick(now)
        self.follow_car()
        for record in self.machine.drain_events():
            message = FloorEvent(event=record.event, from_floor=record.from_floor,
                                 to_floor=record.to_floor, detail=record.detail)
            message.header.stamp = self.get_clock().now().to_msg()
            self.event_pub.publish(message)
            text = f'{record.event} {record.from_floor}->{record.to_floor} {record.detail}'.strip()
            # rclpy forbids changing the severity of one call site, so keep two call sites.
            if record.event == 'RULE_VIOLATION':
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)
        if state != self.last_published or now >= self.next_publish:
            message = FloorState(
                zone=state.zone, floor=state.floor, transit_from=state.transit_from,
                transit_to=state.transit_to, confidence=state.confidence, last_event=state.last_event,
                awaiting_confirm=state.awaiting_confirm, candidate_id=state.candidate_id,
                candidate_source=state.candidate_source)
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.p['map_frame']
            self.state_pub.publish(message)
            self.last_published, self.next_publish = state, now + self.p['publish_period']


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ElevatorMonitor()
        executor = spin_executor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
