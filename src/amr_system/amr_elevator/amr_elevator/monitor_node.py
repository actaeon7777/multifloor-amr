"""elevator_monitor: LiDAR door state + current floor / inside-car estimation.

Shared by the real vehicle and the simulation; it only reads the real AMR
interfaces (/scan, /odometry/filtered, TF, /localization/status) and the
elevator topics (/elevator/beacon, /elevator/command).

Inside the car AMCL cannot be trusted, so the robot position is the last pose
verified by localization outside the car carried forward by odometry
(``anchor`` = map->odom at that time).  When the floor changes the anchor is
moved with the car: the pose relative to the car is kept on the new floor map.
"""

import json
import math

from amr_elevator_interfaces.msg import (
    ElevatorBeacon, ElevatorCommand, ElevatorDoorState, FloorEvent, FloorState,
)
from amr_elevator_interfaces.srv import SetCurrentFloor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from amr_elevator.car_matcher import car_wall_segments, fit_car
from amr_elevator.door_detector import NAMES as DOOR_NAMES, DoorDetector, enclosure_ratio
from amr_elevator.floor_state import ZONE_IN_ELEVATOR, FloorStateMachine
from amr_elevator.geometry import compose, depth_beyond, inverse, point_in_polygon


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
            'exit_margin': .37, 'enclosure_near': 3., 'enclosure_ratio': .75,
            'stationary_speed': .05, 'publish_period': 1.,
            'car_wall_correction': True, 'car_correction_gain': .5,
        }.items()}
        self.p = p
        self.building = building or load_floor_config(p['floors_config'] or None)
        self.elevator_id = p['elevator_id']
        self.machine = FloorStateMachine(self.building, p['initial_floor'] or None)
        self.detector = DoorDetector()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.anchor = None          # map->odom on the map of anchor_floor
        self.anchor_floor = None
        self.odom_pose = None
        self.speed = (0., 0.)
        self.laser_offset = None    # base->laser
        self.enclosed = False
        self.ready = False
        self.commands = {}          # sequence -> ElevatorCommand
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
        self.create_subscription(ElevatorBeacon, 'elevator/beacon', self.on_beacon, 50)
        self.create_subscription(ElevatorCommand, 'elevator/command', self.on_command, 20)
        self.create_service(SetCurrentFloor, 'floor/set_current', self.on_set_floor)
        self.create_timer(.1, self.tick)
        self.get_logger().info(f'Elevator monitor: {sorted(self.building.floors)}, '
                               f'start {self.machine.state.floor} (unconfirmed)')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # Geometry -------------------------------------------------------------------

    def current_floor(self):
        s = self.machine.state
        return s.floor or s.transit_from or self.machine.building.default_floor

    def geometry(self, floor=None):
        return self.building.elevator_geometry(floor or self.current_floor(), self.elevator_id)

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
        if self.anchor is None or not floor or floor == self.anchor_floor:
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

    def on_command(self, message):
        self.commands[message.sequence] = message
        if len(self.commands) > 64:
            self.commands.pop(next(iter(self.commands)))

    def on_beacon(self, message):
        now = self.now()
        stamp = Time.from_msg(message.header.stamp).nanoseconds * 1e-9
        verdict = self.machine.on_beacon(message.event, message.location, message.floor_code,
                                         message.sequence, stamp, message.rssi_dbm, now)
        if (verdict == 'ok' and message.event == ElevatorBeacon.EVENT_BUTTON_ACK
                and message.location == ElevatorBeacon.LOCATION_CAR):
            command = self.commands.get(message.ack_sequence)
            if command is not None and command.command == ElevatorCommand.CMD_CAR_FLOOR:
                floor = self.building.floor_by_beacon_code(command.floor_code)
                if floor is not None:
                    self.machine.on_car_target(floor.name)

    def on_scan(self, message):
        if self.laser_offset is None:
            try:
                transform = self.tf.lookup_transform(self.p['base_frame'], message.header.frame_id,
                                                     Time(), Duration(seconds=0.))
            except TransformException:
                return
            self.laser_offset = pose_of_transform(transform)
        ranges = list(message.ranges)
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
        points = []
        for index, value in enumerate(ranges):
            if math.isfinite(value) and message.range_min <= value <= 4.:
                angle = laser[2] + message.angle_min + index * message.angle_increment
                points.append((laser[0] + value * math.cos(angle), laser[1] + value * math.sin(angle)))
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
            self.machine.on_position(inside, past_hall, self.enclosed, now)
        state = self.machine.tick(now)
        self.follow_car()
        for record in self.machine.drain_events():
            message = FloorEvent(event=record.event, from_floor=record.from_floor,
                                 to_floor=record.to_floor, detail=record.detail)
            message.header.stamp = self.get_clock().now().to_msg()
            self.event_pub.publish(message)
            text = f'{record.event} {record.from_floor}->{record.to_floor} {record.detail}'.strip()
            # rclpy forbids changing the severity of one call site, so keep two call sites.
            if record.event in ('RULE_VIOLATION', 'FLOOR_CONFLICT', 'ARRIVAL_UNCONFIRMED'):
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)
        if state != self.last_published or now >= self.next_publish:
            message = FloorState(
                zone=state.zone, floor=state.floor, transit_from=state.transit_from,
                transit_to=state.transit_to, confidence=state.confidence, last_event=state.last_event,
                floor_conflict=state.floor_conflict, duplicate_count=state.duplicate_count,
                out_of_order_count=state.out_of_order_count, stale_count=state.stale_count)
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.p['map_frame']
            self.state_pub.publish(message)
            self.last_published, self.next_publish = state, now + self.p['publish_period']


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ElevatorMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
