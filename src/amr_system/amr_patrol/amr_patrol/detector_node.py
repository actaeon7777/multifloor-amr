"""Patrol-only motion detection; the always-on event subscriber owns spool writes."""
from datetime import datetime
import json
import math
from pathlib import Path
import time
from uuid import uuid4

from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from amr_stack.floor_config import load_floor_config
from amr_patrol.detection import Grid, MotionDetector, Settings, apply_update
from amr_patrol.routes import default_config_path, load_patrol_config
from amr_patrol.spool import EventSpool


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MotionDetectorNode(Node):
    def __init__(self, building=None, **kwargs):
        super().__init__('motion_detector', **kwargs)
        defaults = dict(vars(Settings()), config=str(default_config_path()),
                        detect_period=.5, camera_max_age_s=3.0, status_period=1.0,
                        image_keep_days=7.0, spool_max_files=2000,
                        spool_dir=str(Path.home() / 'amr_data/events/queue'),
                        image_dir=str(Path.home() / 'amr_data/security'))
        self.p = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        settings = Settings(**{key: self.p[key] for key in vars(Settings())})
        for key in ('detect_period', 'camera_max_age_s', 'status_period'):
            value = self.p[key]
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be positive and finite')
        for key in ('config', 'spool_dir', 'image_dir'):
            if not isinstance(self.p[key], str) or not self.p[key].strip():
                raise ValueError(f'{key} must be a non-empty path')
        config = load_patrol_config(self.p['config'], building or load_floor_config())
        self.detector = MotionDetector(settings, config.exclusions)
        self.spool = EventSpool(self.p['spool_dir'], self.p['image_dir'],
                                self.p['spool_max_files'], self.p['image_keep_days'],
                                self.get_logger().warning)
        self.active, self.floor = False, None
        self.sensor_subscriptions = []
        self.costmap = None
        self.coverage = None
        self.version, self.processed_version = 0, 0
        self.camera, self.camera_received = None, -math.inf
        self.previous_pose = None
        self.detections, self.last_detection_stamp = 0, None
        self.retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf = self.tf_listener = None
        self.event_pub = self.create_publisher(String, '/robot/event', 10)
        self.status_pub = self.create_publisher(String, '/patrol/detector_status', self.retained)
        self.create_subscription(String, '/robot/event', self.on_event, 10)
        self.create_subscription(String, '/patrol/status', self.on_status, self.retained)
        self.create_timer(self.p['detect_period'], self.detect)
        self.create_timer(self.p['status_period'], self.status_tick)
        self.create_timer(60.0, self.maintain_spool)
        self.maintain_spool()
        self.publish_status()

    def ros_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def publish(publisher, value):
        publisher.publish(String(data=json.dumps(value, ensure_ascii=False, allow_nan=False)))

    def on_event(self, message):
        try:
            # Includes our own publications exactly once; detect() never writes JSON.
            self.spool.write(json.loads(message.data))
        except (OSError, ValueError, TypeError) as error:
            self.get_logger().error(f'event spool failed: {error}')

    def maintain_spool(self):
        try:
            self.spool.enforce_bound()
            self.spool.prune_images()
        except OSError as error:
            self.get_logger().error(f'event spool maintenance failed: {error}')

    def on_status(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                return
            floor = status.get('floor')
            active = (status.get('active') is True and status.get('phase') in ('NAV', 'DWELL', 'RETURN')
                      and isinstance(floor, str) and bool(floor.strip()))
        except (ValueError, TypeError):
            return
        if active == self.active and floor == self.floor:
            return
        # Resubscribe to the transient-local map on every start/floor change.
        for subscription in self.sensor_subscriptions:
            self.destroy_subscription(subscription)
        self.sensor_subscriptions.clear()
        self.stop_tf()
        self.active, self.floor = active, floor
        self.costmap = self.detector.static = None
        self.coverage = None
        self.detector.reset(stamp=self.ros_now())
        self.previous_pose = None
        self.camera, self.camera_received = None, -math.inf
        self.version = self.processed_version = 0
        if active:
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self, spin_thread=False)
            self.detections, self.last_detection_stamp = 0, None
            camera_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.sensor_subscriptions = [
                self.create_subscription(OccupancyGrid, '/map', self.on_map, self.retained),
                self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                         self.on_costmap, self.retained),
                self.create_subscription(OccupancyGridUpdate, '/global_costmap/costmap_updates',
                                         self.on_update, 10),
                self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed',
                                         self.on_camera, camera_qos),
            ]
        self.publish_status()

    def stop_tf(self):
        if self.tf_listener is not None:
            self.tf_listener.unregister()
            self.tf_listener = None
        self.tf = None

    def destroy_node(self):
        self.stop_tf()
        return super().destroy_node()

    @staticmethod
    def grid(message):
        if message.header.frame_id != 'map':
            raise ValueError('expected grid in map frame')
        info = message.info
        data = np.asarray(message.data, dtype=np.int8).reshape(info.height, info.width)
        origin = info.origin
        return Grid(data, info.resolution,
                    (origin.position.x, origin.position.y, yaw_of(origin.orientation)))

    def on_map(self, message):
        if not self.active:
            return
        try:
            grid = self.grid(message)
            old = self.detector.static
            if old is not None and old.geometry == grid.geometry and np.array_equal(old.data, grid.data):
                return
            self.detector.set_static(grid)
            self.previous_pose = None
        except ValueError as error:
            self.get_logger().warning(f'invalid static map: {error}')

    def on_costmap(self, message):
        if not self.active:
            return
        try:
            grid = self.grid(message)
            # A retained full grid may date from Nav2 startup. Only subsequent
            # update rectangles establish fresh coverage, even for equal geometry.
            self.detector.clear_tracks()
            self.previous_pose = None
            self.costmap = grid
            self.coverage = np.zeros(grid.data.shape, dtype=bool)
            self.version += 1
        except ValueError as error:
            self.get_logger().warning(f'invalid costmap: {error}')

    def on_update(self, message):
        if not self.active or self.costmap is None or message.header.frame_id not in ('', 'map'):
            return
        try:
            apply_update(self.costmap, message.x, message.y, message.width, message.height, message.data)
            self.coverage[message.y:message.y + message.height,
                          message.x:message.x + message.width] = True
            self.version += 1
        except ValueError as error:
            self.get_logger().warning(str(error))

    def on_camera(self, message):
        if self.active and ('jpeg' in message.format.lower() or 'jpg' in message.format.lower()):
            self.camera = message
            self.camera_received = time.monotonic()

    def camera_fresh(self):
        return (self.camera is not None and bool(self.camera.data)
                and time.monotonic() - self.camera_received <= self.p['camera_max_age_s'])

    def detect(self):
        if (not self.active or self.detector.static is None or self.costmap is None
                or self.version == self.processed_version):
            return
        try:
            transform = self.tf.lookup_transform('map', 'base_footprint', Time()).transform
        except TransformException:
            return
        pose = (transform.translation.x, transform.translation.y, yaw_of(transform.rotation))
        now = self.ros_now()
        yaw_rate = 0.0
        if self.previous_pose is not None:
            stamp, previous_yaw = self.previous_pose
            if now > stamp:
                delta = math.atan2(math.sin(pose[2] - previous_yaw), math.cos(pose[2] - previous_yaw))
                yaw_rate = delta / (now - stamp)
            else:
                self.detector.reset(stamp=now)
        self.previous_pose = (now, pose[2])
        self.processed_version = self.version
        for detection in self.detector.process(self.costmap, pose, self.floor, now, yaw_rate,
                                               coverage=self.coverage):
            event_id = uuid4().hex
            wall_time = time.time()
            image_path = None
            if self.camera_fresh():
                name = datetime.fromtimestamp(wall_time).strftime('%Y%m%d-%H%M%S') + f'-{event_id}.jpg'
                path = self.spool.image_dir / name
                try:
                    path.write_bytes(bytes(self.camera.data))
                    image_path = str(path)
                except OSError as error:
                    try:
                        path.unlink(missing_ok=True)
                    except OSError:
                        pass
                    self.get_logger().error(f'snapshot failed: {error}')
            self.publish(self.event_pub, dict(
                id=event_id, kind='security.motion', severity='alert',
                text=f'{self.floor} ({detection.x:.1f}, {detection.y:.1f})에서 지도에 없는 움직임 감지',
                floor=self.floor, room='', pose=[detection.x, detection.y, 0],
                image_path=image_path, stamp=now, wall_time=wall_time,
                extra=dict(diameter_m=detection.diameter_m, displacement_m=detection.displacement_m,
                           cells=detection.cells, track_id=detection.track_id, robot_pose=list(pose))))
            self.detections += 1
            self.last_detection_stamp = now

    def status_tick(self):
        if self.active:
            self.publish_status()

    def publish_status(self):
        self.publish(self.status_pub, dict(active=self.active, tracks=len(self.detector.tracks),
                     detections=self.detections, last_detection_stamp=self.last_detection_stamp,
                     camera_fresh=self.camera_fresh()))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotionDetectorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
