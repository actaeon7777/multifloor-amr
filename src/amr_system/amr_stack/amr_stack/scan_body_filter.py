"""Hide S2 returns that land on the robot's own frame before anything reads /scan.

The v80 2020 uprights cross the scan plane at 0.27-0.28 m from the sensor.
Returns inside the configured base_link boxes become NaN (no measurement), so
AMCL, costmaps and the elevator clearance checks neither mark nor clear on them.
A beam that only grazes an upright edge returns a mixed range 0.36-0.50 m out,
outside the footprint (real vehicle, 2026-10-23). Each upright's shadow sector,
widened by shadow_margin_deg, is therefore also cleared up to shadow_range.
Nothing real is visible behind an upright, so only the margin costs coverage.
Scans are dropped, never passed through unfiltered, until the laser TF is known.
"""

import array
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


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


def body_boxes(values, margin):
    """[x_min, x_max, y_min, y_max] rows in base_link, grown by the margin."""
    boxes = np.asarray(values, dtype=float)
    if boxes.size == 0 or boxes.size % 4:
        raise ValueError('body_boxes needs x_min, x_max, y_min, y_max per box')
    boxes = boxes.reshape(-1, 4)
    if (not np.isfinite(boxes).all() or (boxes[:, 0] >= boxes[:, 1]).any()
            or (boxes[:, 2] >= boxes[:, 3]).any()):
        raise ValueError('body_boxes must be finite with min < max')
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('margin must be finite and non-negative')
    return boxes + np.array([-margin, margin, -margin, margin])


def shadow_sectors(boxes, pose, margin):
    """(centre, half width) bearing in base_link of each box seen from the laser.

    Use the boxes without their range margin: growing a box towards the
    sensor would widen its sector far beyond the part it models.
    """
    sectors = []
    for x_min, x_max, y_min, y_max in boxes:
        corners = np.arctan2(np.array([y_min, y_max, y_min, y_max]) - pose[1],
                             np.array([x_min, x_min, x_max, x_max]) - pose[0])
        centre = math.atan2(np.sin(corners).sum(), np.cos(corners).sum())
        offsets = np.angle(np.exp(1j * (corners - centre)))
        if offsets.max() - offsets.min() >= math.pi:
            raise ValueError('body box surrounds the laser')
        middle = (offsets.max() + offsets.min()) / 2
        sectors.append((centre + middle, (offsets.max() - offsets.min()) / 2 + margin))
    return np.array(sectors).reshape(-1, 2)


def shadow_mask(ranges, angle_min, angle_increment, pose, sectors, max_range):
    """True for finite returns within max_range inside any shadow sector."""
    r = np.asarray(ranges, dtype=float)
    finite = np.isfinite(r)
    angle = angle_min + angle_increment * np.arange(r.size) + pose[2]
    offset = np.abs(np.angle(np.exp(1j * (angle[:, None] - sectors[:, 0]))))
    return finite & (r <= max_range) & (offset <= sectors[:, 1]).any(axis=1)


def hidden_mask(cache, key, ranges, angle_min, angle_increment, pose, boxes, sectors, max_range):
    """body_mask | shadow_mask with the per-beam geometry cached under ``key``.

    Beam angles, their cos/sin and the shadow-sector membership depend only
    on the scan layout (frame, angle_min, angle_increment, beam count);
    recomputing them for 3240 beams at 10 Hz was most of this node's work.
    """
    r = np.asarray(ranges, dtype=float)
    cached = cache.get(key)
    if cached is None:
        angle = angle_min + angle_increment * np.arange(r.size) + pose[2]
        offset = np.abs(np.angle(np.exp(1j * (angle[:, None] - sectors[:, 0]))))
        cached = cache[key] = (np.cos(angle), np.sin(angle), (offset <= sectors[:, 1]).any(axis=1))
        if len(cache) > 4:
            cache.pop(next(iter(cache)))
    cosine, sine, in_shadow = cached
    finite = np.isfinite(r)
    distance = np.where(finite, r, 0.0)
    x = (pose[0] + distance * cosine)[:, None]
    y = (pose[1] + distance * sine)[:, None]
    inside = ((x >= boxes[:, 0]) & (x <= boxes[:, 1])
              & (y >= boxes[:, 2]) & (y <= boxes[:, 3])).any(axis=1)
    return finite & (inside | ((r <= max_range) & in_shadow))


def laser_pose(transform):
    """Planar (x, y, yaw) of a level laser from a base_link <- laser transform."""
    t, q = transform.translation, transform.rotation
    # Tilt makes the planar projection wrong; the S2 is mounted level.
    if abs(2 * (q.w * q.x + q.y * q.z)) > 0.02 or abs(2 * (q.w * q.y - q.z * q.x)) > 0.02:
        raise ValueError('laser frame is not level with base_link')
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return t.x, t.y, yaw


def body_mask(ranges, angle_min, angle_increment, pose, boxes):
    """True for finite returns whose end point lies inside any body box."""
    r = np.asarray(ranges, dtype=float)
    finite = np.isfinite(r)
    angle = angle_min + angle_increment * np.arange(r.size) + pose[2]
    distance = np.where(finite, r, 0.0)
    x = (pose[0] + distance * np.cos(angle))[:, None]
    y = (pose[1] + distance * np.sin(angle))[:, None]
    inside = ((x >= boxes[:, 0]) & (x <= boxes[:, 1])
              & (y >= boxes[:, 2]) & (y <= boxes[:, 3])).any(axis=1)
    return finite & inside


class ScanBodyFilter(Node):

    def __init__(self):
        super().__init__('scan_body_filter')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        values = self.declare_parameter('body_boxes', [0.0]).value
        self.parts = body_boxes(values, 0.0)
        self.boxes = body_boxes(values, self.declare_parameter('margin', 0.03).value)
        self.shadow_margin = math.radians(self.declare_parameter('shadow_margin_deg', 3.0).value)
        self.shadow_range = self.declare_parameter('shadow_range', 0.7).value
        if not (math.isfinite(self.shadow_margin) and 0 <= self.shadow_margin < math.pi / 4
                and math.isfinite(self.shadow_range) and self.shadow_range >= 0):
            raise ValueError('shadow_margin_deg must be in [0, 45) and shadow_range >= 0')
        self.poses = {}
        self.beam_cache = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        # Same reliable depth-10 profile as sllidar_node, so both best-effort
        # and reliable consumers of /scan keep working.
        self.publisher = self.create_publisher(LaserScan, 'scan', QoSProfile(depth=10))
        self.create_subscription(LaserScan, 'scan_raw', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        frame = msg.header.frame_id
        entry = self.poses.get(frame)
        if entry is None:
            try:
                transform = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
            except TransformException as error:
                self.get_logger().warn(f'Dropping scan until {self.base_frame} <- {frame} is known: {error}',
                                       throttle_duration_sec=2.0)
                return
            pose = laser_pose(transform.transform)
            entry = self.poses[frame] = (pose, shadow_sectors(self.parts, pose, self.shadow_margin))
            self.get_logger().info(f'{frame} at x={pose[0]:.3f} y={pose[1]:.3f} yaw={pose[2]:.3f}; '
                                   f'{len(self.boxes)} body boxes, shadows to {self.shadow_range:.2f} m')
            # The laser sits on a fixed joint and its pose is cached for good,
            # so stop deserializing /tf (EKF 30 Hz + AMCL) into an unread buffer.
            # The static transform stays in the buffer for a later frame lookup.
            if self.tf_listener is not None:
                self.tf_listener.unregister()
                self.tf_listener = None
        pose, sectors = entry
        ranges = np.array(msg.ranges, dtype=np.float32)
        hidden = self.hidden_mask(frame, msg.angle_min, msg.angle_increment, ranges, pose, sectors)
        ranges[hidden] = np.nan
        msg.ranges = array.array('f', ranges.tobytes())
        self.publisher.publish(msg)

    def hidden_mask(self, frame, angle_min, angle_increment, ranges, pose, sectors):
        return hidden_mask(self.beam_cache, (frame, angle_min, angle_increment, ranges.size),
                           ranges, angle_min, angle_increment, pose, self.boxes, sectors, self.shadow_range)


def main():
    rclpy.init()
    node = ScanBodyFilter()
    executor = spin_executor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
