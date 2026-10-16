"""Hide S2 returns that land on the robot's own frame before anything reads /scan.

The v80 2020 uprights cross the scan plane at 0.27-0.28 m from the sensor.
Returns inside the configured base_link boxes become NaN (no measurement), so
AMCL, costmaps and the elevator clearance checks neither mark nor clear on them.
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
        self.boxes = body_boxes(self.declare_parameter('body_boxes', [0.0]).value,
                                self.declare_parameter('margin', 0.03).value)
        self.poses = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Same reliable depth-10 profile as sllidar_node, so both best-effort
        # and reliable consumers of /scan keep working.
        self.publisher = self.create_publisher(LaserScan, 'scan', QoSProfile(depth=10))
        self.create_subscription(LaserScan, 'scan_raw', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        frame = msg.header.frame_id
        pose = self.poses.get(frame)
        if pose is None:
            try:
                transform = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
            except TransformException as error:
                self.get_logger().warn(f'Dropping scan until {self.base_frame} <- {frame} is known: {error}',
                                       throttle_duration_sec=2.0)
                return
            pose = self.poses[frame] = laser_pose(transform.transform)
            self.get_logger().info(f'{frame} at x={pose[0]:.3f} y={pose[1]:.3f} yaw={pose[2]:.3f}; '
                                   f'{len(self.boxes)} body boxes')
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[body_mask(ranges, msg.angle_min, msg.angle_increment, pose, self.boxes)] = np.nan
        msg.ranges = array.array('f', ranges.tobytes())
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ScanBodyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
