"""Robot-side time validity gate. No motor IO; consumers inhibit AUTO on loss."""
import json
import math
import time
import uuid

import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
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
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from .time_guard import ClockMonitor


class TimeHealth(Node):
    def __init__(self, **kwargs):
        super().__init__('time_health', **kwargs)
        self.max_age = self.declare_parameter('max_data_age', .75).value
        self.future = self.declare_parameter('future_tolerance', .1).value
        self.recovery = self.declare_parameter('recovery_duration', 1.).value
        self.clock_monitor = ClockMonitor(self.declare_parameter('clock_jump_limit', .5).value)
        if any(not math.isfinite(v) or v <= 0 for v in (self.max_age, self.future, self.recovery)):
            raise ValueError('Time health thresholds must be positive and finite')
        self.session, self.sequence, self.epoch = str(uuid.uuid4()), 0, 0
        self.samples = {}
        self.speed = None
        self.good_since = None
        self.latched = False
        self.simulated = self.get_parameter('use_sim_time').value
        self.publisher = self.create_publisher(String, '/system/time_health',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for kind, typ, topic in [('scan', LaserScan, '/scan'), ('wheel', Odometry, '/wheel/odometry'),
                                 ('imu', Imu, '/imu/data_raw')]:
            self.create_subscription(typ, topic, lambda m, k=kind: self.observe(k, m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_service(Trigger, '/system/reset_time_fault', self.reset_fault)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def observe(self, kind, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec/1e9
        if stamp <= 0 or not -self.future <= self.now()-stamp <= self.max_age:
            return
        old = self.samples.get(kind)
        if old and stamp <= old[0]:
            return  # Delayed packets cannot refresh freshness or reset the clock.
        self.samples[kind] = (stamp, time.monotonic())
        return True

    def on_odom(self, message):
        if self.observe('odom', message):
            self.speed = (message.twist.twist.linear.x, message.twist.twist.angular.z)

    def on_tf(self, message):
        for transform in message.transforms:
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_footprint':
                self.observe('odom_tf', transform)

    def data_error(self, ros, steady):
        for name in ('scan', 'wheel', 'imu', 'odom', 'odom_tf'):
            stamp, received = self.samples.get(name, (0., -math.inf))
            if stamp <= 0 or steady-received > self.max_age:
                return f'{name}: missing or stale'
            if not -self.future <= ros-stamp <= self.max_age:
                return f'{name}: timestamp outside allowed range'
        return ''

    def reset_fault(self, _request, response):
        now = time.monotonic()
        stable = self.good_since is not None and now-self.good_since >= self.recovery
        stopped = (self.speed is not None and all(math.isfinite(v) for v in self.speed)
                   and abs(self.speed[0]) <= .02 and abs(self.speed[1]) <= .03)
        response.success = not self.data_error(self.now(), now) and stable and stopped
        if response.success:
            self.latched = False
            response.message = 'Time fault cleared while stationary. Revalidate localization before a new goal.'
        else:
            response.message = 'Fresh sensor/TF data, stable time and a stopped robot are required.'
        return response

    def tick(self):
        ros, steady = self.now(), time.monotonic()
        if self.clock_monitor.observe(ros, steady, self.simulated):
            self.epoch += 1
            self.samples.clear()
            self.speed = self.good_since = None
            self.latched = not self.simulated
            self.get_logger().error('Clock discontinuity: AUTO inhibited; fresh data required')
        error = self.data_error(ros, steady)
        if error:
            self.good_since = None
        elif self.good_since is None:
            self.good_since = steady
        stable = self.good_since is not None and steady-self.good_since >= self.recovery
        self.sequence += 1
        self.publisher.publish(String(data=json.dumps(dict(
            session=self.session, sequence=self.sequence, epoch=self.epoch, stamp=time.time(),
            healthy=not error and stable and not self.latched, latched=self.latched,
            detail='Clock changed; explicit reset and localization validation required' if self.latched
                   else error or ('Time data valid' if stable else 'Validating fresh time data')))))


def main(args=None):
    rclpy.init(args=args)
    node = TimeHealth()
    executor = spin_executor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
