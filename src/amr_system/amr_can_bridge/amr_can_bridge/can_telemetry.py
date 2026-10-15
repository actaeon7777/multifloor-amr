"""Receive-only SocketCAN -> measured wheel odometry, joint states and raw IMU."""

from math import cos, isfinite, sin
import signal
from threading import Event
from time import time, monotonic

from amr_vehicle_interfaces.msg import MotorStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState

from amr_can_bridge.can_sender import require_interface_up
from amr_control.time_guard import ClockMonitor
from amr_can_bridge.telemetry import TELEMETRY_IDS, ImuPairer, WheelOdometry, decode_frame, positive


class CanTelemetry(Node):
    def __init__(self, bus_factory=None, **kwargs):
        super().__init__('can_telemetry', **kwargs)
        self.bus = None
        defaults = {
            'channel': 'can0', 'wheel_diameter': .172, 'track_width': .499,
            'left_direction': -1., 'right_direction': 1.,
            'encoder_turns_per_wheel_turn': 1., 'max_valid_wheel_rev_s': 5.,
            'wheel_pair_slop': .02, 'wheel_timeout': .25,
            'imu_pair_slop': .05, 'imu_timeout': .5,
            'accel_range_g': 2, 'gyro_range_dps': 125, 'gyro_bias': [0., 0., 0.],
            'odom_frame': 'odom', 'base_frame': 'base_footprint', 'imu_frame': 'imu_link',
            'max_frame_age': .25,
            # Initial conservative estimates, not a completed sensor calibration.
            'linear_velocity_variance': .0025, 'angular_velocity_variance': .01,
            'gyro_variance': .0009, 'accel_variance': .01,
        }
        try:
            self.config = c = {k: self.declare_parameter(k, v).value for k,v in defaults.items()}
            if self.get_parameter('use_sim_time').value:
                raise ValueError('Live SocketCAN timestamps require use_sim_time=false')
            for key in ('max_frame_age', 'linear_velocity_variance', 'angular_velocity_variance',
                        'gyro_variance', 'accel_variance'):
                positive(key, c[key])
            for key in ('channel', 'odom_frame', 'base_frame', 'imu_frame'):
                if not c[key] or c[key].strip() != c[key] or c[key].startswith('/'):
                    raise ValueError(f'{key} must be nonempty and have no leading slash or whitespace')
            if len({c['odom_frame'], c['base_frame'], c['imu_frame']}) != 3:
                raise ValueError('odom, base and IMU frames must be distinct')
            self.wheels = WheelOdometry(
                c['wheel_diameter'], c['track_width'], c['left_direction'], c['right_direction'],
                c['encoder_turns_per_wheel_turn'], c['wheel_pair_slop'], c['wheel_timeout'],
                c['max_valid_wheel_rev_s'])
            self.imu = ImuPairer(c['accel_range_g'], c['gyro_range_dps'], c['imu_pair_slop'],
                                 c['imu_timeout'], c['gyro_bias'])
            self.odom_pub = self.create_publisher(Odometry, 'wheel/odometry', 10)
            self.joints_pub = self.create_publisher(JointState, 'joint_states', 10)
            self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', qos_profile_sensor_data)
            self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
            self.motor_pubs = {side: self.create_publisher(MotorStatus, f'motor/{side}/telemetry', 10)
                               for side in ('left', 'right')}
            self.received = self.invalid = self.stale = self.odom_count = self.imu_count = 0
            self.last_wheel = self.last_imu = None
            self.clock_monitor = ClockMonitor()
            self.clock_resets = 0
            self.rx_error = ''
            if bus_factory is None:
                from can.interfaces.socketcan import SocketcanBus
                require_interface_up(c['channel'])
                bus_factory = SocketcanBus
            self.bus = bus_factory(
                channel=c['channel'], receive_own_messages=False, fd=False,
                can_filters=[{'can_id': cid, 'can_mask': 0x7ff, 'extended': False}
                             for cid in TELEMETRY_IDS])
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.poll_timer = self.create_timer(.005, self.poll, clock=steady)
            self.health_timer = self.create_timer(1., self.publish_health, clock=steady)
            self.add_on_set_parameters_callback(lambda _: SetParametersResult(
                successful=False, reason='Restart telemetry to apply parameter changes'))
        except Exception:
            self.close()
            self.destroy_node()
            raise
        self.get_logger().info(
            f'Receive-only {c["channel"]}: node 0=right (+), node 1=left (-) by default; '
            f'IMU ranges ±{c["accel_range_g"]}g / ±{c["gyro_range_dps"]}dps. '
            'No CAN requests, motor commands or odom TF are sent.')

    @staticmethod
    def stamp(seconds):
        return Time(nanoseconds=round(seconds * 1e9)).to_msg()

    def poll(self):
        try:
            if self.clock_monitor.observe(time(), monotonic()):
                self.wheels.reset_time()
                self.imu.reset_time()
                self.last_wheel = self.last_imu = None
                self.clock_resets += 1
                self.get_logger().error('System clock changed; sensor time history rebased, pose retained')
            for _ in range(500):  # Bound executor work even during bus flooding.
                message = self.bus.recv(timeout=0.)
                if message is None:
                    break
                self.process_message(message, time())
            self.rx_error = ''
        except Exception as error:
            self.rx_error = str(error)

    def process_message(self, message, now):
        try:
            decoded = decode_frame(message.arbitration_id, message.data,
                                   extended=message.is_extended_id, remote=message.is_remote_frame,
                                   error=message.is_error_frame, fd=message.is_fd)
            if decoded is None:
                return
            self.received += 1
            stamp = message.timestamp  # Kernel reception time, not callback processing time.
            if not isfinite(stamp) or stamp <= 0 or not -.02 <= now - stamp <= self.config['max_frame_age']:
                self.stale += 1
                return
            kind, values = decoded
            if kind in ('left', 'right'):
                sample = self.wheels.update(kind, *values, stamp)
                if sample is not None:
                    self.publish_wheel(sample)
            elif kind.endswith(('_power', '_heartbeat')):
                self.publish_motor(kind, values, stamp)
            else:
                sample = self.imu.update(kind, values, stamp)
                if sample is not None:
                    self.publish_imu(sample)
        except (ValueError, TypeError, OverflowError):
            self.invalid += 1

    def publish_motor(self, kind, values, stamp):
        side, source = kind.split('_', 1)
        msg = MotorStatus()
        msg.node_id = 1 if side == 'left' else 0
        meta = getattr(msg, source)
        meta.received = meta.valid = meta.fresh = meta.stamp_valid = True
        meta.source_stamp = self.stamp(stamp)
        meta.timeout_sec = 2.5 if source == 'power' else .75
        if source == 'power':
            msg.input_voltage_v, msg.bus_current_a = values
        else:
            msg.error_code, msg.axis_state = values
        self.motor_pubs[side].publish(msg)

    def publish_wheel(self, sample):
        c = self.config
        msg = Odometry()
        msg.header.stamp = self.stamp(sample.stamp)
        msg.header.frame_id, msg.child_frame_id = c['odom_frame'], c['base_frame']
        msg.pose.pose.position.x, msg.pose.pose.position.y = sample.x, sample.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = sin(sample.yaw/2), cos(sample.yaw/2)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = sample.v, sample.w
        # Conservative diagonal growth; dropouts imply unobserved displacement.
        position_var = .01 + .02*self.wheels.travel + self.wheels.rebases
        yaw_var = .01 + .02*(self.wheels.travel+self.wheels.rotation) + self.wheels.rebases
        for index, variance in zip((0,7,14,21,28,35),
                                   (position_var, position_var, 1e6, 1e6, 1e6, yaw_var)):
            msg.pose.covariance[index] = float(variance)
        for index, variance in zip((0,7,14,21,28,35),
                                   (c['linear_velocity_variance'], .01, 1e6, 1e6, 1e6,
                                    c['angular_velocity_variance'])):
            msg.twist.covariance[index] = float(variance)
        self.odom_pub.publish(msg)
        joints = JointState()
        joints.header.stamp = msg.header.stamp
        joints.header.frame_id = c['base_frame']
        joints.name = ['left_drive_wheel_joint', 'right_drive_wheel_joint']
        joints.position, joints.velocity = list(sample.positions), list(sample.velocities)
        self.joints_pub.publish(joints)
        self.last_wheel, self.odom_count = sample.stamp, self.odom_count + 1

    def publish_imu(self, sample):
        msg = Imu()
        msg.header.stamp, msg.header.frame_id = self.stamp(sample.stamp), self.config['imu_frame']
        msg.orientation.w = 1.
        msg.orientation_covariance[0] = -1.  # No attitude measurement in the CAN protocol.
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = sample.acceleration
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = sample.angular_velocity
        for index in (0,4,8):
            msg.linear_acceleration_covariance[index] = self.config['accel_variance']
            msg.angular_velocity_covariance[index] = self.config['gyro_variance']
        self.imu_pub.publish(msg)
        self.last_imu, self.imu_count = sample.stamp, self.imu_count + 1

    def publish_health(self):
        now = time()
        problems = []
        for name, stamp, limit in [('wheel', self.last_wheel, self.config['wheel_timeout']),
                                    ('imu', self.last_imu, self.config['imu_timeout'])]:
            if stamp is None or not 0 <= now-stamp <= limit:
                problems.append(f'{name} missing/stale')
        if self.wheels.rebases:
            problems.append('wheel pose continuity lost; inspect counters/restart to re-zero')
        status = DiagnosticStatus()
        status.name = 'amr/can_telemetry'
        status.hardware_id = self.config['channel']
        status.level = DiagnosticStatus.ERROR if self.rx_error else (DiagnosticStatus.WARN if problems else DiagnosticStatus.OK)
        status.message = self.rx_error or '; '.join(problems) or 'Wheel and IMU data fresh'
        counters = dict(received=self.received, invalid=self.invalid, stale_frames=self.stale,
                        odometry_published=self.odom_count, imu_published=self.imu_count,
                        wheel_rebases=self.wheels.rebases, wheel_dropped=self.wheels.dropped,
                        imu_dropped=self.imu.dropped,
                        clock_resets=self.clock_resets,
                        wheel_age_s=now-self.last_wheel if self.last_wheel else -1.,
                        imu_age_s=now-self.last_imu if self.last_imu else -1.)
        status.values = [KeyValue(key=k,value=str(v)) for k,v in counters.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diag_pub.publish(msg)

    def close(self):
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None


def main(args=None):
    stop = Event()
    previous = {}
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, lambda *_: stop.set())
        node = CanTelemetry()
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        rclpy.logging.get_logger('can_telemetry').error(str(error))
        raise SystemExit(1)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)
