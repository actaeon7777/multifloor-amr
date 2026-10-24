"""Run autonomous commands unless LB takes over for manual driving."""

from math import isfinite
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from amr_control.approve_gesture import ApproveGesture
from amr_control.control_gate import ControlGate
from amr_control.time_guard import HealthLease


class ControlMux(Node):
    def __init__(self, **kwargs):
        super().__init__('control_mux', **kwargs)
        defaults = {
            'deadman_button': 6, 'required_axes': [1, 2],
            'joy_timeout': 0.50, 'command_timeout': 0.50, 'release_debounce': 0.10,
            'max_linear_speed': 0.20, 'max_angular_speed': 0.40,
            'wheel_diameter': 0.172, 'track_width': 0.529, 'max_wheel_rev_s': 1.8,
            'auto_timeout': 0.50, 'manual_holdoff': 0.50, 'mode_stop_duration': 0.10,
        }
        parameters = {key: self.declare_parameter(key, value).value
                      for key, value in defaults.items()}
        self.gate = ControlGate(**parameters)
        self.require_time_health = self.declare_parameter('require_time_health', False).value
        self.time_health = HealthLease()
        self.create_subscription(String, '/system/time_health', self.on_time_health,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # cmd_vel_to_can enforces the E-stop interlock; this only drops cached motion.
        self.require_estop = self.declare_parameter('require_estop', False).value
        self.estop = HealthLease()
        self.estop_blocked = self.require_estop
        self.create_subscription(String, '/estop/status', self.on_estop,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Operator at the robot: hold approve_button (LB up, sticks centred) to
        # approve a restart after an E-stop. -1 disables the joystick approval.
        approve_button = self.declare_parameter('approve_button', -1).value
        approve_hold = self.declare_parameter('approve_hold', 2.0).value
        self.approve_gesture = None
        if approve_button >= 0:
            self.approve_gesture = ApproveGesture(
                approve_button, parameters['deadman_button'], approve_hold,
                parameters['required_axes'])
            self.approve_client = self.create_client(Trigger, '/estop/approve_resume')
            self.approve_future = None
        rate = self.declare_parameter('publish_rate', 20.0).value
        if not isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate must be finite and positive')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.status_publisher = self.create_publisher(
            String, '~/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.last_status = None
        self.mode_publisher = self.create_publisher(
            String, '~/mode', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.last_mode = None
        self.joy_subscription = self.create_subscription(
            Joy, '/joy', self.on_joy,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.command_subscription = self.create_subscription(
            Twist, '/cmd_vel_manual', self.on_command, 1,
        )
        self.auto_subscription = self.create_subscription(
            Twist, '/cmd_vel_auto', self.on_auto_command, 1,
        )
        # Timeouts must still run if ROS simulation time stops or jumps.
        self.timer = self.create_timer(
            1.0 / rate, self.publish_command, clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.get_logger().info(
            'AUTO by default, with or without a joystick. Pressing LB takes over (MANUAL) and '
            'stops AUTO; hold LB to drive. AUTO resumes manual_holdoff after LB is released.'
        )
        self.publish_status()

    def on_joy(self, message):
        now = monotonic()
        self.gate.receive_joy(message.axes, message.buttons, now)
        if self.approve_gesture is not None and self.approve_gesture.receive_joy(
                message.axes, message.buttons, now, self.approval_required(now)):
            self.request_approval()
        self.publish_command()

    def approval_required(self, now):
        fresh = 0 <= now - self.estop.received < self.estop.timeout
        return self.require_estop and fresh and self.estop.status.get('approval_required') is True

    def request_approval(self):
        if self.approve_future is not None and not self.approve_future.done():
            return
        if not self.approve_client.service_is_ready():
            self.get_logger().error('Joystick approval: /estop/approve_resume is not available')
            return
        self.get_logger().warning('Joystick approval held; requesting E-stop restart approval')
        self.approve_future = self.approve_client.call_async(Trigger.Request())
        self.approve_future.add_done_callback(self.on_approval)

    def on_approval(self, future):
        try:
            response = future.result()
        except Exception as error:  # Service died mid-call.
            self.get_logger().error(f'Joystick approval failed: {error}')
            return
        if response.success:
            self.get_logger().warning(f'Joystick approval accepted: {response.message}')
        else:
            self.get_logger().error(f'Joystick approval rejected: {response.message}')

    def on_command(self, message):
        self.gate.receive_command((
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        ), monotonic())
        # Track manual input immediately; the timer also refreshes output and
        # checks expiry/debounce even when no callbacks arrive.
        self.publish_command()

    def on_auto_command(self, message):
        self.gate.receive_auto_command((
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        ), monotonic())
        self.publish_command()

    def publish_zero(self):
        self.publisher.publish(Twist())

    def on_time_health(self, message):
        self.time_health.receive(message.data)
        self.publish_command()

    def on_estop(self, message):
        self.estop.receive(message.data)
        self.publish_command()

    def publish_command(self):
        linear, angular = self.gate.output(monotonic())
        if self.require_time_health and self.gate.auto_enabled and not self.time_health.healthy():
            linear = angular = 0.
            self.gate.auto_command = self.gate.auto_time = None
            self.gate.auto_reason = 'time_data_invalid'
        self.estop_blocked = self.require_estop and not self.estop.healthy()
        if self.estop_blocked:
            # After an E-stop, motion resumes only from commands sent after approval:
            # a stick held through it is not one, so LB must be pressed anew once READY.
            linear = angular = 0.
            self.gate.auto_command = self.gate.auto_time = None
            self.gate.manual.stop('estop_not_ready')
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.publisher.publish(message)
        self.publish_status()

    def publish_status(self):
        mode = self.gate.mode
        if mode != self.last_mode:
            self.last_mode = mode
            self.mode_publisher.publish(String(data=mode))
            self.get_logger().info(f'Control mode: {mode}')
        reason = 'estop_not_ready' if self.estop_blocked else self.gate.reason
        if reason == self.last_status:
            return
        self.last_status = reason
        self.status_publisher.publish(String(data=reason))
        if reason in ('joy_timeout', 'command_timeout', 'invalid_joy', 'invalid_command',
                      'auto_command_timeout', 'auto_invalid_command', 'estop_not_ready'):
            self.get_logger().warning(f'Control output paused: {reason}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ControlMux()
        executor = spin_executor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_zero()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
