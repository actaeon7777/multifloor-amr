"""Translate /cmd_vel to the proven Jetson -> STM32 CAN velocity protocol."""

from math import isfinite
import signal
from threading import Event
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from amr_can_bridge.can_sender import CanInterfaceError, CanSender
from amr_can_bridge.drive_protocol import CommandWatchdog, make_payload


class CmdVelToCan(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_can')
        self.sender = None
        self.closed = False
        defaults = {
            'channel': 'can0', 'can_id': 0x201, 'dry_run': False,
            'tx_rate': 20.0, 'command_timeout': 0.50, 'send_timeout': 0.01,
            'max_linear_speed': 0.20, 'max_angular_speed': 0.40,
            'shutdown_stop_count': 5,
        }
        config = {name: self.declare_parameter(name, value).value
                  for name, value in defaults.items()}
        try:
            rate = config['tx_rate']
            if not isfinite(rate) or rate <= 0:
                raise ValueError('tx_rate must be finite and positive')
            self.period = 1.0 / rate
            if config['send_timeout'] >= self.period:
                raise ValueError('send_timeout must be shorter than the TX period')
            self.stop_count = config['shutdown_stop_count']
            if type(self.stop_count) is not int or not 1 <= self.stop_count <= 100:
                raise ValueError('shutdown_stop_count must be between 1 and 100')
            self.watchdog = CommandWatchdog(
                timeout=config['command_timeout'],
                max_linear_speed=config['max_linear_speed'],
                max_angular_speed=config['max_angular_speed'],
            )
            self.preview = self.create_publisher(String, '~/tx_preview', 1)
            self.subscription = self.create_subscription(Twist, '/cmd_vel', self.on_command, 1)
            self.transport_fault = False
            self.last_error_log = float('-inf')
            self.sender = CanSender(
                channel=config['channel'], can_id=config['can_id'], dry_run=config['dry_run'],
                send_timeout=config['send_timeout'],
            )
            self.timer = self.create_timer(
                self.period, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME),
            )
            self.add_on_set_parameters_callback(self.reject_runtime_changes)
        except Exception:
            if self.sender is not None:
                self.sender.close()
            self.destroy_node()
            raise
        mode = 'DRY_RUN: CAN socket is not opened' if self.sender.dry_run else 'LIVE CAN TX'
        self.get_logger().info(
            f'{mode}; {self.sender.channel} ID=0x{self.sender.can_id:03X}, DLC=8, '
            f'<ff> v[m/s], w[rad/s], {rate:g} Hz. Waiting for a fresh zero command.'
        )

    @staticmethod
    def reject_runtime_changes(parameters):
        return SetParametersResult(
            successful=False, reason='Restart the bridge to apply parameter changes',
        )

    def on_command(self, message):
        if self.transport_fault:
            return  # Require a fresh zero AFTER a successful recovery stop frame.
        self.watchdog.receive((
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        ), monotonic())

    def publish_preview(self, v, w, payload, reason):
        mode = 'DRY_RUN' if self.sender.dry_run else 'CAN_TX'
        line = (f'{mode} {self.sender.channel} {self.sender.can_id:03X}#{payload.hex().upper()} '
                f'v={v:+.6f} w={w:+.6f} reason={reason}')
        if rclpy.ok(context=self.context):
            self.preview.publish(String(data=line))

    def tick(self):
        now = monotonic()
        v, w = self.watchdog.sample(now)
        payload = make_payload(v, w)
        try:
            self.sender.send(payload)
        except Exception as error:
            self.transport_fault = True
            self.watchdog.stop('can_tx_error')
            if now - self.last_error_log >= 1.0:
                self.get_logger().error(f'CAN TX failed; sending only stops until recovery: {error}')
                self.last_error_log = now
            return
        if self.transport_fault:
            self.transport_fault = False
            self.get_logger().warning('CAN stop write recovered; a fresh zero command is required')
        self.publish_preview(v, w, payload, self.watchdog.reason)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.timer.cancel()
        self.watchdog.stop('shutdown')
        errors = self.sender.close(
            count=self.stop_count, period=self.period,
            on_stop=lambda: self.publish_preview(0.0, 0.0, bytes(8), 'shutdown'),
        )
        if errors:
            self.get_logger().error(f'Shutdown stop/close had {len(errors)} errors: {errors[-1]}')
        else:
            mode = 'previewed' if self.sender.dry_run else 'written to CAN socket'
            self.get_logger().info(f'{self.stop_count} shutdown stop frames {mode}')


def main(args=None):
    # Keep the ROS context alive during shutdown so stop frames and previews can
    # finish. SIGINT/SIGTERM request exit; they do not send from a signal handler.
    stop_requested = Event()
    previous_handlers = {}
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, lambda *_: stop_requested.set())
        node = CmdVelToCan()
        while rclpy.ok() and not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except CanInterfaceError as error:
        rclpy.logging.get_logger('cmd_vel_to_can').error(str(error))
        raise SystemExit(1)
    finally:
        try:
            if node is not None:
                try:
                    node.close()
                finally:
                    node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
