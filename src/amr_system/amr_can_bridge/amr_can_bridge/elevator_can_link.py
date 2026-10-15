"""/elevator/command <-> CAN 0x210 and CAN 0x310 -> /elevator/beacon (STM32 RF relay).

The link owns its own SocketCAN socket filtered to the elevator event ID, so the
existing drive (cmd_vel_to_can) and telemetry (can_telemetry) sockets are untouched.
dry_run (default) never imports CAN: commands are only previewed on ~/tx_preview.
"""

from math import isfinite
import signal
from threading import Event
from time import time

from amr_elevator_interfaces.msg import ElevatorBeacon, ElevatorCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String

from amr_can_bridge.can_sender import require_interface_up
from amr_can_bridge.elevator_protocol import (
    ELEVATOR_COMMAND_CAN_ID, ELEVATOR_EVENT_CAN_ID, decode_event, encode_command,
)


class ElevatorLink:
    """ROS-independent frame handling; ``bus`` is None in dry-run."""

    def __init__(self, bus=None, max_frame_age=.5):
        if not isfinite(max_frame_age) or max_frame_age <= 0:
            raise ValueError('max_frame_age must be positive')
        self.bus = bus
        self.max_frame_age = max_frame_age
        self.sent = self.received = self.invalid = self.stale = self.tx_errors = 0
        self.last_error = ''
        if bus is not None:
            import can
            self.can = can

    def command_frame(self, command, floor_code, direction, sequence):
        payload = encode_command(command, floor_code, direction, sequence)
        if self.bus is not None:
            try:
                self.bus.send(self.can.Message(
                    arbitration_id=ELEVATOR_COMMAND_CAN_ID, data=payload, is_extended_id=False,
                    is_remote_frame=False, is_error_frame=False, is_fd=False, check=True), timeout=.01)
            except Exception as error:
                self.tx_errors += 1
                self.last_error = str(error)
                raise
        self.sent += 1
        return payload

    def poll(self, now, limit=100):
        """Return [(event, kernel_stamp)] of fresh, valid frames."""
        events = []
        if self.bus is None:
            return events
        for _ in range(limit):
            message = self.bus.recv(timeout=0.)
            if message is None:
                break
            if (message.arbitration_id != ELEVATOR_EVENT_CAN_ID or message.is_extended_id
                    or message.is_remote_frame or message.is_error_frame or message.is_fd):
                continue
            self.received += 1
            stamp = message.timestamp
            if not isfinite(stamp) or stamp <= 0 or not -.02 <= now - stamp <= self.max_frame_age:
                self.stale += 1
                continue
            try:
                events.append((decode_event(message.data), stamp))
            except ValueError as error:
                self.invalid += 1
                self.last_error = str(error)
        return events

    def close(self):
        if self.bus is not None:
            try:
                self.bus.shutdown()
            finally:
                self.bus = None


class ElevatorCanLink(Node):
    def __init__(self, bus_factory=None, **kwargs):
        super().__init__('elevator_can_link', **kwargs)
        self.link = None
        try:
            channel = self.declare_parameter('channel', 'can0').value
            dry_run = self.declare_parameter('dry_run', True).value
            max_age = float(self.declare_parameter('max_frame_age', .5).value)
            if type(dry_run) is not bool:
                raise ValueError('dry_run must be a boolean')
            if self.get_parameter('use_sim_time').value:
                raise ValueError('Live SocketCAN timestamps require use_sim_time=false')
            bus = None
            if not dry_run:
                if bus_factory is None:
                    from can.interfaces.socketcan import SocketcanBus
                    require_interface_up(channel)
                    bus_factory = SocketcanBus
                bus = bus_factory(channel=channel, receive_own_messages=False, fd=False,
                                  can_filters=[{'can_id': ELEVATOR_EVENT_CAN_ID, 'can_mask': 0x7ff,
                                                'extended': False}])
            self.link = ElevatorLink(bus, max_age)
            self.beacon_pub = self.create_publisher(ElevatorBeacon, 'elevator/beacon', 20)
            self.preview_pub = self.create_publisher(String, '~/tx_preview', 10)
            self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
            self.create_subscription(ElevatorCommand, 'elevator/command', self.on_command, 10)
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(.01, self.poll, clock=steady)
            self.create_timer(1., self.publish_health, clock=steady)
            self.add_on_set_parameters_callback(lambda _: SetParametersResult(
                successful=False, reason='Restart elevator_can_link to apply parameter changes'))
        except Exception:
            self.close()
            self.destroy_node()
            raise
        self.get_logger().info(
            f'Elevator RF relay on {channel}: TX 0x{ELEVATOR_COMMAND_CAN_ID:03X}, '
            f'RX 0x{ELEVATOR_EVENT_CAN_ID:03X} (proposed IDs, TODO firmware). '
            + ('DRY RUN: no CAN socket.' if dry_run else 'LIVE.'))

    def on_command(self, message):
        try:
            payload = self.link.command_frame(message.command, message.floor_code,
                                              message.direction, message.sequence)
        except Exception as error:  # noqa: BLE001 - report and keep relaying
            self.get_logger().error(f'Elevator command rejected: {error}')
            return
        self.preview_pub.publish(String(data=f'0x{ELEVATOR_COMMAND_CAN_ID:03X} {payload.hex(" ")}'))

    def poll(self):
        for event, stamp in self.link.poll(time()):
            beacon = ElevatorBeacon(
                event=event.event, location=event.location, floor_code=event.floor_code,
                sequence=event.sequence, ack_sequence=event.ack_sequence,
                rssi_dbm=event.rssi_dbm, source='can')
            beacon.header.stamp = Time(nanoseconds=round(stamp * 1e9)).to_msg()
            self.beacon_pub.publish(beacon)

    def publish_health(self):
        link = self.link
        status = DiagnosticStatus(
            name='amr/elevator_can_link', hardware_id='can',
            level=DiagnosticStatus.WARN if (link.invalid or link.tx_errors) else DiagnosticStatus.OK,
            message=link.last_error or ('dry run' if link.bus is None else 'ok'),
            values=[KeyValue(key=k, value=str(getattr(link, k)))
                    for k in ('sent', 'received', 'invalid', 'stale', 'tx_errors')])
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(array)

    def close(self):
        if self.link is not None:
            self.link.close()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = None
    try:
        node = ElevatorCanLink()
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
