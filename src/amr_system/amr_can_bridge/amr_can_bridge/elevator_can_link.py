"""/elevator/command <-> CAN 0x210; CAN 0x310/0x311 -> /elevator/beacon, command status, RF status.

The link owns its own SocketCAN socket filtered to the elevator event and diagnostic
IDs, so the existing drive (cmd_vel_to_can) and telemetry (can_telemetry) sockets are
untouched. The RF relay is independent of the E-stop: it keeps running while latched.
dry_run (default) never imports CAN: commands are only previewed on ~/tx_preview.
"""

import json
from math import isfinite
import signal
from threading import Event, Lock, Thread
from time import time
import uuid

from amr_elevator_interfaces.msg import ElevatorBeacon, ElevatorCommand, ElevatorCommandStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String

from amr_can_bridge.can_sender import require_interface_up
from amr_can_bridge.elevator_protocol import (
    ELEVATOR_COMMAND_CAN_ID, ELEVATOR_DIAG_CAN_ID, ELEVATOR_EVENT_CAN_ID, CommandStatus,
    ElevatorEvent, decode_diagnostic, decode_event, encode_command,
)
from amr_can_bridge.elevator_rf_status import ERROR, OK, WARN, RfStatusBoard

# One kernel filter entry passes 0x310 (events) and 0x311 (diagnostics).
CAN_FILTER = {'can_id': ELEVATOR_EVENT_CAN_ID, 'can_mask': 0x7FE, 'extended': False}
RECENT_ERROR_WINDOW = 10.  # s; a decode or TX error warns for this long, not forever.


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


class ElevatorLink:
    """ROS-independent frame handling; ``bus`` is None in dry-run."""

    def __init__(self, bus=None, max_frame_age=.5):
        if not isfinite(max_frame_age) or max_frame_age <= 0:
            raise ValueError('max_frame_age must be positive')
        self.bus = bus
        self.max_frame_age = max_frame_age
        self.sent = self.received = self.invalid = self.stale = self.tx_errors = 0
        self.last_error = ''
        self.last_error_at = None
        if bus is not None:
            import can
            self.can = can

    def command_frame(self, command, floor_code, direction, sequence, repeat=0):
        payload = encode_command(command, floor_code, direction, sequence, repeat)
        if self.bus is not None:
            try:
                self.bus.send(self.can.Message(
                    arbitration_id=ELEVATOR_COMMAND_CAN_ID, data=payload, is_extended_id=False,
                    is_remote_frame=False, is_error_frame=False, is_fd=False, check=True), timeout=.01)
            except Exception as error:
                self.tx_errors += 1
                self.last_error, self.last_error_at = str(error), time()
                raise
        self.sent += 1
        return payload

    def poll(self, now, limit=100):
        """Return [(record, kernel_stamp)] of fresh, valid frames.

        record is an ElevatorEvent (0x310) or a CommandStatus/LinkStatus/DeviceStatus (0x311).
        """
        records = []
        if self.bus is None:
            return records
        for _ in range(limit):
            message = self.bus.recv(timeout=0.)
            if message is None:
                break
            record = self.handle(message, now)
            if record is not None:
                records.append(record)
        return records

    def handle(self, message, now):
        """Return (record, kernel_stamp) for one fresh, valid classic CAN frame."""
        decoders = {ELEVATOR_EVENT_CAN_ID: decode_event, ELEVATOR_DIAG_CAN_ID: decode_diagnostic}
        decode = decoders.get(message.arbitration_id)
        if (decode is None or message.is_extended_id or message.is_remote_frame
                or message.is_error_frame or message.is_fd):
            return None
        self.received += 1
        stamp = message.timestamp
        if not isfinite(stamp) or stamp <= 0 or not -.02 <= now - stamp <= self.max_frame_age:
            self.stale += 1
            return None
        try:
            return decode(message.data), stamp
        except ValueError as error:
            self.invalid += 1
            self.last_error, self.last_error_at = str(error), now
            return None

    def recent_error(self, now):
        return self.last_error_at is not None and now - self.last_error_at < RECENT_ERROR_WINDOW

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
        self.lock = Lock()
        self.stop_receiving = Event()
        self.receiver = None
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
                                  can_filters=[dict(CAN_FILTER)])
            self.link = ElevatorLink(bus, max_age)
            self.board = RfStatusBoard()
            self.session, self.sequence = str(uuid.uuid4()), 0
            self.beacon_pub = self.create_publisher(ElevatorBeacon, 'elevator/beacon', 20)
            self.command_status_pub = self.create_publisher(
                ElevatorCommandStatus, 'elevator/command_status', 20)
            self.rf_status_pub = self.create_publisher(
                String, 'elevator/rf_status',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.preview_pub = self.create_publisher(String, '~/tx_preview', 10)
            self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
            self.create_subscription(ElevatorCommand, 'elevator/command', self.on_command, 10)
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(1., self.publish_health, clock=steady)
            self.add_on_set_parameters_callback(lambda _: SetParametersResult(
                successful=False, reason='Restart elevator_can_link to apply parameter changes'))
            if bus is not None:
                self.receiver = Thread(target=self.receive_loop, name='elevator_can_rx', daemon=True)
                self.receiver.start()
        except Exception:
            self.close()
            self.destroy_node()
            raise
        self.get_logger().info(
            f'Elevator RF relay on {channel}: TX 0x{ELEVATOR_COMMAND_CAN_ID:03X}, '
            f'RX 0x{ELEVATOR_EVENT_CAN_ID:03X}/0x{ELEVATOR_DIAG_CAN_ID:03X} '
            '(docs/ELEVATOR_CAN_PROTOCOL.md v1.1). '
            + ('DRY RUN: no CAN socket.' if dry_run else 'LIVE.'))

    def on_command(self, message):
        try:
            with self.lock:
                payload = self.link.command_frame(message.command, message.floor_code,
                                                  message.direction, message.sequence, message.repeat)
        except Exception as error:  # noqa: BLE001 - report and keep relaying
            self.get_logger().error(f'Elevator command rejected: {error}')
            return
        self.preview_pub.publish(String(data=f'0x{ELEVATOR_COMMAND_CAN_ID:03X} {payload.hex(" ")}'))

    def receive_loop(self):
        while not self.stop_receiving.is_set():
            try:
                message = self.link.bus.recv(timeout=.1)
                if message is not None:
                    with self.lock:
                        now = time()  # Freshness is evaluated when the frame is handled.
                        result = self.link.handle(message, now)
                        if result is not None:
                            self._publish_record(*result, now)
                        if self.board.changed:
                            self._publish_rf_status(now)
            except Exception as error:
                with self.lock:
                    self.link.last_error, self.link.last_error_at = str(error), time()
                self.stop_receiving.wait(.1)

    def poll(self):
        """Drain synchronously for callers without the receive thread."""
        with self.lock:
            now = time()
            for record, stamp in self.link.poll(now):
                self._publish_record(record, stamp, now)
            if self.board.changed:
                self._publish_rf_status(now)

    def _publish_record(self, record, stamp, now):
        """Publish and update the board while holding self.lock."""
        header_stamp = Time(nanoseconds=round(stamp * 1e9)).to_msg()
        if isinstance(record, ElevatorEvent):
            beacon = ElevatorBeacon(
                event=record.event, location=record.location, floor_code=record.floor_code,
                sequence=record.sequence, ack_sequence=record.ack_sequence,
                rssi_dbm=record.rssi_dbm, source='can')
            beacon.header.stamp = header_stamp
            self.beacon_pub.publish(beacon)
            self.board.event(record, now)
            return
        if isinstance(record, CommandStatus):
            status = ElevatorCommandStatus(
                cmd_seq=record.cmd_seq, status=record.status,
                device_result=record.device_result, target=record.target,
                attempts=record.attempts, source='can')
            status.header.stamp = header_stamp
            self.command_status_pub.publish(status)
        self.board.diagnostic(record, now)

    def publish_rf_status(self, now):
        with self.lock:
            self._publish_rf_status(now)

    def _publish_rf_status(self, now):
        self.board.changed = False
        self.sequence += 1
        link = self.link
        report = self.board.report(now, live=link.bus is not None)
        report.update(session=self.session, sequence=self.sequence, stamp=now, counters={
            k: getattr(link, k) for k in ('sent', 'received', 'invalid', 'stale', 'tx_errors')})
        self.rf_status_pub.publish(String(data=json.dumps(report)))

    def publish_health(self):
        with self.lock:
            self._publish_health()

    def _publish_health(self):
        now = time()
        self._publish_rf_status(now)
        link = self.link
        live = link.bus is not None
        problems = self.board.problems(now, live)
        level = max([DiagnosticStatus.OK] + [
            {OK: DiagnosticStatus.OK, WARN: DiagnosticStatus.WARN,
             ERROR: DiagnosticStatus.ERROR}[severity] for severity, _ in problems])
        message = '; '.join(text for _, text in problems)
        if link.recent_error(now):
            level = max(level, DiagnosticStatus.WARN)
            message = '; '.join(filter(None, [link.last_error, message]))
        status = DiagnosticStatus(
            name='amr/elevator_can_link', hardware_id='can', level=level,
            message=message or ('dry run' if not live else 'ok'),
            values=[KeyValue(key=k, value=str(getattr(link, k)))
                    for k in ('sent', 'received', 'invalid', 'stale', 'tx_errors')])
        report = self.board.report(now, live)
        if report['link'] is not None:
            status.values += [KeyValue(key=f'link_{k}', value=str(v))
                              for k, v in report['link'].items()]
        for beacon in report['beacons']:
            status.values.append(KeyValue(key=f'beacon_{beacon["label"]}', value=json.dumps(
                {k: beacon[k] for k in ('presence_age', 'rssi_dbm', 'battery_mv', 'flags')})))
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(array)

    def close(self):
        self.stop_receiving.set()
        if self.receiver is not None and self.receiver.is_alive():
            self.receiver.join(timeout=1.)
        if self.link is not None:
            self.link.close()


def main(args=None):
    # Signals only complete the future and wake the executor. Keep the ROS
    # context alive until close() has finished sending shutdown stop frames.
    previous = {}
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = executor = None
    try:
        executor = spin_executor()
        stopped = executor.create_future()

        def request_stop(*_):
            if not stopped.done():
                stopped.set_result(None)
                executor.wake()

        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, request_stop)
        node = ElevatorCanLink()
        executor.add_node(node)
        # Return to Python regularly: the C++ wait does not dispatch Python signals.
        while rclpy.ok() and not stopped.done():
            executor.spin_until_future_complete(stopped, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            try:
                if executor is not None:
                    executor.shutdown()
                    if node is not None:
                        executor.remove_node(node)
            finally:
                if node is not None:
                    try:
                        node.close()
                    finally:
                        node.destroy_node()
        finally:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                for signum, handler in previous.items():
                    signal.signal(signum, handler)
