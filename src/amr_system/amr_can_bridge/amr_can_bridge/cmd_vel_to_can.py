"""Translate /cmd_vel to the proven Jetson -> STM32 CAN velocity protocol.

Drive frames are gated by the STM32 E-stop status (0x202): nothing is sent while
stopped, then (0, 0) until the STM32 drive gate opens.
"""

import json
from math import isfinite
import signal
from time import monotonic, time
import uuid

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from amr_can_bridge.can_sender import CanInterfaceError, CanSender
from amr_can_bridge.drive_protocol import CommandWatchdog, make_payload
from amr_can_bridge.estop_protocol import (
    ARMING, ESTOP, ESTOP_STATUS_ID, READY, SEND_COMMAND, SEND_ZERO, UNKNOWN,
    EstopSupervisor, InvalidStatus, decode_estop_status,
)


BYPASS = 'BYPASS'
# The STM32 closes its drive gate after a 300 ms gap between accepted 0x201 frames.
MIN_TX_RATE = 10.0
STATE_DETAIL = {
    UNKNOWN: 'STM32 E-stop 상태 수신을 기다립니다.',
    ESTOP: 'E-stop 정지 중입니다. 버튼을 원위치로 돌리세요.',
    ARMING: 'E-stop 해제됨. STM32 주행 준비를 기다립니다.',
    READY: '주행 가능',
    BYPASS: 'E-stop 상태 확인 없이 동작 중입니다 (dry run 또는 구 펌웨어).',
}
ARMING_SLOW_DETAIL = ('STM32 주행 준비가 늦어지고 있습니다. '
                      'ODrive 연결·오류 상태를 확인하세요.')


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


class CmdVelToCan(Node):
    def __init__(self, **kwargs):
        super().__init__('cmd_vel_to_can', **kwargs)
        self.sender = None
        self.closed = False
        defaults = {
            'channel': 'can0', 'can_id': 0x201, 'dry_run': False,
            'tx_rate': 20.0, 'command_timeout': 0.50, 'send_timeout': 0.01,
            'max_linear_speed': 0.20, 'max_angular_speed': 0.40,
            'shutdown_stop_count': 5,
            'require_estop_status': True, 'status_timeout': 0.50,
        }
        config = {name: self.declare_parameter(name, value).value
                  for name, value in defaults.items()}
        try:
            rate = config['tx_rate']
            if not isfinite(rate) or rate < MIN_TX_RATE:
                raise ValueError(f'tx_rate must be finite and at least {MIN_TX_RATE:g} Hz '
                                 '(the STM32 drive gate closes after 300 ms)')
            self.period = 1.0 / rate
            if config['send_timeout'] >= self.period:
                raise ValueError('send_timeout must be shorter than the TX period')
            self.stop_count = config['shutdown_stop_count']
            if type(self.stop_count) is not int or not 1 <= self.stop_count <= 100:
                raise ValueError('shutdown_stop_count must be between 1 and 100')
            if type(config['require_estop_status']) is not bool:
                raise ValueError('require_estop_status must be a boolean')
            self.watchdog = CommandWatchdog(
                timeout=config['command_timeout'],
                max_linear_speed=config['max_linear_speed'],
                max_angular_speed=config['max_angular_speed'],
            )
            # Without status frames (dry run, or firmware older than 0x202) the
            # interlock cannot run; the bridge then behaves as before.
            self.estop = None
            if config['require_estop_status'] and not config['dry_run']:
                self.estop = EstopSupervisor(config['status_timeout'])
                if config['status_timeout'] <= self.period:
                    raise ValueError('status_timeout must be longer than the TX period')
            self.estop_epoch = 0
            self.estop_state = self.state()
            self.session, self.sequence = str(uuid.uuid4()), 0
            self.preview = self.create_publisher(String, '~/tx_preview', 1)
            self.status_publisher = self.create_publisher(
                String, '/estop/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.diagnostics = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
            self.subscription = self.create_subscription(Twist, '/cmd_vel', self.on_command, 1)
            self.transport_fault = False
            self.last_error_log = float('-inf')
            self.rx_error = ''
            self.sender = CanSender(
                channel=config['channel'], can_id=config['can_id'], dry_run=config['dry_run'],
                send_timeout=config['send_timeout'],
                receive_ids=(ESTOP_STATUS_ID,) if self.estop is not None else (),
            )
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(self.period, self.tick, clock=steady)
            self.diagnostic_timer = self.create_timer(1.0, self.publish_diagnostics, clock=steady)
            self.add_on_set_parameters_callback(self.reject_runtime_changes)
        except Exception:
            if self.sender is not None:
                self.sender.close(count=0)
            self.destroy_node()
            raise
        mode = 'DRY_RUN: CAN socket is not opened' if self.sender.dry_run else 'LIVE CAN TX'
        self.get_logger().info(
            f'{mode}; {self.sender.channel} ID=0x{self.sender.can_id:03X}, DLC=8, '
            f'<ff> v[m/s], w[rad/s], {rate:g} Hz. Waiting for a fresh zero command.'
        )
        if self.estop is None:
            self.get_logger().warning(
                'E-stop status 0x202 is NOT checked (dry_run or require_estop_status=false); '
                'restart after an E-stop is not interlocked by this bridge.')
        else:
            self.get_logger().info(
                'E-stop interlock on: no drive frames until 0x202 is clear; '
                're-arms automatically after release.')
        self.publish_status()

    @staticmethod
    def reject_runtime_changes(parameters):
        return SetParametersResult(
            successful=False, reason='Restart the bridge to apply parameter changes',
        )

    def state(self):
        return BYPASS if self.estop is None else self.estop.state

    def on_command(self, message):
        if self.transport_fault:
            return  # Require a fresh zero AFTER a successful recovery stop frame.
        self.watchdog.receive((
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        ), monotonic())

    def receive_status(self, now):
        if self.estop is None:
            return
        try:
            frames = self.sender.receive()
            self.rx_error = ''
        except Exception as error:
            # The status timeout turns a dead receive path into an E-stop.
            self.rx_error = f'CAN RX failed: {error}'
            frames = []
        for frame in frames:
            try:
                status = decode_estop_status(
                    frame.arbitration_id, bytes(frame.data), extended=frame.is_extended_id,
                    remote=frame.is_remote_frame, error=frame.is_error_frame,
                    fd=getattr(frame, 'is_fd', False))
            except InvalidStatus as error:
                self.estop.receive_invalid('invalid_status')
                self.get_logger().error(f'Invalid E-stop status frame: {error}',
                                        throttle_duration_sec=1.0)
                continue
            if status is not None:
                self.estop.receive(status, now)
        self.estop.check(now)

    def after_estop_update(self):
        if self.estop is None:
            return
        if self.estop.epoch != self.estop_epoch:
            self.estop_epoch = self.estop.epoch
            # Discard any cached motion; only commands issued after re-arming move the robot.
            self.watchdog.stop('estop')
            self.get_logger().error(f'E-stop event ({self.estop.reason}); drive frames blocked')
        state = self.estop.state
        if state != self.estop_state:
            if state == ARMING:
                # Released or gate closed: zeros sent before re-arming do not count.
                self.watchdog.stop(self.estop.reason)
            text = f'E-stop interlock {self.estop_state} -> {state} ({self.estop.reason})'
            if state == ESTOP:
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)
            self.estop_state = state
            self.publish_status()

    def publish_preview(self, v, w, payload, reason):
        # The shutdown stop frame also lands here; check the context before rcl.
        if not rclpy.ok(context=self.context):
            return
        if self.preview.get_subscription_count() == 0:
            return  # tests and debugging only; skip the hex formatting at 20 Hz
        mode = 'DRY_RUN' if self.sender.dry_run else 'CAN_TX'
        line = (f'{mode} {self.sender.channel} {self.sender.can_id:03X}#{payload.hex().upper()} '
                f'v={v:+.6f} w={w:+.6f} reason={reason}')
        self.preview.publish(String(data=line))

    def publish_status(self):
        if not rclpy.ok(context=self.context):
            return
        state = self.state()
        report = {} if self.estop is None else self.estop.report(monotonic())
        self.sequence += 1
        # Same envelope as /system/time_health, so consumers can use HealthLease.
        self.status_publisher.publish(String(data=json.dumps(dict(
            report, session=self.session, sequence=self.sequence, stamp=time(),
            epoch=report.get('epoch', 0), state=state,
            healthy=state in (READY, BYPASS), latched=state in (UNKNOWN, ESTOP),
            detail=self.detail(),
            rx_error=self.rx_error))))

    def detail(self):
        if self.estop is not None and self.estop.arming_slow(monotonic()):
            return ARMING_SLOW_DETAIL
        return STATE_DETAIL[self.state()]

    def tick(self):
        now = monotonic()
        self.receive_status(now)
        self.after_estop_update()
        allowed = SEND_COMMAND if self.estop is None else self.estop.output(now)
        self.publish_status()
        if allowed not in (SEND_COMMAND, SEND_ZERO):
            return  # ESTOP/UNKNOWN: send no 0x201 at all, not even zeros.
        v, w = self.watchdog.sample(now)
        reason = self.watchdog.reason
        if allowed == SEND_ZERO:
            v = w = 0.0
            reason = 'estop_arming'
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
        if allowed == SEND_ZERO:
            self.estop.zero_sent(now)
        if self.transport_fault:
            self.transport_fault = False
            self.get_logger().warning('CAN stop write recovered; a fresh zero command is required')
        self.publish_preview(v, w, payload, reason)

    def publish_diagnostics(self):
        state = self.state()
        status = DiagnosticStatus()
        status.name = 'amr/estop'
        status.hardware_id = self.sender.channel
        if self.rx_error or state in (UNKNOWN, ESTOP):
            status.level = DiagnosticStatus.ERROR
        elif state in (ARMING, BYPASS):
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.message = self.rx_error or self.detail()
        values = {'state': state}
        if self.estop is not None:
            report = self.estop.report(monotonic())
            values.update({key: value for key, value in report.items() if key != 'stm32'})
            values.update(report['stm32'])
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics.publish(message)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.timer.cancel()
        self.diagnostic_timer.cancel()
        self.watchdog.stop('shutdown')
        # Zeros would open the STM32 drive gate, so none are sent while blocked.
        count = self.stop_count if self.state() in (READY, ARMING, BYPASS) else 0
        errors = self.sender.close(
            count=count, period=self.period,
            on_stop=lambda: self.publish_preview(0.0, 0.0, bytes(8), 'shutdown'),
        )
        if errors:
            self.get_logger().error(f'Shutdown stop/close had {len(errors)} errors: {errors[-1]}')
        elif count:
            mode = 'previewed' if self.sender.dry_run else 'written to CAN socket'
            self.get_logger().info(f'{count} shutdown stop frames {mode}')
        else:
            self.get_logger().info(f'No shutdown stop frames: E-stop interlock is {self.state()}')


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
        node = CmdVelToCan()
        executor.add_node(node)
        # Return to Python regularly: the C++ wait does not dispatch Python signals.
        while rclpy.ok() and not stopped.done():
            executor.spin_until_future_complete(stopped, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except CanInterfaceError as error:
        rclpy.logging.get_logger('cmd_vel_to_can').error(str(error))
        raise SystemExit(1)
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
