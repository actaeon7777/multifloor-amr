"""Opt-in elevator RX/executor tests with queue transports; no physical CAN."""
import json
import os
from queue import Empty, Queue
from threading import Event, Thread
import time
from types import SimpleNamespace

import pytest

if os.environ.get('AMR_RUN_ELEVATOR_CAN_TESTS') != '1':
    pytest.skip('Set AMR_RUN_ELEVATOR_CAN_TESTS=1 for fake-bus ROS tests', allow_module_level=True)

import rclpy
from amr_elevator_interfaces.msg import ElevatorBeacon, ElevatorCommandStatus
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String

from amr_can_bridge import can_telemetry, cmd_vel_to_can, elevator_can_link
from amr_can_bridge.elevator_protocol import (
    CommandStatus, ElevatorEvent, ELEVATOR_DIAG_CAN_ID, ELEVATOR_EVENT_CAN_ID,
    EVENT_PRESENCE, LOCATION_HALL, STATUS_COMPLETED, encode_diagnostic, encode_event,
)
from executor_test_helpers import assert_test_domain, run_main_sigterm


class QueueBus:
    def __init__(self, **kwargs):
        assert kwargs['receive_own_messages'] is False
        assert kwargs['fd'] is False
        assert kwargs['can_filters'] == [elevator_can_link.CAN_FILTER]
        self.inbox, self.taken = Queue(), Queue()
        self.closed = False
        self.sent = []

    def recv(self, timeout):
        assert timeout == .1
        try:
            message = self.inbox.get(timeout=timeout)
        except Empty:
            return None
        self.taken.put(message)
        if isinstance(message, Exception):
            raise message
        return message

    def send(self, message, timeout):
        self.sent.append(message)

    def shutdown(self):
        self.closed = True


def frame(record, stamp, **changes):
    event = isinstance(record, ElevatorEvent)
    message = SimpleNamespace(
        arbitration_id=ELEVATOR_EVENT_CAN_ID if event else ELEVATOR_DIAG_CAN_ID,
        data=encode_event(record) if event else encode_diagnostic(record), timestamp=stamp,
        is_extended_id=False, is_remote_frame=False, is_error_frame=False, is_fd=False)
    for key, value in changes.items():
        setattr(message, key, value)
    return message


@pytest.fixture
def ros_context():
    assert_test_domain()
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_queue_rx_publications_health_lock_and_close(ros_context, monkeypatch):
    node = elevator_can_link.ElevatorCanLink(
        bus_factory=QueueBus, parameter_overrides=[Parameter('dry_run', value=False)])
    bus = node.link.bus
    probe = rclpy.create_node('elevator_can_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    beacons, commands, reports, diagnostics = [], [], [], []
    probe.create_subscription(ElevatorBeacon, 'elevator/beacon', beacons.append, 20)
    probe.create_subscription(ElevatorCommandStatus, 'elevator/command_status', commands.append, 20)
    probe.create_subscription(String, 'elevator/rf_status',
                              lambda msg: reports.append(json.loads(msg.data)),
                              QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    probe.create_subscription(DiagnosticArray, 'diagnostics', diagnostics.append, 10)
    now = 1000.123456789
    monkeypatch.setattr(elevator_can_link, 'time', lambda: now)
    release_health, entered_health = Event(), Event()
    health_thread = None
    errors = []

    def spin_until(predicate):
        end = time.monotonic() + 3.
        while not predicate():
            assert time.monotonic() < end
            executor.spin_once(timeout_sec=.01)

    def health():
        try:
            node.publish_health()
        except Exception as error:
            errors.append(error)

    try:
        assert node.receiver.daemon
        assert [timer.timer_period_ns for timer in node.timers] == [10**9]
        spin_until(lambda: all(pub.get_subscription_count() for pub in (
            node.beacon_pub, node.command_status_pub, node.rf_status_pub, node.diag_pub)))
        event = ElevatorEvent(EVENT_PRESENCE, 4, LOCATION_HALL, 7, 0, -55)
        completed = CommandStatus(0x19, STATUS_COMPLETED, 0, 0x21, 1)
        stamp = now - .01
        # Observe each change before sending the next: rf_status intentionally has depth 1.
        bus.inbox.put(frame(event, stamp))
        bus.taken.get(timeout=1.)
        spin_until(lambda: len(beacons) == len(reports) == 1)
        bus.inbox.put(frame(completed, stamp))
        bus.taken.get(timeout=1.)
        spin_until(lambda: len(commands) == 1 and len(reports) == 2)
        for message in (frame(event, now - 1.), frame(event, now + .03),
                        frame(event, stamp, data=b''),
                        frame(event, stamp, is_remote_frame=True),
                        frame(event, stamp, arbitration_id=0x201)):
            bus.inbox.put(message)
        for _ in range(5):
            bus.taken.get(timeout=1.)
        spin_until(lambda: len(beacons) == len(commands) == 1 and len(reports) == 2)
        expected_beacon = ElevatorBeacon(
            event=event.event, location=event.location, floor_code=event.floor_code,
            sequence=event.sequence, ack_sequence=event.ack_sequence,
            rssi_dbm=event.rssi_dbm, source='can')
        expected_command = ElevatorCommandStatus(
            cmd_seq=completed.cmd_seq, status=completed.status, device_result=completed.device_result,
            target=completed.target, attempts=completed.attempts, source='can')
        expected_beacon.header.stamp = expected_command.header.stamp = Time(
            nanoseconds=round(stamp * 1e9)).to_msg()
        assert beacons == [expected_beacon] and commands == [expected_command]
        # No health timer has run: these reports must come from board.changed in RX.
        assert reports[0]['beacons'][0]['presence_count'] == 1
        assert reports[1]['last_command']['cmd_seq'] == completed.cmd_seq
        with node.lock:
            assert (node.link.received, node.link.stale, node.link.invalid) == (5, 2, 1)

        original_report = node.board.report

        def blocked_report(*args, **kwargs):
            assert node.lock.locked()
            entered_health.set()
            assert release_health.wait(1.)
            return original_report(*args, **kwargs)

        monkeypatch.setattr(node.board, 'report', blocked_report)
        health_thread = Thread(target=health, daemon=True)
        health_thread.start()
        assert entered_health.wait(1.)
        next_event = ElevatorEvent(EVENT_PRESENCE, 5, LOCATION_HALL, 8, 0, -56)
        bus.inbox.put(frame(next_event, stamp))
        bus.taken.get(timeout=1.)  # RX has a frame but health still owns the board lock.
        assert node.link.received == 5
        release_health.set()
        health_thread.join(timeout=1.)
        assert not health_thread.is_alive() and errors == []
        spin_until(lambda: len(beacons) == 2 and reports[-1]['counters']['received'] == 6
                   and diagnostics)
        health_counters = {value.key: value.value for value in diagnostics[-1].status[0].values}
        assert all(health_counters[key] == value for key, value in {
            'sent': '0', 'received': '5', 'invalid': '1', 'stale': '2', 'tx_errors': '0'}.items())
        assert reports[-1]['counters']['received'] == 6
        assert [b['label'] for b in reports[-1]['beacons']] == ['4F', '5F']
        assert [b.sequence for b in beacons] == [7, 8]
        assert node.link.last_error_at == now
        assert not errors

        # A recv failure is recorded; the thread resumes on the next good frame.
        bus.inbox.put(OSError('fake receive failure'))
        bus.inbox.put(frame(completed, stamp))
        spin_until(lambda: len(commands) == 2)
        with node.lock:
            assert node.link.last_error == 'fake receive failure'
            assert node.link.last_error_at == now
            assert node.link.received == 7
        start = time.monotonic()
        node.close()
        assert time.monotonic() - start < 1.
        assert not node.receiver.is_alive() and bus.closed and node.link.bus is None
    finally:
        release_health.set()
        if health_thread is not None:
            health_thread.join(timeout=1.)
        node.close()
        executor.shutdown()
        executor.remove_node(probe)
        probe.destroy_node()
        node.destroy_node()


def test_dry_run_has_no_receiver(ros_context):
    def forbidden_bus(**_):
        pytest.fail('dry run opened a bus')

    node = elevator_can_link.ElevatorCanLink(bus_factory=forbidden_bus)
    try:
        assert node.receiver is None and node.link.bus is None
        assert [timer.timer_period_ns for timer in node.timers] == [10**9]
    finally:
        node.close()
        node.destroy_node()


def test_main_sigterm_stops_elevator_receiver(monkeypatch):
    assert_test_domain()
    buses = []

    def bus_factory(**kwargs):
        bus = QueueBus(**kwargs)
        buses.append(bus)
        return bus

    node_class = elevator_can_link.ElevatorCanLink
    node = run_main_sigterm(monkeypatch, elevator_can_link, 'ElevatorCanLink',
        lambda: node_class(bus_factory=bus_factory,
                           parameter_overrides=[Parameter('dry_run', value=False)]))
    assert buses[0].closed and not node.receiver.is_alive()


@pytest.mark.parametrize('module', [cmd_vel_to_can, can_telemetry, elevator_can_link])
def test_spin_executor_returns_installed_events_executor(ros_context, monkeypatch, module):
    experimental = pytest.importorskip('rclpy.experimental')
    factory = experimental.EventsExecutor  # A factory function, not a Python class.
    created = []

    def create():
        executor = factory()
        created.append(executor)
        return executor

    monkeypatch.setattr(experimental, 'EventsExecutor', create)
    executor = module.spin_executor()
    try:
        assert created == [executor]
        assert all(hasattr(executor, method) for method in (
            'create_future', 'spin_until_future_complete', 'wake', 'shutdown', 'remove_node'))
    finally:
        executor.shutdown()
