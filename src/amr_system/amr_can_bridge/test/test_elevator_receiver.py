"""Exercise the RX thread with plain publishers; no ROS node/context is started."""
from queue import Empty, Queue
from threading import Event, Lock, Thread
from time import monotonic
from types import SimpleNamespace

import pytest
from rclpy.time import Time

from amr_can_bridge import elevator_can_link
from amr_can_bridge.elevator_protocol import (
    CMD_CAR_FLOOR, DIR_NONE, CommandStatus, ELEVATOR_DIAG_CAN_ID, STATUS_COMPLETED,
    encode_diagnostic, encode_event,
)
from amr_can_bridge.elevator_rf_status import RfStatusBoard
from test_elevator_link import PRESENCE, frame


class BlockingBus:
    def __init__(self):
        self.inbox, self.taken = Queue(), Queue()
        self.closed = False

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

    def shutdown(self):
        self.closed = True


@pytest.fixture
def receiver(monkeypatch):
    # Same transport and receiver methods, without calling Node.__init__ or rclpy.init.
    node = object.__new__(elevator_can_link.ElevatorCanLink)
    node.link = elevator_can_link.ElevatorLink(BlockingBus())
    node.lock, node.stop_receiving = Lock(), Event()
    node.board = RfStatusBoard()
    node.session, node.sequence = 'test', 0
    for name in ('beacon_pub', 'command_status_pub', 'rf_status_pub', 'diag_pub'):
        messages = Queue()
        setattr(node, name, SimpleNamespace(publish=messages.put, messages=messages))
    monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(
        now=lambda: Time(nanoseconds=100 * 10**9)))
    node.receiver = Thread(target=node.receive_loop, daemon=True)
    node.receiver.start()
    bus = node.link.bus
    try:
        yield node, bus
    finally:
        start = monotonic()
        node.close()
        assert monotonic() - start < 1.
        assert not node.receiver.is_alive() and bus.closed


def test_health_report_excludes_frame_waiting_for_its_lock(receiver, monkeypatch):
    import json

    node, bus = receiver
    monkeypatch.setattr(elevator_can_link, 'time', lambda: 100.)
    entered, release = Event(), Event()
    original = node.board.report
    errors = []

    def report(*args, **kwargs):
        assert node.lock.locked()
        entered.set()
        assert release.wait(1.)
        return original(*args, **kwargs)

    def health():
        try:
            node.publish_health()
        except Exception as error:
            errors.append(error)

    monkeypatch.setattr(node.board, 'report', report)
    worker = Thread(target=health, daemon=True)
    worker.start()
    try:
        assert entered.wait(1.)
        bus.inbox.put(frame(encode_event(PRESENCE), 99.99))
        bus.taken.get(timeout=1.)
        assert node.link.received == 0
        release.set()
        worker.join(timeout=1.)
        assert not worker.is_alive() and errors == []
        before = json.loads(node.rf_status_pub.messages.get(timeout=1.).data)
        after = json.loads(node.rf_status_pub.messages.get(timeout=1.).data)
        assert before['counters']['received'] == 0 and before['beacons'] == []
        assert after['counters']['received'] == 1
        assert after['beacons'][0]['presence_count'] == 1
        beacon = node.beacon_pub.messages.get(timeout=1.)
        assert beacon.header.stamp == Time(nanoseconds=round(99.99 * 1e9)).to_msg()
        assert (beacon.event, beacon.floor_code, beacon.sequence, beacon.source) == (
            PRESENCE.event, PRESENCE.floor_code, PRESENCE.sequence, 'can')
        diagnostic = node.diag_pub.messages.get(timeout=1.).status[0]
        assert {value.key: value.value for value in diagnostic.values}['received'] == '0'
    finally:
        release.set()
        worker.join(timeout=1.)


def test_receiver_checks_age_when_handled_and_recovers_from_recv_error(receiver, monkeypatch):
    node, bus = receiver
    now = [100.]
    monkeypatch.setattr(elevator_can_link, 'time', lambda: now[0])
    handled = Queue()
    original = node.link.handle

    def handle(message, current):
        result = original(message, current)
        handled.put(current)
        return result

    monkeypatch.setattr(node.link, 'handle', handle)
    with node.lock:
        bus.inbox.put(frame(encode_event(PRESENCE), 100.))
        bus.taken.get(timeout=1.)
        now[0] = 101.  # It becomes stale while health owns the lock.
    assert handled.get(timeout=1.) == 101.
    assert (node.link.received, node.link.stale, node.link.invalid) == (1, 1, 0)
    assert node.beacon_pub.messages.empty()
    bus.inbox.put(OSError('fake RX failure'))
    record = CommandStatus(0x19, STATUS_COMPLETED, 0, 0x21, 1)
    bus.inbox.put(frame(encode_diagnostic(record), 101., can_id=ELEVATOR_DIAG_CAN_ID))
    status = node.command_status_pub.messages.get(timeout=1.)
    assert status.header.stamp == Time(nanoseconds=101 * 10**9).to_msg()
    assert (status.cmd_seq, status.status, status.device_result, status.target,
            status.attempts, status.source) == (0x19, STATUS_COMPLETED, 0, 0x21, 1, 'can')
    with node.lock:
        assert (node.link.received, node.link.stale, node.link.invalid) == (2, 1, 0)
        assert node.link.last_error == 'fake RX failure' and node.link.last_error_at == 101.


@pytest.mark.parametrize('send_fails', [False, True])
def test_command_and_receive_publish_consistent_counters(receiver, monkeypatch, send_fails):
    import json

    node, bus = receiver
    monkeypatch.setattr(elevator_can_link, 'time', lambda: 100.)
    entered, release = Event(), Event()
    errors, logged, previews = [], [], []

    def send(message, timeout):
        assert timeout == .01
        assert node.lock.locked()
        entered.set()
        assert release.wait(1.)
        if send_fails:
            raise OSError('fake TX failure')

    def preview(message):
        # Reacquiring the same non-reentrant lock must work after command_frame returns.
        assert node.lock.acquire(timeout=1.)
        try:
            previews.append(message)
        finally:
            node.lock.release()

    def command():
        try:
            node.on_command(SimpleNamespace(command=CMD_CAR_FLOOR, floor_code=4,
                                            direction=DIR_NONE, sequence=7, repeat=0))
        except Exception as error:
            errors.append(error)

    monkeypatch.setattr(bus, 'send', send, raising=False)
    monkeypatch.setattr(node, 'get_logger', lambda: SimpleNamespace(error=logged.append))
    node.preview_pub = SimpleNamespace(publish=preview)
    worker = Thread(target=command, daemon=True)
    worker.start()
    try:
        assert entered.wait(1.)
        bus.inbox.put(frame(encode_event(PRESENCE), 99.99))
        bus.taken.get(timeout=1.)  # RX has a frame, but TX still owns the counters lock.
        assert node.link.received == 0
        assert node.rf_status_pub.messages.empty()
        release.set()
        worker.join(timeout=1.)
        assert not worker.is_alive() and errors == []
        report = json.loads(node.rf_status_pub.messages.get(timeout=1.).data)
        assert report['counters'] == dict(sent=int(not send_fails), received=1,
                                           invalid=0, stale=0, tx_errors=int(send_fails))
        assert report['beacons'][0]['presence_count'] == 1
        assert len(previews) == int(not send_fails)
        assert logged == (['Elevator command rejected: fake TX failure'] if send_fails else [])
        with node.lock:
            assert node.link.last_error == ('fake TX failure' if send_fails else '')
            assert node.link.last_error_at == (100. if send_fails else None)
    finally:
        release.set()
        worker.join(timeout=1.)
