"""ElevatorLink frame handling with an injected bus; no CAN hardware."""

import builtins
from types import SimpleNamespace

import pytest

from amr_can_bridge.elevator_can_link import CAN_FILTER, ElevatorLink
from amr_can_bridge.elevator_protocol import (
    CMD_CAR_FLOOR, CMD_DOOR_OPEN, DIR_NONE, ELEVATOR_COMMAND_CAN_ID, ELEVATOR_DIAG_CAN_ID,
    ELEVATOR_EVENT_CAN_ID, EVENT_PRESENCE, LOCATION_CAR, LOCATION_HALL, STATUS_COMPLETED,
    CommandStatus, DeviceStatus, ElevatorEvent, LinkStatus, encode_diagnostic, encode_event,
)
from amr_can_bridge.elevator_rf_status import ERROR, OK, WARN, RfStatusBoard

pytest.importorskip('can')


class FakeBus:
    def __init__(self, inbox=()):
        self.inbox = list(inbox)
        self.sent = []
        self.shut = False
        self.fail = False

    def send(self, message, timeout):
        if self.fail:
            raise OSError('bus down')
        self.sent.append(message)

    def recv(self, timeout):
        assert timeout == 0.
        return self.inbox.pop(0) if self.inbox else None

    def shutdown(self):
        self.shut = True


def frame(data, stamp, can_id=ELEVATOR_EVENT_CAN_ID, **flags):
    return SimpleNamespace(arbitration_id=can_id, data=data, timestamp=stamp,
                           is_extended_id=flags.get('extended', False), is_remote_frame=False,
                           is_error_frame=False, is_fd=False)


PRESENCE = ElevatorEvent(EVENT_PRESENCE, 4, LOCATION_HALL, 7, 0, -55)


def test_dry_run_never_imports_can(monkeypatch):
    original = builtins.__import__

    def guard(name, *args, **kwargs):
        if name == 'can' or name.startswith('can.'):
            pytest.fail('dry run imported CAN')
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', guard)
    link = ElevatorLink(bus=None)
    assert link.command_frame(CMD_CAR_FLOOR, 3, DIR_NONE, 1)[:4] == bytes([1, 1, 2, 3])
    assert link.poll(100.) == []


def test_command_is_sent_on_the_elevator_id_only():
    bus = FakeBus()
    link = ElevatorLink(bus)
    link.command_frame(CMD_CAR_FLOOR, 5, DIR_NONE, 42)
    (message,) = bus.sent
    assert message.arbitration_id == ELEVATOR_COMMAND_CAN_ID and not message.is_extended_id
    assert bytes(message.data)[1:4] == bytes([42, 2, 5])
    link.command_frame(CMD_DOOR_OPEN, 0, DIR_NONE, 0x19, 10)
    assert bytes(bus.sent[-1].data).hex(' ') == '01 19 03 00 00 0a 00 03'
    bus.fail = True
    with pytest.raises(OSError):
        link.command_frame(CMD_CAR_FLOOR, 5, DIR_NONE, 43)
    assert (link.sent, link.tx_errors) == (2, 1)


def test_receive_filters_stale_invalid_and_foreign_frames():
    now = 1000.
    good = encode_event(PRESENCE)
    corrupt = bytearray(good)
    corrupt[7] ^= 0xFF
    bus = FakeBus([
        frame(good, now - .05),
        frame(good, now - 2.),                     # older than max_frame_age
        frame(bytes(corrupt), now - .01),          # CRC error
        frame(good, now - .01, can_id=0x201),      # not ours (kernel filter bypassed)
        frame(good, now - .01, extended=True),
    ])
    link = ElevatorLink(bus, max_frame_age=.5)
    events = link.poll(now)
    assert events == [(PRESENCE, now - .05)]
    assert (link.received, link.stale, link.invalid) == (3, 1, 1)
    link.close()
    assert bus.shut and link.bus is None


def kernel_filter_passes(can_id):
    return can_id & CAN_FILTER['can_mask'] == CAN_FILTER['can_id'] & CAN_FILTER['can_mask']


def test_kernel_filter_passes_events_and_diagnostics_only():
    assert kernel_filter_passes(ELEVATOR_EVENT_CAN_ID)
    assert kernel_filter_passes(ELEVATOR_DIAG_CAN_ID)
    assert not any(kernel_filter_passes(i) for i in (0x201, 0x202, 0x210, 0x301, 0x302, 0x312))


def test_diagnostics_and_car_presence_are_received():
    now = 1000.
    car = ElevatorEvent(EVENT_PRESENCE, 0, LOCATION_CAR, 0x8F, 0, -60)
    completed = CommandStatus(0x19, STATUS_COMPLETED, 0, 0x21, 1)
    corrupt = bytearray(encode_diagnostic(completed))
    corrupt[3] ^= 1
    bus = FakeBus([
        frame(encode_event(car), now - .01),
        frame(encode_diagnostic(completed), now - .01, can_id=ELEVATOR_DIAG_CAN_ID),
        frame(bytes(corrupt), now - .01, can_id=ELEVATOR_DIAG_CAN_ID),
        frame(encode_diagnostic(completed), now - 2., can_id=ELEVATOR_DIAG_CAN_ID),
    ])
    link = ElevatorLink(bus)
    assert link.poll(now) == [(car, now - .01), (completed, now - .01)]
    assert (link.received, link.invalid, link.stale) == (4, 1, 1)
    assert link.recent_error(now) and not link.recent_error(now + 11.)


def test_rf_status_board_merges_presence_and_device_status():
    board = RfStatusBoard(link_timeout=3.)
    now = 50.
    assert board.report(now, live=True)['level'] == WARN  # No LINK_STATUS yet.
    board.diagnostic(LinkStatus(0x01, 10, 0, 0, 0), now)
    board.event(ElevatorEvent(EVENT_PRESENCE, 4, LOCATION_HALL, 7, 0, -60), now)
    board.event(ElevatorEvent(EVENT_PRESENCE, 0, LOCATION_CAR, 3, 0, -80), now)
    board.diagnostic(DeviceStatus(0x44, 0x60, 3000, 1), now)
    board.diagnostic(CommandStatus(0x17, 3, 0x82, 0x45, 1), now)
    report = board.report(now + 1., live=True)
    assert report['level'] == OK and report['problems'] == []
    assert [b['label'] for b in report['beacons']] == ['E1', '4F']
    hall = report['beacons'][1]
    assert (hall['battery_mv'], hall['flags'], hall['rssi_dbm']) == (3000, ['HAS_UP', 'HAS_DOWN'], -60)
    assert hall['presence_age'] == 1.0
    assert report['last_command'] == {
        'cmd_seq': 0x17, 'status': 'DEVICE_REJECTED', 'device_result': 'UNSUPPORTED',
        'target': 0x45, 'target_label': '5F', 'attempts': 1, 'age': 1.0}
    assert report['link']['radio_ok'] and report['link']['rx_ok'] == 10

    board.diagnostic(DeviceStatus(0x41, 0x22, 2100, 1), now)  # 1F battery low.
    assert board.level(now, live=True) == WARN
    board.diagnostic(DeviceStatus(0x21, 0x18, 3000, 1), now)  # Car CONFIG_ERR.
    assert board.level(now, live=True) == ERROR
    board.diagnostic(LinkStatus(0x00, 11, 0, 0, 0), now)
    assert any('nRF24' in text for text in board.report(now, live=True)['problems'])
    assert board.level(now + 4., live=False) == ERROR  # Beacon faults persist.
    assert any('0x311' in text for _, text in board.problems(now + 4., live=True))


@pytest.mark.parametrize('flag', [
    'is_extended_id', 'is_remote_frame', 'is_error_frame', 'is_fd', None,
])
def test_handle_ignores_foreign_ids_and_non_classic_frames(flag):
    message = frame(encode_event(PRESENCE), 100.)
    if flag is None:
        message.arbitration_id = 0x201
    else:
        setattr(message, flag, True)
    link = ElevatorLink()
    assert link.handle(message, 100.) is None
    assert (link.received, link.stale, link.invalid) == (0, 0, 0)
    assert link.last_error == '' and link.last_error_at is None


@pytest.mark.parametrize('stamp,valid', [
    (99.5, True), (99.499, False), (100.019, True), (100.021, False),
    (0., False), (-1., False), (float('nan'), False), (float('inf'), False),
])
def test_handle_kernel_timestamp_window(stamp, valid):
    link = ElevatorLink(max_frame_age=.5)
    result = link.handle(frame(encode_event(PRESENCE), stamp), 100.)
    assert result == ((PRESENCE, stamp) if valid else None)
    assert (link.received, link.stale, link.invalid) == (1, int(not valid), 0)


def test_handle_matches_poll_records_bytes_counters_and_decode_errors():
    records = [PRESENCE, CommandStatus(0x19, STATUS_COMPLETED, 0, 0x21, 1),
               LinkStatus(1, 10, 2, 3, 4), DeviceStatus(0x44, 0x60, 3000, 1)]
    messages = [frame(encode_event(records[0]), 100.)] + [
        frame(encode_diagnostic(record), 100., can_id=ELEVATOR_DIAG_CAN_ID)
        for record in records[1:]]
    corrupt = bytearray(encode_event(PRESENCE))
    corrupt[-1] ^= 0xff
    messages += [frame(bytes(corrupt), 100.), frame(b'', 100.),
                 frame(encode_event(PRESENCE), 99.), frame(encode_event(PRESENCE), 101.)]
    direct, polled = ElevatorLink(), ElevatorLink(FakeBus(messages))
    actual = [result for message in messages if (result := direct.handle(message, 100.))]
    assert actual == polled.poll(100.) == [(record, 100.) for record in records]
    assert encode_event(actual[0][0]) == messages[0].data
    for (record, _), message in zip(actual[1:], messages[1:]):
        assert encode_diagnostic(record) == message.data
    for link in (direct, polled):
        assert (link.received, link.stale, link.invalid) == (8, 2, 2)
        assert link.last_error and link.last_error_at == 100.
    assert direct.last_error == polled.last_error
