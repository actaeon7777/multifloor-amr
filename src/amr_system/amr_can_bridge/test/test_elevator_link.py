"""ElevatorLink frame handling with an injected bus; no CAN hardware."""

import builtins
from types import SimpleNamespace

import pytest

from amr_can_bridge.elevator_can_link import ElevatorLink
from amr_can_bridge.elevator_protocol import (
    CMD_CAR_FLOOR, DIR_NONE, ELEVATOR_COMMAND_CAN_ID, ELEVATOR_EVENT_CAN_ID, EVENT_PRESENCE,
    LOCATION_HALL, ElevatorEvent, encode_event,
)

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
    bus.fail = True
    with pytest.raises(OSError):
        link.command_frame(CMD_CAR_FLOOR, 5, DIR_NONE, 43)
    assert (link.sent, link.tx_errors) == (1, 1)


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
