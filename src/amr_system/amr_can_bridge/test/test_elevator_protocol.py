"""Elevator relay frames: exact bytes, CRC and rejection of malformed frames."""

import pytest

from amr_can_bridge.elevator_protocol import (
    CMD_CAR_FLOOR, CMD_DOOR_OPEN, CMD_HALL_CALL, DIR_DOWN, DIR_NONE, DIR_UP,
    ELEVATOR_COMMAND_CAN_ID, ELEVATOR_DIAG_CAN_ID, ELEVATOR_EVENT_CAN_ID, EVENT_BUTTON_ACK,
    EVENT_PRESENCE, LOCATION_CAR, LOCATION_HALL, STATUS_COMPLETED, CommandStatus, DeviceStatus,
    ElevatorEvent, LinkStatus, beacon_node_id, crc8, decode_diagnostic, decode_event,
    encode_command, encode_diagnostic, encode_event,
)
from amr_can_bridge.drive_protocol import DRIVE_CAN_ID
from amr_can_bridge.estop_protocol import ESTOP_STATUS_ID
from amr_can_bridge.telemetry import TELEMETRY_IDS


def test_elevator_ids_do_not_collide_with_existing_frames():
    used = set(TELEMETRY_IDS) | {DRIVE_CAN_ID, ESTOP_STATUS_ID}
    used |= {node << 5 | cmd for node in (0, 1) for cmd in range(0x20)}  # every ODrive cmd id
    ours = {ELEVATOR_COMMAND_CAN_ID, ELEVATOR_EVENT_CAN_ID, ELEVATOR_DIAG_CAN_ID}
    assert len(ours) == 3 and not ours & used
    assert ELEVATOR_EVENT_CAN_ID & 0x7FE == ELEVATOR_DIAG_CAN_ID & 0x7FE  # One kernel filter.


# docs/ELEVATOR_CAN_PROTOCOL.md section 9, byte for byte.
COMMAND_VECTORS = [
    ((CMD_HALL_CALL, 4, DIR_DOWN, 0x15), '01 15 01 04 02 00 00 20'),
    ((CMD_CAR_FLOOR, 3, DIR_NONE, 0x16), '01 16 02 03 00 00 00 49'),
    ((CMD_DOOR_OPEN, 0, DIR_NONE, 0x18), '01 18 03 00 00 00 00 A8'),
    ((CMD_DOOR_OPEN, 0, DIR_NONE, 0x19, 10), '01 19 03 00 00 0A 00 03'),
]
EVENT_VECTORS = [
    (ElevatorEvent(EVENT_BUTTON_ACK, 0, LOCATION_CAR, 0x92, 0x19, -60), '01 92 02 00 01 19 C4 E8'),
    (ElevatorEvent(EVENT_PRESENCE, 4, LOCATION_HALL, 0x2A, 0, -60), '01 2A 01 04 00 00 C4 91'),
    (ElevatorEvent(EVENT_PRESENCE, 3, LOCATION_HALL, 0x10, 0, -80), '01 10 01 03 00 00 B0 00'),
    (ElevatorEvent(EVENT_BUTTON_ACK, 4, LOCATION_HALL, 0x2B, 0x15, -60), '01 2B 02 04 00 15 C4 08'),
    (ElevatorEvent(EVENT_BUTTON_ACK, 0, LOCATION_CAR, 0x90, 0x16, -60), '01 90 02 00 01 16 C4 79'),
    (ElevatorEvent(EVENT_PRESENCE, 0, LOCATION_CAR, 0x8F, 0, -60), '01 8F 01 00 01 00 C4 F8'),
]
DIAGNOSTIC_VECTORS = [
    (CommandStatus(0x19, STATUS_COMPLETED, 0x00, 0x21, 1), '01 01 19 02 00 21 01 2F'),
    (CommandStatus(0x15, STATUS_COMPLETED, 0x00, 0x44, 1), '01 01 15 02 00 44 01 0D'),
    (CommandStatus(0x17, 3, 0x82, 0x45, 1), '01 01 17 03 82 45 01 17'),
    (CommandStatus(0x16, 4, 0xFF, 0x21, 2), '01 01 16 04 FF 21 02 49'),
    (LinkStatus(0x01, 0x34, 0, 0, 0), '01 02 01 34 00 00 00 1E'),
    (DeviceStatus(0x44, 0x60, 3000, 1), '01 03 44 60 B8 0B 01 9D'),
]


@pytest.mark.parametrize('args, data', COMMAND_VECTORS)
def test_command_vectors(args, data):
    assert encode_command(*args) == bytes.fromhex(data)


@pytest.mark.parametrize('event, data', EVENT_VECTORS)
def test_event_vectors(event, data):
    assert decode_event(bytes.fromhex(data)) == event
    assert encode_event(event) == bytes.fromhex(data)


@pytest.mark.parametrize('record, data', DIAGNOSTIC_VECTORS)
def test_diagnostic_vectors(record, data):
    assert decode_diagnostic(bytes.fromhex(data)) == record
    assert encode_diagnostic(record) == bytes.fromhex(data)


def test_beacon_node_ids_follow_the_rf_address_plan():
    assert beacon_node_id(LOCATION_HALL, 1) == 0x41
    assert beacon_node_id(LOCATION_HALL, 7) == 0x47
    assert beacon_node_id(LOCATION_CAR, 0) == 0x21


def test_crc8_reference_value():
    assert crc8(b'123456789') == 0xF4  # CRC-8/SMBUS check value


def test_command_wire_bytes():
    frame = encode_command(CMD_HALL_CALL, 4, DIR_DOWN, 17)
    assert frame[:7] == bytes([1, 17, 1, 4, 2, 0, 0])
    assert frame[7] == crc8(frame[:7])
    assert encode_command(CMD_CAR_FLOOR, -1, DIR_NONE, 255)[3] == 0xFF  # B1 as int8


@pytest.mark.parametrize('args', [
    (CMD_HALL_CALL, 0, DIR_UP, 0), (CMD_HALL_CALL, 3, DIR_NONE, 0),
    (CMD_CAR_FLOOR, 0, DIR_NONE, 0), (CMD_CAR_FLOOR, 3, DIR_UP, 0),
    (CMD_DOOR_OPEN, 3, DIR_NONE, 0), (CMD_DOOR_OPEN, 0, DIR_UP, 0),
    (9, 3, DIR_NONE, 0), (CMD_CAR_FLOOR, 200, DIR_NONE, 0), (CMD_CAR_FLOOR, 3, DIR_NONE, 256),
    (CMD_CAR_FLOOR, 3.0, DIR_NONE, 0), (True, 3, DIR_NONE, 0),
    # repeat is for DOOR_OPEN only, at most 15 presses.
    (CMD_DOOR_OPEN, 0, DIR_NONE, 0, 16), (CMD_DOOR_OPEN, 0, DIR_NONE, 0, -1),
    (CMD_DOOR_OPEN, 0, DIR_NONE, 0, 10.0), (CMD_HALL_CALL, 4, DIR_UP, 0, 1),
    (CMD_CAR_FLOOR, 3, DIR_NONE, 0, 2),
])
def test_invalid_commands_are_rejected(args):
    with pytest.raises(ValueError):
        encode_command(*args)


@pytest.mark.parametrize('event', [
    ElevatorEvent(EVENT_PRESENCE, 5, LOCATION_HALL, 0, 0, -61),
    ElevatorEvent(EVENT_BUTTON_ACK, 3, LOCATION_HALL, 255, 17, 0),
    ElevatorEvent(EVENT_BUTTON_ACK, 0, LOCATION_CAR, 9, 200, -40),
])
def test_event_round_trip(event):
    assert decode_event(encode_event(event)) == event


def test_corrupt_and_inconsistent_events_are_rejected():
    good = bytearray(encode_event(ElevatorEvent(EVENT_PRESENCE, 4, LOCATION_HALL, 1, 0, -50)))
    with pytest.raises(ValueError, match='8 bytes'):
        decode_event(good[:7])
    flipped = bytearray(good)
    flipped[3] ^= 1
    with pytest.raises(ValueError, match='CRC'):
        decode_event(flipped)

    def resealed(index, value):
        frame = bytearray(good)
        frame[index] = value
        frame[7] = crc8(frame[:7])
        return bytes(frame)

    for index, value, message in ((0, 2, 'version'), (2, 7, 'event'), (4, 3, 'location'),
                                  (3, 0, 'floor code'), (4, LOCATION_CAR, 'floor code')):
        with pytest.raises(ValueError, match=message):
            decode_event(resealed(index, value))
    car_presence = bytearray(resealed(4, LOCATION_CAR))
    car_presence[3] = 0
    car_presence[7] = crc8(car_presence[:7])
    # Protocol 1.1: the car beacon also announces itself, with floor code 0.
    assert decode_event(bytes(car_presence)).location == LOCATION_CAR


def test_malformed_diagnostics_are_rejected():
    good = bytes.fromhex('01 01 19 02 00 21 01 2F')
    with pytest.raises(ValueError, match='8 bytes'):
        decode_diagnostic(good[:7])
    flipped = bytearray(good)
    flipped[4] ^= 1
    with pytest.raises(ValueError, match='CRC'):
        decode_diagnostic(flipped)

    def resealed(index, value):
        frame = bytearray(good)
        frame[index] = value
        frame[7] = crc8(frame[:7])
        return bytes(frame)

    for index, value, message in ((0, 2, 'version'), (1, 4, 'kind'), (1, 0, 'kind'),
                                  (3, 0, 'status'), (3, 9, 'status')):
        with pytest.raises(ValueError, match=message):
            decode_diagnostic(resealed(index, value))
