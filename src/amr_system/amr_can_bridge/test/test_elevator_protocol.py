"""Elevator relay frames: exact bytes, CRC and rejection of malformed frames."""

import pytest

from amr_can_bridge.elevator_protocol import (
    CMD_CAR_FLOOR, CMD_DOOR_OPEN, CMD_HALL_CALL, DIR_DOWN, DIR_NONE, DIR_UP,
    ELEVATOR_COMMAND_CAN_ID, ELEVATOR_EVENT_CAN_ID, EVENT_BUTTON_ACK, EVENT_PRESENCE,
    LOCATION_CAR, LOCATION_HALL, ElevatorEvent, crc8, decode_event, encode_command, encode_event,
)
from amr_can_bridge.drive_protocol import DRIVE_CAN_ID
from amr_can_bridge.telemetry import TELEMETRY_IDS


def test_proposed_ids_do_not_collide_with_existing_frames():
    used = set(TELEMETRY_IDS) | {DRIVE_CAN_ID}
    used |= {node << 5 | cmd for node in (0, 1) for cmd in range(0x20)}  # every ODrive cmd id
    assert ELEVATOR_COMMAND_CAN_ID not in used
    assert ELEVATOR_EVENT_CAN_ID not in used
    assert ELEVATOR_COMMAND_CAN_ID != ELEVATOR_EVENT_CAN_ID


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
    with pytest.raises(ValueError, match='only hall'):
        decode_event(bytes(car_presence))
