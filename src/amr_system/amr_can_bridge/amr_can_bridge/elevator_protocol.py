"""Elevator RF relay frames between Jetson and STM32 (8-byte classic CAN).

The STM32 owns the RF module: it forwards AMR button commands to the RF devices
and relays hall beacons / button acknowledgements back over CAN.

TODO(firmware): the IDs and byte layout below are PROPOSALS, not agreed with the
STM32 firmware. Keep every protocol constant in this file so confirming them only
changes this module and its tests. Existing IDs: 0x001-0x038 ODrive nodes 0/1,
0x201 drive command, 0x301/0x302 IMU.

Command 0x210 (Jetson -> STM32 -> RF)
  [0] version [1] sequence [2] command [3] floor_code (int8) [4] direction
  [5] reserved [6] reserved [7] CRC-8 of bytes 0..6
Event 0x310 (RF -> STM32 -> Jetson)
  [0] version [1] sequence [2] event [3] floor_code (int8) [4] location
  [5] ack_sequence [6] rssi_dbm (int8, 0 = unknown) [7] CRC-8 of bytes 0..6
"""

from dataclasses import dataclass
import struct

ELEVATOR_COMMAND_CAN_ID = 0x210  # TODO(firmware): proposed
ELEVATOR_EVENT_CAN_ID = 0x310  # TODO(firmware): proposed
PROTOCOL_VERSION = 1

CMD_HALL_CALL, CMD_CAR_FLOOR, CMD_DOOR_OPEN = 1, 2, 3
DIR_NONE, DIR_UP, DIR_DOWN = 0, 1, 2
EVENT_PRESENCE, EVENT_BUTTON_ACK = 1, 2
LOCATION_HALL, LOCATION_CAR = 0, 1

_FRAME = struct.Struct('<BBBbBBbB')


@dataclass(frozen=True)
class ElevatorEvent:
    event: int
    floor_code: int
    location: int
    sequence: int
    ack_sequence: int
    rssi_dbm: int


def crc8(data):
    """CRC-8/SMBUS (poly 0x07, init 0x00), cheap on the STM32."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xff if crc & 0x80 else (crc << 1) & 0xff
    return crc


def _int(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'{name} must be an integer in [{low}, {high}]')
    return value


def validate_command(command, floor_code, direction):
    _int(command, 'command', 1, 3)
    _int(floor_code, 'floor_code', -128, 127)
    _int(direction, 'direction', 0, 2)
    if command == CMD_HALL_CALL and (floor_code == 0 or direction == DIR_NONE):
        raise ValueError('hall call needs a floor and a direction')
    if command == CMD_CAR_FLOOR and (floor_code == 0 or direction != DIR_NONE):
        raise ValueError('car floor command needs a floor and no direction')
    if command == CMD_DOOR_OPEN and (floor_code != 0 or direction != DIR_NONE):
        raise ValueError('door-open command takes no floor or direction')


def encode_command(command, floor_code, direction, sequence):
    validate_command(command, floor_code, direction)
    _int(sequence, 'sequence', 0, 255)
    body = _FRAME.pack(PROTOCOL_VERSION, sequence, command, floor_code, direction, 0, 0, 0)[:7]
    return body + bytes([crc8(body)])


def decode_event(data):
    """Return an ElevatorEvent or raise ValueError for any malformed frame."""
    data = bytes(data)
    if len(data) != 8:
        raise ValueError('elevator event frames must have 8 bytes')
    if crc8(data[:7]) != data[7]:
        raise ValueError('elevator event CRC mismatch')
    version, sequence, event, floor_code, location, ack_sequence, rssi, _ = _FRAME.unpack(data)
    if version != PROTOCOL_VERSION:
        raise ValueError(f'unsupported elevator protocol version {version}')
    if event not in (EVENT_PRESENCE, EVENT_BUTTON_ACK):
        raise ValueError(f'unknown elevator event {event}')
    if location not in (LOCATION_HALL, LOCATION_CAR):
        raise ValueError(f'unknown elevator device location {location}')
    if (location == LOCATION_HALL) == (floor_code == 0):
        raise ValueError('hall devices carry a floor code and the car device carries 0')
    if event == EVENT_PRESENCE and location != LOCATION_HALL:
        raise ValueError('only hall devices announce a floor')
    return ElevatorEvent(event, floor_code, location, sequence, ack_sequence, rssi)


def encode_event(event):
    """Inverse of decode_event; used by tests and STM32 bench tools."""
    body = _FRAME.pack(PROTOCOL_VERSION, event.sequence, event.event, event.floor_code,
                       event.location, event.ack_sequence, event.rssi_dbm, 0)[:7]
    frame = body + bytes([crc8(body)])
    decode_event(frame)
    return frame
