"""Elevator RF relay frames between Jetson and STM32 (8-byte classic CAN).

The STM32 owns the RF module: it forwards AMR button commands to the RF devices
and relays beacon frames, button acknowledgements and its own RF diagnostics
back over CAN. Layout: docs/ELEVATOR_CAN_PROTOCOL.md v1.1 (frame version 1).
Other IDs on the bus: 0x001-0x038 ODrive nodes 0/1, 0x201 drive command,
0x202 E-stop status, 0x301/0x302 IMU.

Command 0x210 (Jetson -> STM32 -> RF)
  [0] version [1] sequence [2] command [3] floor_code (int8) [4] direction
  [5] repeat (DOOR_OPEN only, 0..15; 0/1 = once) [6] reserved [7] CRC-8 of bytes 0..6
Event 0x310 (RF -> STM32 -> Jetson): PRESENCE from hall and car beacons, BUTTON_ACK
  [0] version [1] sequence [2] event [3] floor_code (int8) [4] location
  [5] ack_sequence [6] rssi_dbm (int8 grade: -60 RPD=1, -80 RPD=0; 0 = unknown, sim only)
  [7] CRC-8 of bytes 0..6
Diagnostic 0x311 (STM32 -> Jetson)
  [0] version [1] kind [2..6] per kind (see decode_diagnostic) [7] CRC-8 of bytes 0..6
"""

from dataclasses import dataclass
import struct

ELEVATOR_COMMAND_CAN_ID = 0x210
ELEVATOR_EVENT_CAN_ID = 0x310
ELEVATOR_DIAG_CAN_ID = 0x311
PROTOCOL_VERSION = 1

CMD_HALL_CALL, CMD_CAR_FLOOR, CMD_DOOR_OPEN = 1, 2, 3
DIR_NONE, DIR_UP, DIR_DOWN = 0, 1, 2
EVENT_PRESENCE, EVENT_BUTTON_ACK = 1, 2
LOCATION_HALL, LOCATION_CAR = 0, 1
MAX_DOOR_OPEN_REPEAT = 15
DOOR_OPEN_REPEAT_DEFAULT = 10  # About 10 s of door-open presses while the AMR leaves the car.

DIAG_COMMAND_STATUS, DIAG_LINK_STATUS, DIAG_DEVICE_STATUS = 1, 2, 3
(STATUS_DELIVERED, STATUS_COMPLETED, STATUS_DEVICE_REJECTED, STATUS_RF_TIMEOUT,
 STATUS_NO_RESULT, STATUS_NO_ROUTE, STATUS_BAD_FRAME, STATUS_REPLACED) = range(1, 9)
STATUS_NAMES = {
    STATUS_DELIVERED: 'DELIVERED', STATUS_COMPLETED: 'COMPLETED',
    STATUS_DEVICE_REJECTED: 'DEVICE_REJECTED', STATUS_RF_TIMEOUT: 'RF_TIMEOUT',
    STATUS_NO_RESULT: 'NO_RESULT', STATUS_NO_ROUTE: 'NO_ROUTE',
    STATUS_BAD_FRAME: 'BAD_FRAME', STATUS_REPLACED: 'REPLACED',
}
RESULT_NONE = 0xFF
RESULT_NAMES = {
    0x00: 'PRESSED', 0x01: 'DUPLICATE', 0x81: 'WRONG_DEVICE', 0x82: 'UNSUPPORTED',
    0x83: 'BUTTON_FAULT', 0x84: 'BAD_REQUEST', RESULT_NONE: 'NONE',
}
LINK_RADIO_OK, LINK_RX_OVERFLOW, LINK_TX_TIMEOUT = 0x01, 0x02, 0x04
DEVICE_FLAG_NAMES = {
    0x01: 'BOOT', 0x02: 'BATT_LOW', 0x04: 'BUTTON_FAULT', 0x08: 'CONFIG_ERR',
    0x10: 'HAS_DOOR_OPEN', 0x20: 'HAS_UP', 0x40: 'HAS_DOWN',
}
DEVICE_BATT_LOW, DEVICE_BUTTON_FAULT, DEVICE_CONFIG_ERR = 0x02, 0x04, 0x08
CAR_NODE_ID = 0x21  # Car beacon of elevator E1; hall beacons are 0x40 + floor_code.

_FRAME = struct.Struct('<BBBbBBbB')


@dataclass(frozen=True)
class ElevatorEvent:
    event: int
    floor_code: int
    location: int
    sequence: int
    ack_sequence: int
    rssi_dbm: int


@dataclass(frozen=True)
class CommandStatus:
    cmd_seq: int
    status: int
    device_result: int
    target: int
    attempts: int


@dataclass(frozen=True)
class LinkStatus:
    flags: int
    rx_ok: int
    rx_bad: int
    tx_fail: int
    pending: int


@dataclass(frozen=True)
class DeviceStatus:
    node_id: int
    flags: int
    battery_mv: int
    fw_version: int


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


def validate_command(command, floor_code, direction, repeat=0):
    _int(command, 'command', 1, 3)
    _int(floor_code, 'floor_code', -128, 127)
    _int(direction, 'direction', 0, 2)
    _int(repeat, 'repeat', 0, MAX_DOOR_OPEN_REPEAT if command == CMD_DOOR_OPEN else 0)
    if command == CMD_HALL_CALL and (floor_code == 0 or direction == DIR_NONE):
        raise ValueError('hall call needs a floor and a direction')
    if command == CMD_CAR_FLOOR and (floor_code == 0 or direction != DIR_NONE):
        raise ValueError('car floor command needs a floor and no direction')
    if command == CMD_DOOR_OPEN and (floor_code != 0 or direction != DIR_NONE):
        raise ValueError('door-open command takes no floor or direction')


def encode_command(command, floor_code, direction, sequence, repeat=0):
    """repeat: DOOR_OPEN presses about 1 s apart, done by the car beacon (0/1 = once)."""
    validate_command(command, floor_code, direction, repeat)
    _int(sequence, 'sequence', 0, 255)
    body = _FRAME.pack(PROTOCOL_VERSION, sequence, command, floor_code, direction, repeat, 0, 0)[:7]
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
    return ElevatorEvent(event, floor_code, location, sequence, ack_sequence, rssi)


def encode_event(event):
    """Inverse of decode_event; used by tests and STM32 bench tools."""
    body = _FRAME.pack(PROTOCOL_VERSION, event.sequence, event.event, event.floor_code,
                       event.location, event.ack_sequence, event.rssi_dbm, 0)[:7]
    frame = body + bytes([crc8(body)])
    decode_event(frame)
    return frame


def beacon_node_id(location, floor_code):
    """RF node_id of a beacon: hall 0x40 + floor_code, car E1 0x21."""
    return CAR_NODE_ID if location == LOCATION_CAR else 0x40 + floor_code


def decode_diagnostic(data):
    """Return a CommandStatus, LinkStatus or DeviceStatus; raise ValueError if malformed."""
    data = bytes(data)
    if len(data) != 8:
        raise ValueError('elevator diagnostic frames must have 8 bytes')
    if crc8(data[:7]) != data[7]:
        raise ValueError('elevator diagnostic CRC mismatch')
    if data[0] != PROTOCOL_VERSION:
        raise ValueError(f'unsupported elevator protocol version {data[0]}')
    kind, body = data[1], data[2:7]
    if kind == DIAG_COMMAND_STATUS:
        record = CommandStatus(*body)
        if record.status not in STATUS_NAMES:
            raise ValueError(f'unknown elevator command status {record.status}')
        return record
    if kind == DIAG_LINK_STATUS:
        return LinkStatus(*body)
    if kind == DIAG_DEVICE_STATUS:
        node_id, flags, battery_mv, fw_version = struct.unpack('<BBHB', body)
        return DeviceStatus(node_id, flags, battery_mv, fw_version)
    raise ValueError(f'unknown elevator diagnostic kind {kind}')


def encode_diagnostic(record):
    """Inverse of decode_diagnostic; used by tests and the simulator."""
    if isinstance(record, CommandStatus):
        body = bytes([DIAG_COMMAND_STATUS, record.cmd_seq, record.status, record.device_result,
                      record.target, record.attempts])
    elif isinstance(record, LinkStatus):
        body = bytes([DIAG_LINK_STATUS, record.flags, record.rx_ok, record.rx_bad,
                      record.tx_fail, record.pending])
    elif isinstance(record, DeviceStatus):
        body = bytes([DIAG_DEVICE_STATUS]) + struct.pack(
            '<BBHB', record.node_id, record.flags, record.battery_mv, record.fw_version)
    else:
        raise TypeError(f'not an elevator diagnostic: {record!r}')
    frame = bytes([PROTOCOL_VERSION]) + body
    frame += bytes([crc8(frame)])
    decode_diagnostic(frame)
    return frame
