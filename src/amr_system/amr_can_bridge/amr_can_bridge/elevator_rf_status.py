"""Elevator RF relay health for /elevator/rf_status and /diagnostics, independent of ROS.

Combines beacon PRESENCE (0x310) with the STM32 diagnostics (0x311): link counters,
per-beacon battery/flags and the last command status. Hall beacons are only heard
near their floor, so an old PRESENCE is shown but is not a fault.
"""

from amr_can_bridge.elevator_protocol import (
    DEVICE_BATT_LOW, DEVICE_BUTTON_FAULT, DEVICE_CONFIG_ERR, DEVICE_FLAG_NAMES, EVENT_PRESENCE,
    LINK_RADIO_OK, LINK_RX_OVERFLOW, LINK_TX_TIMEOUT, RESULT_NAMES, STATUS_NAMES,
    CommandStatus, DeviceStatus, LinkStatus, beacon_node_id,
)

OK, WARN, ERROR = 'OK', 'WARN', 'ERROR'
_RANK = {OK: 0, WARN: 1, ERROR: 2}


def beacon_label(node_id):
    """'E1' for car beacons (0x20 + n), '4F' / 'B1' for hall beacons (0x40 + floor)."""
    if 0x20 < node_id < 0x30:
        return f'E{node_id - 0x20}'
    code = node_id - 0x40
    return f'{code}F' if code > 0 else f'B{-code}'


def flag_names(flags):
    return [name for mask, name in DEVICE_FLAG_NAMES.items() if flags & mask]


def _age(now, then):
    return None if then is None else round(max(0., now - then), 2)


class RfStatusBoard:
    def __init__(self, link_timeout=3.0):
        if not 0 < link_timeout < float('inf'):
            raise ValueError('link_timeout must be finite and positive')
        self.link_timeout = link_timeout
        self.link = None
        self.link_at = None
        self.beacons = {}
        self.last_command = None
        self.last_command_at = None
        self.changed = False  # Something worth publishing at once (not just counters).

    def _beacon(self, node_id):
        if node_id not in self.beacons:
            car = 0x20 < node_id < 0x30
            self.beacons[node_id] = {
                'node_id': node_id, 'label': beacon_label(node_id),
                'role': 'car' if car else 'hall', 'floor_code': 0 if car else node_id - 0x40,
                'presence_at': None, 'rssi_dbm': None, 'sequence': None, 'presence_count': 0,
                'device_at': None, 'flags': None, 'battery_mv': None, 'fw_version': None,
            }
            self.changed = True
        return self.beacons[node_id]

    def event(self, event, now):
        if event.event != EVENT_PRESENCE:
            return
        entry = self._beacon(beacon_node_id(event.location, event.floor_code))
        entry.update(presence_at=now, rssi_dbm=event.rssi_dbm, sequence=event.sequence)
        entry['presence_count'] += 1

    def diagnostic(self, record, now):
        if isinstance(record, LinkStatus):
            if self.link is None or self.link.flags != record.flags:
                self.changed = True
            self.link, self.link_at = record, now
        elif isinstance(record, DeviceStatus):
            entry = self._beacon(record.node_id)
            if entry['flags'] != record.flags:
                self.changed = True
            entry.update(device_at=now, flags=record.flags, battery_mv=record.battery_mv,
                         fw_version=record.fw_version)
        elif isinstance(record, CommandStatus):
            self.last_command, self.last_command_at = record, now
            self.changed = True

    def problems(self, now, live):
        """[(level, text)]; the overall level is the worst one."""
        found = []
        if live and (self.link_at is None or now - self.link_at > self.link_timeout):
            found.append((WARN, '하위제어기 RF 링크 상태(0x311)가 수신되지 않습니다.'))
        elif self.link is not None and not self.link.flags & LINK_RADIO_OK:
            found.append((ERROR, 'nRF24 레지스터 검증 실패: 하위제어기 RF 모듈을 확인하세요.'))
        for entry in sorted(self.beacons.values(), key=lambda e: e['node_id']):
            flags = entry['flags'] or 0
            if flags & (DEVICE_CONFIG_ERR | DEVICE_BUTTON_FAULT):
                names = [n for n in flag_names(flags) if n in ('CONFIG_ERR', 'BUTTON_FAULT')]
                found.append((ERROR, f'{entry["label"]} 비콘 고장: {", ".join(names)}'))
            elif flags & DEVICE_BATT_LOW:
                found.append((WARN, f'{entry["label"]} 비콘 배터리 부족'
                                    + (f' ({entry["battery_mv"]} mV)' if entry['battery_mv'] else '')))
        return found

    def level(self, now, live):
        return max((level for level, _ in self.problems(now, live)), key=_RANK.get, default=OK)

    def report(self, now, live):
        link = None
        if self.link is not None:
            link = {
                'radio_ok': bool(self.link.flags & LINK_RADIO_OK),
                'rx_overflow': bool(self.link.flags & LINK_RX_OVERFLOW),
                'tx_timeout': bool(self.link.flags & LINK_TX_TIMEOUT),
                'rx_ok': self.link.rx_ok, 'rx_bad': self.link.rx_bad,
                'tx_fail': self.link.tx_fail, 'pending': self.link.pending,
                'age': _age(now, self.link_at),
            }
        command = None
        if self.last_command is not None:
            record = self.last_command
            command = {
                'cmd_seq': record.cmd_seq,
                'status': STATUS_NAMES.get(record.status, str(record.status)),
                'device_result': RESULT_NAMES.get(record.device_result,
                                                  f'0x{record.device_result:02X}'),
                'target': record.target,
                'target_label': beacon_label(record.target) if record.target else '',
                'attempts': record.attempts, 'age': _age(now, self.last_command_at),
            }
        beacons = []
        for entry in sorted(self.beacons.values(),
                            key=lambda e: (e['role'] != 'car', e['floor_code'])):
            beacons.append({
                'node_id': entry['node_id'], 'label': entry['label'], 'role': entry['role'],
                'floor_code': entry['floor_code'],
                'presence_age': _age(now, entry['presence_at']),
                'presence_count': entry['presence_count'],
                'rssi_dbm': entry['rssi_dbm'], 'sequence': entry['sequence'],
                'battery_mv': entry['battery_mv'],
                'flags': None if entry['flags'] is None else flag_names(entry['flags']),
                'fw_version': entry['fw_version'], 'device_age': _age(now, entry['device_at']),
            })
        problems = self.problems(now, live)
        return {
            'live': live, 'link': link, 'beacons': beacons, 'last_command': command,
            'level': max((level for level, _ in problems), key=_RANK.get, default=OK),
            'problems': [text for _, text in problems],
        }
