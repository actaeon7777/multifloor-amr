"""Offline-capable HTTP UI. ROS commands are executed only on the ROS executor."""
import argparse
from array import array
from collections import OrderedDict, deque
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import math
import os
from pathlib import Path
import queue
import re
import secrets
import signal
import threading
import time
from urllib.parse import urlsplit
from amr_control.time_guard import HealthLease, SessionOrder

from .geometry import Grid, validate_pose
from .telemetry import DisplayTelemetry

LOCALIZATION_STATES = {'WAITING_DATA', 'SEARCHING', 'VERIFYING', 'WAITING_INITIAL_POSE',
                       'STARTING_NAVIGATION', 'READY', 'FAILED', 'SUSPENDED'}
ACTIVE_NAVIGATION = {'WAITING_NAV2', 'RUNNING', 'STOPPING', 'MANEUVER'}
FLOOR_MISSION_ACTIVE = {'NAV_TO_HALL', 'CALL_ELEVATOR', 'WAIT_DOOR_OPEN', 'BOARDING', 'BACKING_OUT',
                        'IN_CAR_ALIGN', 'SELECT_FLOOR', 'RIDING', 'SWITCH_MAP', 'ALIGHTING',
                        'RETURN_TO_CAR', 'RELOCALIZE', 'NAV_TO_GOAL'}
ACTIVE_GOALS = {'SENDING', 'ACCEPTED', 'CANCELING'}
TERMINAL_GOALS = {'SUCCEEDED', 'FAILED', 'REJECTED', 'CANCELED'}
KIOSK_STATE_KEYS = frozenset((
    'preview', 'connection_epoch', 'mode', 'navigation', 'action_ready', 'floor_action_ready',
    'floor', 'floor_mission', 'goal', 'notice', 'pending_request', 'localization',
    'localization_reset_pending', 'robot', 'global_path', 'tf_error', 'estop', 'telemetry',
    'acknowledged_request', 'kiosk', 'map_version', 'hold', 'task'))
ESTOP_STATES = {'UNKNOWN', 'ESTOP', 'ARMING', 'READY', 'BYPASS'}
ESTOP_STM32_KEYS = ('raw', 'debounced', 'latched', 'inhibit', 'odrv_idle', 'bus_off', 'odrv_hb_ok',
                    'drive_enabled', 'stopped', 'trip_count', 'release_count', 'reset_rejected', 'sequence')
# /elevator/rf_status is published at 1 Hz (elevator_can_link); allow two misses.
ELEVATOR_RF_TIMEOUT = 3.
RF_LEVELS = {'OK', 'WARN', 'ERROR'}
RF_LINK_KEYS = {'radio_ok': bool, 'rx_overflow': bool, 'tx_timeout': bool, 'rx_ok': int,
                'rx_bad': int, 'tx_fail': int, 'pending': int}
RF_BEACON_KEYS = {'node_id': int, 'label': str, 'role': str, 'floor_code': int,
                  'presence_age': (int, float), 'rssi_dbm': int, 'battery_mv': int,
                  'fw_version': int, 'device_age': (int, float)}
# Floor transit view: beacon reception timeline (1 s bins) and the mission event log.
TRANSIT_WINDOW = 60
TRANSIT_EVENTS = 60
# /elevator/door follows the scan (~10 Hz); a door state must hold this long to be logged.
DOOR_TIMEOUT = 1.
DOOR_SETTLE = .5
MISSION_TEXT_KEYS = ('phase', 'detail', 'goal_floor', 'goal_room', 'start_floor', 'current_floor', 'confidence')
COMMAND_STATUS_NAMES = {1: 'DELIVERED', 2: 'COMPLETED', 3: 'DEVICE_REJECTED', 4: 'RF_TIMEOUT',
                        5: 'NO_RESULT', 6: 'NO_ROUTE', 7: 'BAD_FRAME', 8: 'REPLACED'}
DEVICE_RESULT_NAMES = {0x00: 'PRESSED', 0x01: 'DUPLICATE', 0x81: 'WRONG_DEVICE', 0x82: 'UNSUPPORTED',
                       0x83: 'BUTTON_FAULT', 0x84: 'BAD_REQUEST', 0xFF: 'NONE'}


def delivery_announcement(goal, before):
    """/robot/announcement key for a kiosk delivery (a goal with a room) leaving ``before``."""
    status = goal.get('status')
    if goal.get('origin') == 'return_home' or not goal.get('room') or status == before:
        return None
    if status == 'ACCEPTED' and before == 'SENDING':
        return 'delivery_start'
    if status == 'SUCCEEDED':
        return 'room_arrival'
    if status == 'CANCELED':
        return 'delivery_cancel'
    # A floor_goal failure is already announced by the elevator mission; an E-stop or LB
    # takeover (navigation_manager ESTOP_DETAIL / OVERRIDE_DETAIL) is not a robot error.
    if (status in ('FAILED', 'REJECTED') and goal.get('kind') == 'goal'
            and not str(goal.get('error_message', '')).startswith(('E-stop', '수동 조작', '관리자'))):
        return 'robot_error'
    return None


# A stop requested while the robot crosses the elevator door, rides, or has not yet
# relocalised on the new floor would leave it in the doorway or the car; the hold is
# recorded as pending and engages once the mission is back in a corridor (NAV_TO_GOAL
# or finished). Hall phases before boarding stop at once.
HOLD_DEFER_PHASES = frozenset(('BOARDING', 'BACKING_OUT', 'IN_CAR_ALIGN', 'SELECT_FLOOR', 'RIDING',
                               'SWITCH_MAP', 'ALIGHTING', 'RETURN_TO_CAR', 'RELOCALIZE'))


def task_summary(state):
    """One Korean task line, shared by the console and kiosk snapshots."""
    estop = state.get('estop') or {}
    if estop.get('error') or estop.get('state') == 'ESTOP':
        return '비상 정지'
    if (state.get('hold') or {}).get('held'):
        return '관리자 정지'
    if state.get('mode') == 'MANUAL':
        return '수동 조작'
    goal = state.get('goal') or {}
    status, room = goal.get('status'), goal.get('room')
    if status in ACTIVE_GOALS:
        if goal.get('origin') == 'return_home':
            text = f'복귀 중 ({room}호)'
        else:
            text = f'{room}호 배송 중' if room else '지정 위치로 이동 중'
        phase = (state.get('floor_mission') or {}).get('phase') or goal.get('phase')
        labels = {
            'NAV_TO_HALL': '엘리베이터 홀 이동', 'CALL_ELEVATOR': '호출', 'WAIT_DOOR_OPEN': '호출',
            'BOARDING': '탑승', 'BACKING_OUT': '탑승', 'IN_CAR_ALIGN': '탑승', 'SELECT_FLOOR': '탑승',
            'RIDING': '이동', 'SWITCH_MAP': '이동', 'ALIGHTING': '하차', 'RETURN_TO_CAR': '하차',
            'RELOCALIZE': '하차', 'NAV_TO_GOAL': '목적지 이동'}
        if phase in labels:
            text += ' · ' + labels[phase]
        if (state.get('hold') or {}).get('pending'):
            text += ' · 하차 후 정지 예정'
        return text + (' (취소 중)' if status == 'CANCELING' else '')
    if (room and status in TERMINAL_GOALS and goal.get('origin') != 'return_home'
            and (not state.get('acknowledged_request')
                 or goal.get('request_id') != state.get('acknowledged_request'))):
        if status == 'SUCCEEDED':
            return f'{room}호 도착 · 수령 대기'
        return '배송 취소됨' if status == 'CANCELED' else f'{room}호 배송 실패'
    if not (state.get('localization') or {}).get('ready'):
        return '위치 확인 필요'
    return '대기'


def _typed(source, keys):
    """Copy whitelisted keys whose value has the expected type (bool is not an int here)."""
    result = {}
    for key, kind in keys.items():
        value = source.get(key)
        if value is None or (isinstance(value, bool) and kind is not bool):
            continue
        if isinstance(value, kind) and (not isinstance(value, float) or math.isfinite(value)):
            result[key] = value[:40] if isinstance(value, str) else value
    return result


CAMERA_MAX_AGE = 3.  # seconds; older frames are not served or relayed


def jpeg_size(data):
    """(width, height) from the JPEG SOF marker; raises ValueError for anything else.

    A marker walk over a few header segments: microseconds, no decoding.
    """
    data = bytes(data)
    if len(data) < 4 or data[:3] != b'\xff\xd8\xff':
        raise ValueError('not a JPEG')
    index = 2
    while index + 9 <= len(data):
        if data[index] != 0xFF:
            raise ValueError('corrupt JPEG header')
        marker = data[index + 1]
        if marker == 0xFF:  # fill byte
            index += 1
            continue
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            index += 2
            continue
        length = int.from_bytes(data[index + 2:index + 4], 'big')
        if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
            height = int.from_bytes(data[index + 5:index + 7], 'big')
            width = int.from_bytes(data[index + 7:index + 9], 'big')
            if width <= 0 or height <= 0:
                raise ValueError('corrupt JPEG size')
            return width, height
        if marker == 0xDA or length < 2:
            break
        index += 2 + length
    raise ValueError('JPEG size not found')


class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def kiosk_launcher_pid():
    """PID of the running scripts/kiosk/launch.sh (it writes the file), else None."""
    path = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp')) / f'amr-kiosk-{os.getuid()}.pid'
    try:
        pid = int(path.read_text())
        argv = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
    except (OSError, ValueError):
        return None
    # A stale file can name a reused PID; signal only the launcher itself.
    return pid if any(arg.endswith(b'/kiosk/launch.sh') for arg in argv) else None


def kiosk_browser_pid(proc=Path('/proc')):
    """Main process of a kiosk Chromium (``--class=amr-kiosk``) left without its launcher."""
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            # Chromium rewrites its process title: one space-separated string.
            argv = (entry/'cmdline').read_bytes().replace(b'\0', b' ').split()
        except OSError:
            continue
        if argv and argv[0].endswith(b'/chrome') and b'--class=amr-kiosk' in argv and not any(arg.startswith(b'--type=') for arg in argv):
            return int(entry.name)
    return None


def json_bytes(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode()


class ScanPoints(list):
    """Replace as a whole: snapshots share both the points and their JSON fragment.

    The fragment is encoded on the first request for this scan, not on arrival:
    scans come at 10 Hz but the page polls at <= 5 Hz, and encoding every scan
    cost more than the per-request encoding it replaced (bag replay 2026-10-25).
    """
    def __init__(self, points):
        super().__init__(points)
        self._json = None

    @property
    def json(self):
        # Two request threads may race here; both produce the same bytes.
        if self._json is None:
            self._json = json_bytes(self)
        return self._json


def encode_snapshot(snapshot):
    scan = snapshot.get('scan')
    if not scan or not isinstance(scan.get('points'), ScanPoints):
        return json_bytes(snapshot)
    # Encode only changing metadata; never walk the cached point list here.
    scan_meta = {key: value for key, value in scan.items() if key != 'points'}
    scan_json = json_bytes(scan_meta)[:-1] + b',"points":' + scan['points'].json + b'}'
    rest = {key: value for key, value in snapshot.items() if key != 'scan'}
    return json_bytes(rest)[:-1] + b',"scan":' + scan_json + b'}'


class SharedState:
    def __init__(self, preview=False, kiosk_pin=''):
        self.lock = threading.RLock()
        self.commands = queue.Queue(maxsize=32)
        # Safety commands must not be refused by the ordinary bounded queue.
        self.operator_commands = deque()
        self.hold_sequence = 0
        self.cancel_goal_ids = set()
        self.pending_return = None
        self.announce = None  # ros_bridge: announce(key, goal) publishes /robot/announcement
        self.grids = {}
        self.grid_versions = {}
        self.requests = OrderedDict()
        self.kiosk_pin = kiosk_pin
        self.kiosk_tokens = OrderedDict()
        self.kiosk_failures = deque()
        self.kiosk_blocked_until = -math.inf
        self.localization_reset_baseline = None
        self.localization_generation = -1
        self.localization_stamp = -math.inf
        self.localization_session = None
        self.localization_order = SessionOrder()
        self.time_health = HealthLease()
        self.require_time_health = False
        # /estop/status from cmd_vel_to_can; a stale or missing lease blocks goals.
        self.estop = HealthLease()
        self.require_estop = False
        # Elevator RF relay health (/elevator/rf_status) and the last command status.
        self.elevator_rf = None
        self.elevator_rf_received = -math.inf
        self.elevator_command = None
        # Floor transit view (/elevator/command, /elevator/beacon, /elevator/door, /floor/events).
        self.transit_events = deque(maxlen=TRANSIT_EVENTS)
        self.beacon_history = {}  # ('car', 0) | ('hall', floor_code) -> deque[(monotonic, rssi)]
        self.door = None
        self.door_candidate = (None, -math.inf)
        self.door_logged = None
        self.sent_command = None
        self.mission_phases = []
        self.mission_received = (-math.inf, -math.inf)  # (phase change, last status) monotonic
        self.connection_epoch = 0
        self.telemetry = DisplayTelemetry()
        # Multi-floor (floors.yaml); None when the elevator packages are absent.
        self.building = None
        self.simulation = False
        self.floor_grids = {}
        self.floor_errors = {}
        self.scan_stamp = None
        self.last_snapshot_at = float('-inf')
        # Latest JPEG from the depth camera's color stream (bytes, monotonic, width, height).
        # Encoded by the camera driver; this process never decodes or re-encodes images.
        self.camera = None
        self.state = dict(hold=dict(held=False, since=None, reason='', pending=False), preview=preview, floor=None, floor_mission=None, floor_action_ready=False, navigation='UNKNOWN', mode='UNKNOWN',
                          action_ready=False, robot=None, amcl=None, scan=None,
                          global_path=None, local_path=None, footprint=None,
                          goal=None, notice=None, velocity=None, tf_error=None, pending_request=None,
                          localization=None, localization_reset_pending=False, acknowledged_request=None)

    def unlock_kiosk(self, payload):
        with self.lock:
            now = time.monotonic()
            if now < self.kiosk_blocked_until:
                raise ApiError(429, '잠시 후 다시 시도해 주세요.')
            while self.kiosk_failures and now-self.kiosk_failures[0] >= 60:
                self.kiosk_failures.popleft()
            pin = payload.get('pin')
            if self.kiosk_pin and (not isinstance(pin, str) or not secrets.compare_digest(
                    pin.encode('utf-8'), self.kiosk_pin.encode('utf-8'))):
                self.kiosk_failures.append(now)
                if len(self.kiosk_failures) >= 5:
                    self.kiosk_blocked_until = now + 30
                    self.kiosk_failures.clear()
                    raise ApiError(429, '잠시 후 다시 시도해 주세요.')
                raise ApiError(403, 'PIN이 맞지 않습니다.')
            self._prune_kiosk_tokens(now)
            token = secrets.token_hex(16)
            self.kiosk_tokens[token] = now + 600
            while len(self.kiosk_tokens) > 8:
                self.kiosk_tokens.popitem(last=False)
            return dict(token=token, expires_in=600)

    def _prune_kiosk_tokens(self, now):
        for token, expires in list(self.kiosk_tokens.items()):
            if now >= expires:
                del self.kiosk_tokens[token]

    def check_kiosk_request(self, kind, payload, token):
        # UX guard, not authentication: kiosk access is local (localhost), and
        # remote access is token-checked by the central server. The client header
        # is self-declared; deployments must restrict direct network access.
        with self.lock:
            if kind == 'goal':
                if self.mission_active():
                    raise ApiError(409, '진행 중인 층간 이동이 있습니다.')
                floor, room = payload.get('floor'), payload.get('room')
                configured = (self.building.floors.get(floor) if self.building is not None
                              and isinstance(floor, str) else None)
                destination = configured.rooms.get(room) if configured and isinstance(room, str) else None
                try:
                    valid = destination is not None and all(
                        type(payload.get(key)) in (int, float) and math.isfinite(payload[key])
                        and abs(payload[key]-expected) <= 1e-6
                        for key, expected in zip(('x', 'y', 'yaw'), destination.goal))
                except OverflowError:
                    valid = False
                if not valid:
                    raise ApiError(403, '키오스크에서는 등록된 호실로만 보낼 수 있습니다.')
            elif kind in ('initialpose', 'relocalize', 'set_floor', 'kiosk/exit'):
                self._prune_kiosk_tokens(time.monotonic())
                if self.kiosk_pin and token not in self.kiosk_tokens:
                    raise ApiError(403, '관리 모드 잠금을 먼저 해제하세요.')
            elif kind not in ('cancel', 'acknowledge', 'kiosk/unlock'):
                raise ApiError(403, '키오스크에서 허용하지 않는 명령입니다.')

    def acknowledge(self, payload):
        with self.lock:
            request_id = payload.get('request_id')
            goal = self.state.get('goal') or {}
            if (not isinstance(request_id, str) or not request_id
                    or request_id != goal.get('request_id') or goal.get('status') not in TERMINAL_GOALS):
                raise ApiError(409, '현재 완료된 배송만 확인할 수 있습니다.')
            first = self.state.get('acknowledged_request') != request_id
            self.state['acknowledged_request'] = request_id
            if (first and self.announce and goal.get('origin') != 'return_home'
                    and goal.get('room') and goal.get('status') == 'SUCCEEDED'):
                self.announce('received', goal)
            return dict(acknowledged=request_id)

    def update_camera(self, data, fmt=''):
        """Store one camera JPEG; other encodings and truncated frames are dropped."""
        try:
            size = jpeg_size(data)
        except ValueError:
            return False
        with self.lock:
            self.camera = (bytes(data), time.monotonic(), size[0], size[1])
        return True

    def camera_frame(self, max_age=CAMERA_MAX_AGE):
        """(jpeg, age) of a frame newer than max_age, else None."""
        with self.lock:
            frame = self.camera
        if frame is None:
            return None
        age = time.monotonic() - frame[1]
        return None if age > max_age else (frame[0], age)

    def camera_snapshot(self, now):
        with self.lock:
            frame = self.camera
        if frame is None:
            return None
        return dict(age=max(0., now - frame[1]), width=frame[2], height=frame[3], bytes=len(frame[0]))

    def update_localization(self, text):
        """Only the supervisor can grant readiness; malformed input fails closed."""
        try:
            value = json.loads(text, parse_constant=lambda _s: None)
            if (not isinstance(value, dict) or value.get('state') not in LOCALIZATION_STATES
                    or not isinstance(value.get('ready'), bool)
                    or not isinstance(value.get('detail'), str)):
                raise ValueError('Invalid localization status')
            status = dict(state=value['state'], ready=value['ready'] and value['state'] == 'READY',
                          detail=value['detail'][:1000], source=value.get('source', 'unknown'))
            if status['source'] not in ('auto', 'manual', 'unknown'):
                status['source'] = 'unknown'
            warning = value.get('tracking_warning', '')
            prior_verified = value.get('prior_verified', False)
            bad_samples = value.get('tracking_bad_samples', 0)
            bad_duration = value.get('tracking_bad_duration', 0.0)
            if (not isinstance(warning, str) or not isinstance(prior_verified, bool)
                    or type(bad_samples) is not int or bad_samples < 0
                    or type(bad_duration) not in (int, float)
                    or not math.isfinite(bad_duration) or bad_duration < 0):
                raise ValueError('Invalid localization tracking diagnostics')
            status.update(tracking_warning=warning[:1000], prior_verified=prior_verified,
                          tracking_bad_samples=bad_samples, tracking_bad_duration=bad_duration)
            # Advisory only: a malformed saved pose is dropped, never trusted.
            last = value.get('last_pose')
            keys = ('x', 'y', 'yaw', 'saved_at')
            status['last_pose'] = ({k: float(last[k]) for k in keys} if isinstance(last, dict) and all(
                type(last.get(k)) in (int, float) and math.isfinite(last[k]) for k in keys) else None)
            stamp, generation, session = value.get('stamp'), value.get('generation'), value.get('session')
            if (not isinstance(stamp, (int, float)) or isinstance(stamp, bool) or not math.isfinite(stamp)
                    or not isinstance(generation, int) or isinstance(generation, bool)
                    or generation < 0
                    or not isinstance(session, str) or not session):
                raise ValueError('Missing status freshness metadata')
            status.update(stamp=stamp, generation=generation, session=session)
        except (ValueError, TypeError):
            status = dict(state='FAILED', ready=False, source='unknown',
                          detail='위치 확인 상태를 읽을 수 없습니다.')
        with self.lock:
            if 'generation' in status:
                # Different DDS callbacks can deliver an older heartbeat after
                # a new initialization has already revoked readiness. Preserve
                # this watermark even when malformed input fails closed.
                if not -1 <= time.time()-status['stamp'] < 2:
                    self.state['localization'] = None
                    return
                old_session = self.localization_order.session
                if not self.localization_order.accept(status['session'], status['generation'], status['stamp']):
                    return
                if old_session is not None and status['session'] != old_session:
                    self.reset_connection('제어기가 재시작되었습니다. 새 위치 데이터를 기다립니다.')
                self.localization_generation = status['generation']
                self.localization_stamp = status['stamp']
                self.localization_session = status['session']
            # The supervisor advances its generation when it consumes a new pose
            # or begins global search. Old queued READY/non-ready messages cannot
            # release a reset requested by this UI.
            identity = (status.get('session'), status.get('generation'))
            if self.state['localization_reset_pending']:
                baseline = self.localization_reset_baseline
                if (identity[0] is not None and
                        ((baseline is not None and (identity[0] != baseline[0] or identity[1] > baseline[1]))
                         or (baseline is None and status['state'] != 'READY'))):
                    self.state['localization_reset_pending'] = False
            status['received_at'] = time.monotonic()
            self.state['localization'] = status

    def update_estop(self, text):
        with self.lock:
            accepted = self.estop.receive(text)
            if accepted and self.estop.status.get('state') not in ESTOP_STATES:
                self.estop.status = {}
            return accepted

    def update_elevator_rf(self, text):
        """Keep only typed, whitelisted fields; anything malformed is dropped."""
        try:
            value = json.loads(text, parse_constant=lambda _s: None)
            if not isinstance(value, dict) or value.get('level') not in RF_LEVELS:
                raise ValueError('invalid RF status')
            link = value.get('link')
            beacons = value.get('beacons') if isinstance(value.get('beacons'), list) else []
            status = dict(
                level=value['level'], live=value.get('live') is True,
                problems=[str(p)[:200] for p in value.get('problems', [])[:16]
                          if isinstance(p, str)] if isinstance(value.get('problems'), list) else [],
                link=dict(_typed(link, RF_LINK_KEYS), age=link.get('age') if type(link.get('age')) in (int, float) else None)
                if isinstance(link, dict) else None,
                beacons=[])
            for beacon in beacons[:32]:
                if not isinstance(beacon, dict):
                    continue
                entry = _typed(beacon, RF_BEACON_KEYS)
                flags = beacon.get('flags')
                entry['flags'] = ([str(f)[:20] for f in flags[:8]] if isinstance(flags, list) else None)
                if 'node_id' in entry and 'label' in entry:
                    status['beacons'].append(entry)
        except (ValueError, TypeError, AttributeError):
            return False
        with self.lock:
            self.elevator_rf, self.elevator_rf_received = status, time.monotonic()
        return True

    def update_elevator_command(self, cmd_seq, status, device_result, target, attempts, source):
        with self.lock:
            self.elevator_command = dict(
                cmd_seq=cmd_seq, status=COMMAND_STATUS_NAMES.get(status, str(status)),
                device_result=DEVICE_RESULT_NAMES.get(device_result, f'0x{device_result:02X}'),
                target=target, attempts=attempts, source=source[:8], received_at=time.monotonic())
            name, result = self.elevator_command['status'], self.elevator_command['device_result']
            sent = self.sent_command
            if sent is not None and cmd_seq in sent['sequences']:
                sent.update(status=name, device_result=result)
            self.log_transit('status', cmd_seq=cmd_seq, status=name, device_result=result, target=target)

    def log_transit(self, kind, **fields):
        self.transit_events.append(dict(t=time.time(), kind=kind, **fields))

    def mission_active(self):
        return (self.state.get('floor_mission') or {}).get('phase') in FLOOR_MISSION_ACTIVE

    def update_floor_mission(self, text):
        """/floor_mission/status; only typed, whitelisted fields reach the browser."""
        try:
            value = json.loads(text, parse_constant=lambda _s: None)
        except ValueError:
            return False
        if not isinstance(value, dict) or not isinstance(value.get('phase'), str):
            return False
        mission = {k: value[k][:300] if isinstance(value.get(k), str) else '' for k in MISSION_TEXT_KEYS}
        for key in ('zone', 'door'):
            mission[key] = value[key] if type(value.get(key)) is int else None
        number = lambda v: type(v) in (int, float) and math.isfinite(v)
        mission['counters'] = {str(k)[:40]: v for k, v in (value.get('counters') or {}).items()
                               if type(v) is int} if isinstance(value.get('counters'), dict) else {}
        mission['timings'] = {str(k)[:40]: float(v) for k, v in (value.get('timings') or {}).items()
                              if number(v)} if isinstance(value.get('timings'), dict) else {}
        mission['elapsed'] = float(value['elapsed']) if number(value.get('elapsed')) else None
        now = time.monotonic()
        with self.lock:
            old = self.state.get('floor_mission') or {}
            phase = mission['phase']
            if phase != old.get('phase'):
                if phase in FLOOR_MISSION_ACTIVE and old.get('phase') not in FLOOR_MISSION_ACTIVE:
                    # A new mission: start a fresh log and button history.
                    self.transit_events.clear()
                    self.mission_phases = []
                    self.sent_command = self.door_logged = None
                self.mission_phases = (self.mission_phases + [phase])[-40:]
                self.mission_received = (now, now)
                self.log_transit('phase', phase=phase, detail=mission['detail'])
            else:
                self.mission_received = (self.mission_received[0], now)
            self.state['floor_mission'] = mission
        return True

    def update_sent_command(self, command, floor_code, direction, sequence, repeat):
        """An ElevatorCommand from the mission; a resend of the same press counts as a retry."""
        now = time.monotonic()
        key = (command, floor_code, direction, repeat)
        with self.lock:
            last = self.sent_command
            retry = last is not None and last['key'] == key and last['acked_at'] is None
            self.sent_command = dict(
                key=key, command=command, floor_code=floor_code, direction=direction, repeat=repeat,
                sequence=sequence, sequences=(last['sequences'] if retry else set()) | {sequence},
                attempt=last['attempt'] + 1 if retry else 1, sent_at=now, acked_at=None,
                status=None, device_result=None)
            self.log_transit('command', command=command, floor_code=floor_code, direction=direction,
                             repeat=repeat, sequence=sequence, attempt=self.sent_command['attempt'])

    def update_beacon(self, event, location, floor_code, ack_sequence, rssi):
        now = time.monotonic()
        with self.lock:
            if event == 1:  # PRESENCE
                key = ('car', 0) if location == 1 else ('hall', floor_code)
                samples = self.beacon_history.setdefault(key, deque())
                samples.append((now, rssi))
                while samples and now - samples[0][0] > TRANSIT_WINDOW:
                    samples.popleft()
            elif event == 2:  # BUTTON_ACK
                sent = self.sent_command
                if sent is not None and sent['acked_at'] is None and ack_sequence in sent['sequences']:
                    sent['acked_at'] = now
                self.log_transit('ack', location=location, floor_code=floor_code, ack_sequence=ack_sequence)

    def update_door(self, state, open_ratio, valid_beams):
        now = time.monotonic()
        with self.lock:
            self.door = dict(state=state, open_ratio=open_ratio if math.isfinite(open_ratio) else None,
                             valid_beams=valid_beams, received_at=now)
            if state != self.door_candidate[0]:
                self.door_candidate = (state, now)
            elif (state != self.door_logged and now - self.door_candidate[1] >= DOOR_SETTLE
                  and self.mission_active()):
                self.door_logged = state
                self.log_transit('door', state=state)

    def update_floor_event(self, event, from_floor, to_floor, detail):
        with self.lock:
            self.log_transit('floor', event=event[:40], from_floor=from_floor[:16], to_floor=to_floor[:16],
                             detail=detail[:200])

    def transit_snapshot(self, now):
        """The floor transit view; None without floors.yaml."""
        with self.lock:
            if self.building is None:
                return None
            mission = self.state.get('floor_mission')
            mission = dict(mission) if mission else None
            phases, (phase_at, status_at) = list(self.mission_phases), self.mission_received
            history = {key: list(samples) for key, samples in self.beacon_history.items()}
            expected = self.expected_beacons()
            devices = {('car', 0) if b.get('role') == 'car' else ('hall', b.get('floor_code')): b
                       for b in (self.elevator_rf or {}).get('beacons', [])}
            door = dict(self.door) if self.door else None
            sent = dict(self.sent_command) if self.sent_command else None
            events = list(self.transit_events)
        if mission:
            mission['phase_age'] = max(0., now - phase_at) if math.isfinite(phase_at) else None
            if mission.get('elapsed') is not None and mission.get('phase') in FLOOR_MISSION_ACTIVE:
                mission['elapsed'] += max(0., now - status_at)
            mission['phases'] = phases
        if door:
            door['age'] = max(0., now - door.pop('received_at'))
        if sent:
            for name in ('key', 'sequences'):
                sent.pop(name)
            sent['age'] = max(0., now - sent.pop('sent_at'))
            acked = sent.pop('acked_at')
            sent['ack_age'] = None if acked is None else max(0., now - acked)
        rows = [(('car', 0) if row['role'] == 'car' else ('hall', row['floor_code']), row) for row in expected]
        known = {key for key, _row in rows}
        rows += [(key, dict(label=f'{key[1]}F' if key[1] > 0 else f'B{-key[1]}', role='hall',
                            floor_code=key[1], mapped=False))  # Heard but not in floors.yaml.
                 for key in sorted(set(history) - known) if key[0] == 'hall']
        beacons = []
        for key, row in rows:
            samples = [(t, r) for t, r in history.get(key, ()) if 0 <= now - t < TRANSIT_WINDOW]
            bins = [None] * TRANSIT_WINDOW
            for t, rssi in samples:
                index = TRANSIT_WINDOW - 1 - int(now - t)
                bins[index] = rssi if bins[index] is None else max(bins[index], rssi)
            device = devices.get(key, {})
            beacons.append(dict(
                label=row['label'], role=row['role'], floor_code=row.get('floor_code', 0),
                mapped=row['mapped'], presence_age=now - samples[-1][0] if samples else None,
                rssi=samples[-1][1] if samples else None, bins=bins,
                **{k: device[k] for k in ('battery_mv', 'flags', 'device_age') if device.get(k) is not None}))
        return dict(mission=mission, door=door, command=sent, events=events, beacons=beacons,
                    window=TRANSIT_WINDOW, door_timeout=DOOR_TIMEOUT)

    def expected_beacons(self):
        """Every installed beacon: floors with a map, pass floors (no map) and the car."""
        if self.building is None:
            return []
        rows = [dict(label=f.name, floor_code=f.beacon_floor_code, level=f.level, role='hall', mapped=mapped)
                for floors, mapped in ((self.building.floors, True),
                                       (getattr(self.building, 'pass_floors', {}), False))
                for f in floors.values() if isinstance(getattr(f, 'beacon_floor_code', None), int)]
        rows.sort(key=lambda r: r['level'])
        cars = [dict(label=elevator, role='car', mapped=True)
                for elevator in getattr(self.building, 'elevators', {})]
        return cars + rows

    def elevator_snapshot(self, now):
        with self.lock:
            rf, received = deepcopy(self.elevator_rf), self.elevator_rf_received
            command = dict(self.elevator_command) if self.elevator_command else None
            expected = self.expected_beacons()
        if command is not None:
            command['age'] = max(0., now - command.pop('received_at'))
        if rf is None and command is None and not expected:
            return None
        fresh = rf is not None and 0 <= now - received < ELEVATOR_RF_TIMEOUT
        heard = {}
        for beacon in (rf or {}).get('beacons', []):
            key = 'E1' if beacon.get('role') == 'car' else beacon.get('floor_code')
            heard[key] = beacon
        rows = []
        for row in expected:
            beacon = heard.pop('E1' if row['role'] == 'car' else row['floor_code'], None)
            rows.append({**row, **(beacon or {}), 'label': row['label'], 'heard': beacon is not None})
        rows += [{**b, 'mapped': False, 'heard': True} for b in heard.values()]  # Not in floors.yaml.
        return dict(
            received=rf is not None, fresh=fresh, timeout=ELEVATOR_RF_TIMEOUT,
            age=max(0., now - received) if math.isfinite(received) else None,
            live=bool(rf and rf['live']), level=rf['level'] if fresh else None,
            problems=rf['problems'] if fresh else [], link=rf['link'] if fresh else None,
            beacons=rows, last_command=command)

    def estop_live(self, now=None):
        """Latest bridge status while its lease is alive, else None."""
        now = time.monotonic() if now is None else now
        status = self.estop.status
        if status and 0 <= now-self.estop.received < self.estop.timeout:
            return status
        return None

    def estop_error(self, now=None):
        if not self.require_estop or self.estop.healthy(now):
            return None
        status = self.estop_live(now)
        state = status.get('state') if status else None
        if state == 'ESTOP':
            if status.get('reason') in ('status_timeout', 'invalid_status'):
                return 'STM32 E-stop 상태 수신이 끊겨 정지했습니다. CAN 연결을 확인하세요.'
            return 'E-stop이 눌려 있습니다. 버튼을 원위치로 돌리세요.'
        if state == 'ARMING':
            return 'E-stop 해제 후 주행 준비 중입니다. 잠시 후 다시 시도하세요.'
        if state == 'UNKNOWN':
            return 'STM32 E-stop 상태를 기다리고 있습니다.'
        return 'E-stop 상태를 받지 못했습니다. CAN 브릿지 연결을 확인하세요.'

    def reset_connection(self, reason):
        """Invalidate display/request state; never replay a command after restart."""
        with self.lock:
            self.cancel_pending_return(reason)
            self.connection_epoch += 1
            self.scan_stamp = None
            self.telemetry.reset()
            for key in ('robot', 'amcl', 'scan', 'global_path', 'local_path', 'footprint',
                        'velocity', 'localization', 'goal'):
                self.state[key] = None
            self.state.update(action_ready=False, navigation='UNKNOWN', mode='UNKNOWN',
                              localization_reset_pending=False, tf_error=reason)
            for request_id, record in list(self.requests.items()):
                if record['status'] in ('QUEUED', 'SENDING', 'ACCEPTED', 'CANCELING'):
                    self.complete(request_id, 'FAILED', reason)
            self.state['pending_request'] = None
            self.localization_reset_baseline = None
            # The log and beacon timeline stay; live door and button state must be re-heard.
            self.door = self.sent_command = None
            while True:
                try:
                    self.commands.get_nowait()
                except queue.Empty:
                    break
            # Preserve the static map and its version; moving overlays must be
            # reacquired. A newly published map advances the existing version.
            for name in ('local_costmap', 'global_costmap'):
                self.grids.pop(name, None)

    def set_building(self, building, simulation):
        # Load/hash static files before publishing the building. No disk I/O or
        # image conversion may hold the ROS callbacks' shared lock.
        grids, errors = {}, {}
        if building is not None:
            from amr_stack.floor_config import load_occupancy_map
            import hashlib
            for name, floor in building.floors.items():
                try:
                    path = building.map_path(name, simulation)
                    occupancy = load_occupancy_map(path)
                    height, width = occupancy.cells.shape
                    raw = occupancy.cells.tobytes()
                    grid = Grid(width, height, occupancy.resolution, occupancy.origin,
                                array('b', raw))
                    digest = hashlib.sha256(name.encode() + path.read_bytes() + raw)
                    grid.version = int(digest.hexdigest()[:12], 16)
                    grid.floor = floor.name
                    grids[name] = grid
                except Exception as error:  # noqa: BLE001 - one bad floor must not disable the others
                    # Keep the text, not the exception: re-raising one instance
                    # grows its traceback (and pinned frames) on every request.
                    errors[name] = f'{type(error).__name__}: {error}'
        with self.lock:
            self.building, self.simulation = building, bool(simulation)
            self.floor_grids, self.floor_errors = grids, errors

    def floor_grid(self, name):
        """Static maps are prepared by set_building, outside the shared lock."""
        with self.lock:
            if self.building is None:
                raise ApiError(404, '층 설정이 없습니다.')
            if name in self.floor_grids:
                return self.floor_grids[name]
            try:
                self.building.floor(name)
            except ValueError:
                raise ApiError(404, '알 수 없는 층입니다.') from None
            if name in self.floor_errors:
                raise ApiError(503, f'층 지도를 읽지 못했습니다: {self.floor_errors[name]}')
            raise ApiError(404, '층 지도가 없습니다.')

    def current_floor(self):
        floor = self.state.get('floor')
        return floor.get('floor') if floor else None

    def floors_info(self):
        with self.lock:
            if self.building is None:
                return dict(floors=[], default_floor=None, current_floor=None, simulation=self.simulation, home=None)
            floors = []
            for floor in sorted(self.building.floors.values(), key=lambda f: f.level):
                try:
                    layer = self.floor_grid(floor.name).metadata()
                except (ApiError, OSError, ValueError):
                    layer = None
                floors.append(dict(
                    name=floor.name, level=floor.level, real_enabled=floor.real_enabled,
                    provisional=floor.provisional_copy_of is not None,
                    selectable=self.simulation or floor.real_enabled, map=layer,
                    rooms=[dict(id=r.room_id, x=r.goal[0], y=r.goal[1], yaw=r.goal[2])
                           for r in sorted(floor.rooms.values(), key=lambda r: r.room_id)]))
            return dict(floors=floors, default_floor=self.building.default_floor,
                        current_floor=self.current_floor(), simulation=self.simulation,
                        home=(dict(floor=self.building.home.floor, room=self.building.home.room)
                              if getattr(self.building, 'home', None) else None))

    def begin_localization_reset(self):
        with self.lock:
            self.localization_reset_baseline = ((self.localization_session, self.localization_generation)
                                                if self.localization_generation >= 0 else None)
            self.state['localization_reset_pending'] = True

    @staticmethod
    def localization_fresh(localization):
        return (localization is not None and time.monotonic()-localization['received_at'] < 2
                and -1 <= time.time()-localization.get('stamp', 0) < 2)

    def readiness_error(self, kind, request_id=None):
        """Shared admission check, repeated on the ROS executor before side effects."""
        with self.lock:
            if self.state['preview']:
                if kind in ('goal', 'floor_goal') and self.state['hold']['held']:
                    return '관리자가 원격으로 정지했습니다. 중앙 관제에서 해제해야 출발할 수 있습니다.'
                return None
            pending = self.state['pending_request']
            if pending and pending['request_id'] != request_id:
                return '이전 요청을 처리 중입니다. 잠시 기다려 주세요.'
            if kind in ('goal', 'floor_goal'):
                estop = self.estop_error()
                if estop:
                    return estop
                if self.state['hold']['held']:
                    return '관리자가 원격으로 정지했습니다. 중앙 관제에서 해제해야 출발할 수 있습니다.'
                if self.state['mode'] == 'MANUAL':
                    return ('수동 조작(LB) 중이라 자율주행 명령을 받을 수 없습니다. '
                            'LB를 놓은 뒤 목적지를 다시 지정하세요.')
                if self.state['mode'] != 'AUTO':
                    return '제어 모드 수신을 기다리고 있습니다.'
                if self.require_time_health and not self.time_health.healthy():
                    return '로봇의 시간·센서 상태를 확인하고 있습니다. ' + self.time_health.status.get('detail', '')
                localization = self.state['localization']
                if (self.state['localization_reset_pending'] or not localization
                        or not self.localization_fresh(localization)
                        or not localization['ready']):
                    return '위치 확인과 주행 준비가 완료될 때까지 기다려 주세요.'
                if not self.state['action_ready']:
                    return '내비게이션 연결을 기다리고 있습니다.'
                robot = self.state['robot']
                if (not robot or time.monotonic()-robot['received_at'] >= 2
                        or self.state['tf_error']):
                    return '로봇 위치를 확인할 수 없습니다. 위치 확인 상태를 확인해 주세요.'
                if (self.state.get('goal') or {}).get('status') in ('SENDING', 'CANCELING'):
                    return '이전 요청을 처리 중입니다. 잠시 기다려 주세요.'
            if kind == 'floor_goal':
                floor = self.state.get('floor')
                if not self.state.get('floor_action_ready'):
                    return '층간 이동 관리자에 연결되지 않았습니다.'
                if (not floor or floor.get('zone') != 1
                        or floor.get('confidence') not in ('BEACON', 'CONFIRMED')):
                    return '현재 층이 확인되지 않았습니다. 엘리베이터 밖에서 층 확인을 기다려 주세요.'
                if (self.state.get('floor_mission') or {}).get('phase') in FLOOR_MISSION_ACTIVE:
                    return '진행 중인 층간 이동이 있습니다.'
            elif kind in ('initialpose', 'relocalize', 'set_floor'):
                if kind == 'set_floor' and self.mission_active():
                    return '진행 중인 층간 이동이 있습니다.'
                if (self.state['navigation'] in ACTIVE_NAVIGATION
                        or (self.state.get('goal') or {}).get('status') in ACTIVE_GOALS):
                    return '진행 중인 목표를 취소한 뒤 위치를 다시 설정해 주세요.'
                velocity = self.state['velocity']
                if not velocity or time.monotonic()-velocity['received_at'] >= 2:
                    return '정지 상태를 확인할 수 없습니다. 속도 데이터 수신을 기다려 주세요.'
                if abs(velocity['linear']) > .02 or abs(velocity['angular']) > .03:
                    return '로봇이 완전히 정지한 뒤 위치를 다시 설정해 주세요.'
            elif kind == 'cancel':
                goal = self.state.get('goal')
                if not goal or goal['status'] != 'ACCEPTED':
                    return '이 웹에서 실행한 취소 가능한 목표가 없습니다.'
            return None

    def layer_png(self, name):
        """PNG bytes and version of a costmap layer, encoded outside the lock.

        Encoding under the lock on a cache miss made costmap and state
        callbacks wait for the whole compress (2026-10-27). The cells are
        copied under the lock and the result is cached only if the grid is
        still the same object at the same version.
        """
        with self.lock:
            grid = self.grids.get(name)
            if grid is None:
                return None, None
            png, version = grid.cached_png, grid.version
            raw = None if png is not None else grid.data.tobytes()
        if png is None:
            png = grid.render(raw)
            with self.lock:
                if self.grids.get(name) is grid and grid.version == version:
                    grid.png = png
        return png, version

    def display_wanted(self, now, window=2.):
        """True while a full (non-kiosk) state client polled within ``window`` s.

        The bridge skips display-only work such as the scan transform when
        nobody is looking; the kiosk never shows the scan.
        """
        return now - self.last_snapshot_at <= window

    def snapshot(self, kiosk=False):
        with self.lock:
            if not kiosk:
                self.last_snapshot_at = time.monotonic()
            # Writers replace whole entries (set_state), never mutate them, so
            # one level of copying is enough for the per-call edits below. A
            # deepcopy of the 3,240-point scan at 5 Hz per client was 30 % of
            # this process's CPU on the Jetson (py-spy, 2026-10-23).
            result = {key: (dict(value) if isinstance(value, dict) else value)
                      for key, value in self.state.items() if not kiosk or key in KIOSK_STATE_KEYS}
            result['connection_epoch'] = self.connection_epoch
            result['kiosk'] = dict(pin_required=bool(self.kiosk_pin))
            live_map = self.grids.get('map')
            result['map_version'] = live_map.version if live_map is not None else None
            if not kiosk:
                result['layers'] = {name: grid.metadata() for name, grid in self.grids.items()}
            result['telemetry'] = self.telemetry.snapshot()
        now = time.monotonic()
        if result.get('goal') and 'feedback_received_at' in result['goal']:
            result['goal']['feedback_age'] = max(0., now-result['goal'].pop('feedback_received_at'))
        if self.time_health.status and math.isfinite(self.time_health.received):
            result['time_health'] = dict(healthy=self.time_health.healthy(now),
                age=max(0., now-self.time_health.received), timeout=self.time_health.timeout,
                detail=self.time_health.status.get('detail', ''))
        else:
            result['time_health'] = None
        if self.require_time_health and not self.time_health.healthy(now):
            detail = ('로봇 시간 감시 응답이 끊겼습니다.'
                      if now-self.time_health.received >= self.time_health.timeout
                      else self.time_health.status.get('detail', '로봇 응답 대기'))
            result['tf_error'] = '시간·센서 상태 확인 필요: ' + detail
        result['estop'] = self.estop_snapshot(now)
        if not kiosk:
            result['elevator'] = self.elevator_snapshot(now)
            result['transit'] = self.transit_snapshot(now)
            result['camera'] = self.camera_snapshot(now)
        for name in ('robot', 'amcl', 'scan', 'global_path', 'local_path', 'footprint', 'velocity', 'localization'):
            item = result.get(name)
            if item:
                item['age'] = max(0, now-item.pop('received_at'))
        localization = result.get('localization')
        if localization:
            localization['age'] = max(localization['age'], time.time()-localization.get('stamp', 0))
            localization['ready'] = (localization['ready'] and -1 <= time.time()-localization.get('stamp', 0) < 2
                                     and localization['age'] < 2
                                     and not result['localization_reset_pending']
                                     and (not self.require_time_health or self.time_health.healthy(now)))
        result['task'] = task_summary(result)
        return {key: value for key, value in result.items() if key in KIOSK_STATE_KEYS} if kiosk else result

    def estop_snapshot(self, now):
        with self.lock:
            status, received = dict(self.estop.status), self.estop.received
        live = bool(status) and 0 <= now-received < self.estop.timeout
        stm32 = status.get('stm32') if isinstance(status.get('stm32'), dict) else {}
        return dict(
            required=self.require_estop, received=bool(status), fresh=live,
            age=max(0., now-received) if math.isfinite(received) else None, timeout=self.estop.timeout,
            state=status.get('state') if live else None,
            healthy=self.estop.healthy(now),
            detail=str(status.get('detail', ''))[:300], reason=str(status.get('reason', ''))[:80],
            rx_error=str(status.get('rx_error', ''))[:300], reboot_suspected=status.get('reboot_suspected') is True,
            status_age=status.get('status_age'),
            stm32={key: stm32[key] for key in ESTOP_STM32_KEYS
                   if type(stm32.get(key)) in (bool, int)},
            error=self.estop_error(now))

    def request(self, kind, payload):
        request_id = payload.get('request_id', '')
        if not isinstance(request_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{8,80}', request_id):
            raise ApiError(400, '요청 ID가 필요합니다.')
        if kind in ('stop', 'resume', 'return_home'):
            return self.operator_request(kind, request_id)
        if kind not in ('goal', 'initialpose', 'cancel', 'relocalize', 'set_floor'):
            raise ApiError(400, '지원하지 않는 명령입니다.')
        pose = validate_pose(payload) if kind in ('goal', 'initialpose') else None
        floor_name, room = payload.get('floor'), payload.get('room', '')
        if kind == 'set_floor':
            if not isinstance(floor_name, str) or not floor_name:
                raise ApiError(400, '층 정보가 필요합니다.')
            with self.lock:
                if self.building is None:
                    raise ApiError(404, '층 설정이 없습니다.')
                if floor_name not in self.building.floors:
                    raise ApiError(404, '알 수 없는 층입니다.')
        elif floor_name is not None or room:
            if kind != 'goal' or not isinstance(floor_name, str) or not isinstance(room, str) or len(room) > 32:
                raise ApiError(400, '층과 호실 정보가 올바르지 않습니다.')
            with self.lock:
                current = self.current_floor()
            if current is None:
                raise ApiError(409, '현재 층 정보가 없습니다. 층 확인을 기다려 주세요.')
            if floor_name != current:
                kind = 'floor_goal'  # another floor: the elevator mission handles it
        signature = (kind, pose, payload.get('map_version'), floor_name, room)
        with self.lock:
            if request_id in self.requests:
                old = self.requests[request_id]
                if old['signature'] != signature:
                    raise ApiError(409, '같은 요청 ID에 다른 명령을 사용할 수 없습니다.')
                return {k: v for k, v in old.items() if k != 'signature'}
            if kind == 'floor_goal':
                grid = self.floor_grid(floor_name)
                if not (self.simulation or self.state['preview'] or self.building.floor(floor_name).real_enabled):
                    raise ApiError(409, f'{floor_name}은 실차 층간 이동이 꺼져 있습니다.')
                if payload.get('map_version') != grid.version:
                    raise ApiError(409, '지도가 갱신되었습니다. 목적지를 다시 선택해 주세요.')
                occupancy = grid.cell(pose[0], pose[1])
                if occupancy < 0 or occupancy >= 65:
                    raise ApiError(422, '벽이나 미확인 영역입니다. 이동 가능한 공간을 선택해 주세요.')
            elif pose is not None or kind == 'relocalize':
                grid = self.grids.get('map')
                if grid is None and pose is not None:
                    raise ApiError(409, '지도를 기다리고 있습니다.')
                if grid is not None and payload.get('map_version') != grid.version:
                    raise ApiError(409, '지도가 갱신되었습니다. 목적지를 다시 선택해 주세요.')
                if pose is not None:
                    occupancy = grid.cell(pose[0], pose[1])
                    if occupancy < 0 or occupancy >= 65:
                        raise ApiError(422, '벽이나 미확인 영역입니다. 이동 가능한 공간을 선택해 주세요.')
            if kind in ('goal', 'floor_goal'):
                self.cancel_pending_return('새 목적지가 요청되어 복귀를 취소했습니다.')
            if not self.state['preview'] or kind in ('goal', 'floor_goal'):
                error = self.readiness_error(kind)
                if error:
                    raise ApiError(409, error)
            record = dict(request_id=request_id, kind=kind, status='QUEUED', signature=signature)
            if self.state['preview']:
                record['status'] = 'PREVIEW'
                if pose:
                    self.state['goal'] = dict(x=pose[0], y=pose[1], yaw=pose[2], floor=floor_name, room=room,
                                              request_id=request_id, status='PREVIEW', kind=kind)
                elif kind == 'cancel':
                    self.state['goal'] = None
            else:
                try:
                    self.commands.put_nowait(dict(kind=kind, request_id=request_id, pose=pose,
                                                 map_version=payload.get('map_version'),
                                                 floor=floor_name, room=room))
                except queue.Full:
                    raise ApiError(429, '요청이 많습니다. 잠시 후 다시 시도해 주세요.')
                self.state['pending_request'] = dict(request_id=request_id, kind=kind)
                if kind in ('initialpose', 'relocalize'):
                    self.begin_localization_reset()
            self.requests[request_id] = record
            while len(self.requests) > 256:
                self.requests.popitem(last=False)
            return {k: v for k, v in record.items() if k != 'signature'}

    def cancel_pending_return(self, detail):
        if self.pending_return:
            request_id = self.pending_return['request_id']
            self.pending_return = None
            self.complete(request_id, 'CANCELED', detail)

    def set_hold(self, held):
        """Change the published hold; returns the /operator/hold message or None if unchanged."""
        self.state['hold']['pending'] = False
        if self.state['hold']['held'] == held:
            return None
        self.state['hold'] = dict(held=held, since=time.time() if held else None, reason='central',
                                  pending=False)
        self.hold_sequence += 1
        return dict(held=held, reason='central', stamp=time.time(), sequence=self.hold_sequence)

    def hold_deferred(self):
        goal = self.state.get('goal') or {}
        phase = (self.state.get('floor_mission') or {}).get('phase')
        return goal.get('status') in ACTIVE_GOALS and phase in HOLD_DEFER_PHASES

    def next_hold_event(self):
        """Poll on the executor: engage a pending hold once the elevator segment is over."""
        with self.lock:
            if not self.state['hold'].get('pending') or self.hold_deferred():
                return None
            goal = self.state.get('goal') or {}
            if goal.get('status') in ACTIVE_GOALS:
                self.cancel_goal_ids.add(goal['request_id'])
            return self.set_hold(True)

    def operator_request(self, kind, request_id):
        with self.lock:
            signature = (kind,)
            old = self.requests.get(request_id)
            if old:
                if old['signature'] != signature:
                    raise ApiError(409, '같은 요청 ID에 다른 명령을 사용할 수 없습니다.')
                return dict(old['response'])
            home = getattr(self.building, 'home', None)
            if kind == 'return_home' and home is None:
                raise ApiError(404, '복귀 지점이 설정되지 않았습니다.')
            response = dict(request_id=request_id)
            deferred = False
            if kind in ('stop', 'return_home'):
                self.cancel_pending_return('새 관리자 명령으로 복귀를 취소했습니다.')
                # Drop goals not yet sent, including a stop followed quickly by resume.
                retained = []
                while not self.commands.empty():
                    command = self.commands.get_nowait()
                    if command['kind'] in ('goal', 'floor_goal'):
                        self.complete(command['request_id'], 'CANCELED', '관리자 명령으로 취소했습니다.')
                    else:
                        retained.append(command)
                for command in retained:
                    self.commands.put_nowait(command)
                goal = self.state.get('goal') or {}
                canceled = goal.get('request_id') if goal.get('status') in ACTIVE_GOALS else None
                deferred = kind == 'stop' and not self.state['hold']['held'] and self.hold_deferred()
                if canceled and not deferred:
                    self.cancel_goal_ids.add(canceled)
                if kind == 'stop':
                    response['canceled'] = None if deferred else canceled
                    response['pending'] = deferred
            if kind == 'stop' and deferred:
                # The ride continues; next_hold_event() engages the hold back in a corridor.
                self.state['hold'] = dict(held=False, since=time.time(), reason='central', pending=True)
                event = None
            else:
                event = self.set_hold(kind == 'stop')
            if kind == 'return_home':
                response['home'] = dict(floor=home.floor, room=home.room)
                self.pending_return = dict(request_id=request_id, home=home, started=time.monotonic())
            else:
                response['held'] = kind == 'stop' and not deferred
            self.requests[request_id] = dict(request_id=request_id, kind=kind, status='QUEUED',
                                             signature=signature, response=response)
            while len(self.requests) > 256:
                self.requests.popitem(last=False)
            self.state['pending_request'] = dict(request_id=request_id, kind=kind)
            if self.state['preview']:
                if kind == 'return_home':
                    pose = self.building.floor(home.floor).rooms[home.room].goal
                    goal_kind = 'goal' if self.current_floor() == home.floor else 'floor_goal'
                    self.state['goal'] = dict(request_id=request_id, kind=goal_kind, origin='return_home',
                        floor=home.floor, room=home.room, x=pose[0], y=pose[1], yaw=pose[2], status='PREVIEW')
                    self.pending_return = None
                self.complete(request_id, 'PREVIEW')
            else:
                self.operator_commands.append(dict(kind=kind, request_id=request_id, hold=event))
            return dict(response)

    def next_return_command(self, now=None):
        """Poll on the executor; dispatch only after cancellation and readiness settle."""
        with self.lock:
            pending = self.pending_return
            if pending is None:
                return None
            now = time.monotonic() if now is None else now
            home, request_id = pending['home'], pending['request_id']
            kind = 'goal' if self.current_floor() == home.floor else 'floor_goal'
            goal = self.state.get('goal') or {}
            # The hold release travels on a topic and the goal on an action: give the
            # managers time to see held=false, or they reject the goal.
            error = ('이전 목표의 종료를 기다리고 있습니다.' if goal.get('status') in ACTIVE_GOALS
                     else '관리자 정지 해제를 기다리고 있습니다.'
                     if now - pending['started'] < .5 or self.state.get('navigation') == 'HELD'
                     else self.readiness_error(kind, request_id))
            command = None
            if not error:
                try:
                    if self.current_floor() is None:
                        raise ApiError(409, '현재 층 정보가 없습니다.')
                    grid = self.floor_grid(home.floor) if kind == 'floor_goal' else self.grids.get('map')
                    if grid is None:
                        raise ApiError(409, '지도를 기다리고 있습니다.')
                    floor = self.building.floor(home.floor)
                    if kind == 'floor_goal' and not (self.simulation or floor.real_enabled):
                        raise ApiError(409, f'{home.floor}은 실차 층간 이동이 꺼져 있습니다.')
                    pose = floor.rooms[home.room].goal
                    if not 0 <= grid.cell(pose[0], pose[1]) < 65:
                        raise ApiError(422, '벽이나 미확인 영역입니다. 이동 가능한 공간을 선택해 주세요.')
                    command = dict(kind=kind, request_id=request_id, pose=pose, map_version=grid.version,
                                   floor=home.floor, room=home.room, origin='return_home')
                except ApiError as exc:
                    error = str(exc)
            if now - pending['started'] >= 30:
                self.pending_return = None
                self.complete(request_id, 'FAILED', '복귀를 시작하지 못했습니다: ' + (error or '출발 대기 시간 초과'))
                return None
            if command:
                self.pending_return = None
                self.state['pending_request'] = dict(request_id=request_id, kind=kind)
            return command

    def complete(self, request_id, status, detail=''):
        with self.lock:
            if request_id in self.requests:
                self.requests[request_id].update(status=status, detail=detail)
            pending = self.state['pending_request']
            if pending and pending['request_id'] == request_id:
                self.state['pending_request'] = None
            self.state['notice'] = dict(request_id=request_id, status=status, detail=detail)


def load_preview(shared, filename):
    import yaml
    from PIL import Image
    filename = Path(filename).resolve()
    config = yaml.safe_load(filename.read_text())
    if config.get('mode', 'trinary') != 'trinary':
        raise ValueError('Preview supports trinary map YAML files')
    image = Image.open(filename.parent / config['image']).convert('L')
    image = image.transpose(getattr(Image, 'Transpose', Image).FLIP_TOP_BOTTOM)
    negate = config.get('negate', 0)
    data = []
    for value in image.tobytes():
        occupied = value/255 if negate else (255-value)/255
        data.append(100 if occupied > config['occupied_thresh'] else
                    0 if occupied < config['free_thresh'] else -1)
    shared.grids['map'] = Grid(*image.size, config['resolution'], config['origin'], data)


def create_http_server(shared, assets, host='127.0.0.1', port=8080):
    assets = Path(assets)

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        # Keep-alive holds a thread per connection; drop idle or half-open
        # clients (a tablet leaving Wi-Fi) instead of keeping them forever.
        timeout = 30

        def log_message(self, *_args):
            pass

        def respond(self, status, body, mime='application/json', headers=None):
            if not isinstance(body, bytes):
                body = json_bytes(body)
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            if self.close_connection:
                self.send_header('Connection', 'close')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy',
                             "default-src 'self'; img-src 'self' blob: data:; "
                             "style-src 'self' 'unsafe-inline'; frame-ancestors 'none'")
            for key, value in (headers or {}).items():
                self.send_header(key, str(value))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/state':
                self.respond(200, encode_snapshot(shared.snapshot()))
            elif path == '/api/kiosk/state':
                self.respond(200, shared.snapshot(kiosk=True))
            elif path == '/api/floors':
                self.respond(200, shared.floors_info())
            elif path.startswith('/api/floors/') and path.endswith('/map.png'):
                name = path.removeprefix('/api/floors/').removesuffix('/map.png')
                try:
                    grid = shared.floor_grid(name)
                    self.respond(200, grid.png, 'image/png', {'X-Map-Version': grid.version})
                except ApiError as error:
                    self.respond(error.status, dict(error=str(error)))
            elif path.startswith('/api/layers/') and path.endswith('.png'):
                name = path.removeprefix('/api/layers/').removesuffix('.png')
                png, version = shared.layer_png(name)
                if png is None:
                    self.respond(404, dict(error='아직 레이어 데이터가 없습니다.'))
                else:
                    self.respond(200, png, 'image/png', {'X-Map-Version': version})
            elif path == '/api/camera.jpg':
                frame = shared.camera_frame()
                if frame is None:
                    self.respond(404, dict(error='카메라 영상이 없습니다.'))
                else:
                    self.respond(200, frame[0], 'image/jpeg', {'X-Camera-Age': f'{frame[1]:.2f}'})
            elif path in ('/', '/index.html', '/app.js', '/geometry.js', '/dashboard.js', '/transit.js',
                          '/battery-profile.js', '/style.css', '/favicon.svg',
                          '/kiosk', '/kiosk.js', '/kiosk.css'):
                name = {'/': 'index.html', '/kiosk': 'kiosk.html'}.get(path, path[1:])
                mime = {'html': 'text/html; charset=utf-8', 'js': 'text/javascript; charset=utf-8',
                        'css': 'text/css; charset=utf-8', 'svg': 'image/svg+xml'}[name.rsplit('.', 1)[1]]
                try:
                    data = (assets/name).read_bytes()
                except FileNotFoundError:
                    self.respond(404, dict(error='Not found'))
                else:
                    self.respond(200, data, mime)
            else:
                self.respond(404, dict(error='Not found'))

        def do_POST(self):
            consumed = False
            try:
                if self.headers.get('Transfer-Encoding'):
                    raise ApiError(400, 'Chunked requests are not supported')
                origin = self.headers.get('Origin')
                if origin and urlsplit(origin).netloc != self.headers.get('Host'):
                    raise ApiError(403, '다른 웹사이트에서 보낸 명령은 허용하지 않습니다.')
                if self.headers.get('X-AMR-Request') != '1':
                    raise ApiError(403, '명령 요청 헤더가 필요합니다.')
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise ApiError(415, 'JSON 요청이 필요합니다.')
                size = int(self.headers.get('Content-Length', '0'))
                if size <= 0 or size > 4096:
                    raise ApiError(413, '요청 크기가 유효하지 않습니다.')
                raw = self.rfile.read(size)
                consumed = len(raw) == size
                payload = json.loads(raw)
                if not isinstance(payload, dict):
                    raise ValueError('JSON object required')
                kinds = {'/api/goal': 'goal', '/api/initialpose': 'initialpose',
                         '/api/cancel': 'cancel', '/api/relocalize': 'relocalize',
                         '/api/set_floor': 'set_floor', '/api/acknowledge': 'acknowledge',
                         '/api/kiosk/unlock': 'kiosk/unlock', '/api/kiosk/exit': 'kiosk/exit',
                         '/api/stop': 'stop',
                         '/api/resume': 'resume', '/api/return_home': 'return_home'}
                kind = kinds.get(self.path)
                if self.headers.get('X-AMR-Client') == 'kiosk':
                    shared.check_kiosk_request(kind, payload, self.headers.get('X-AMR-Kiosk-Token'))
                if self.path not in kinds:
                    raise ApiError(404, 'Not found')
                if kind == 'kiosk/unlock':
                    self.respond(200, shared.unlock_kiosk(payload))
                elif kind == 'kiosk/exit':
                    if not ipaddress.ip_address(self.client_address[0]).is_loopback:
                        raise ApiError(403, '로봇 화면에서만 닫을 수 있습니다.')
                    # SIGTERM lets launch.sh close Chromium and restore the screen-lock settings;
                    # without a launcher, close the orphaned browser itself.
                    pid = kiosk_launcher_pid() or kiosk_browser_pid()
                    if pid is None:
                        raise ApiError(409, '키오스크 화면을 찾지 못했습니다.')
                    self.respond(202, dict(closing=True))
                    try:
                        os.kill(pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                elif kind == 'acknowledge':
                    self.respond(202, shared.acknowledge(payload))
                else:
                    self.respond(202, shared.request(kind, payload))
            except ApiError as error:
                self.close_connection = self.close_connection or not consumed
                self.respond(error.status, dict(error=str(error)))
            except (ValueError, TypeError, json.JSONDecodeError) as error:
                self.close_connection = self.close_connection or not consumed
                self.respond(400, dict(error=str(error)))

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server


def asset_directory():
    local = Path(__file__).resolve().parents[1] / 'web'
    if local.is_dir():
        return local
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('amr_web_ui')) / 'web'


def main(args=None):
    parser = argparse.ArgumentParser(description='AMR touch map navigation')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--preview-map', help='Map YAML; preview only, never starts ROS')
    options, ros_args = parser.parse_known_args(args)
    shared = SharedState(preview=bool(options.preview_map),
                         kiosk_pin=os.environ.get('AMR_KIOSK_PIN', ''))
    if options.preview_map:
        load_preview(shared, options.preview_map)
        try:
            from amr_stack.floor_config import load_floor_config
            shared.set_building(load_floor_config(), simulation=True)
        except Exception:  # noqa: BLE001 - floors are optional in preview
            pass
        node = executor = None
    else:
        import rclpy
        from .ros_bridge import WebBridge, spin_executor
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
        node = WebBridge(shared)
        executor = spin_executor()
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
    server = create_http_server(shared, asset_directory(), options.host, options.port)
    print(f'AMR Web UI: http://{options.host}:{server.server_port}' +
          (' (preview: no ROS commands)' if options.preview_map else ''), flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if executor:
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
