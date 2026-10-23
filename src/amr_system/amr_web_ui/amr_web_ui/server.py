"""Offline-capable HTTP UI. ROS commands are executed only on the ROS executor."""
import argparse
from collections import OrderedDict
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import queue
import re
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
ESTOP_STATES = {'UNKNOWN', 'ESTOP', 'DISARMED', 'ARMING', 'READY', 'BYPASS'}
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
COMMAND_STATUS_NAMES = {1: 'DELIVERED', 2: 'COMPLETED', 3: 'DEVICE_REJECTED', 4: 'RF_TIMEOUT',
                        5: 'NO_RESULT', 6: 'NO_ROUTE', 7: 'BAD_FRAME', 8: 'REPLACED'}
DEVICE_RESULT_NAMES = {0x00: 'PRESSED', 0x01: 'DUPLICATE', 0x81: 'WRONG_DEVICE', 0x82: 'UNSUPPORTED',
                       0x83: 'BUTTON_FAULT', 0x84: 'BAD_REQUEST', 0xFF: 'NONE'}


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


class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


class SharedState:
    def __init__(self, preview=False):
        self.lock = threading.RLock()
        self.commands = queue.Queue(maxsize=32)
        self.grids = {}
        self.grid_versions = {}
        self.requests = OrderedDict()
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
        self.connection_epoch = 0
        self.telemetry = DisplayTelemetry()
        # Multi-floor (floors.yaml); None when the elevator packages are absent.
        self.building = None
        self.simulation = False
        self.floor_grids = {}
        self.state = dict(preview=preview, floor=None, floor_mission=None, floor_action_ready=False, navigation='UNKNOWN', mode='UNKNOWN',
                          action_ready=False, robot=None, amcl=None, scan=None,
                          global_path=None, local_path=None, footprint=None,
                          goal=None, notice=None, velocity=None, tf_error=None, pending_request=None,
                          localization=None, localization_reset_pending=False)

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
            return 'E-stop이 눌려 있습니다. 버튼을 원위치로 돌린 뒤 재개를 승인하세요.'
        if state == 'DISARMED':
            return 'E-stop이 해제되었습니다. 주변을 확인하고 재개를 승인한 뒤 목적지를 지정하세요.'
        if state == 'ARMING':
            return '재개 승인 후 주행 준비 중입니다. 잠시 후 다시 시도하세요.'
        if state == 'UNKNOWN':
            return 'STM32 E-stop 상태를 기다리고 있습니다.'
        return 'E-stop 상태를 받지 못했습니다. CAN 브릿지 연결을 확인하세요.'

    def reset_connection(self, reason):
        """Invalidate display/request state; never replay a command after restart."""
        with self.lock:
            self.connection_epoch += 1
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
        with self.lock:
            self.building, self.simulation = building, bool(simulation)
            self.floor_grids.clear()

    def floor_grid(self, name):
        """Static map of any configured floor (current or not) for choosing a destination."""
        with self.lock:
            if self.building is None:
                raise ApiError(404, '층 설정이 없습니다.')
            if name in self.floor_grids:
                return self.floor_grids[name]
            try:
                floor = self.building.floor(name)
            except ValueError:
                raise ApiError(404, '알 수 없는 층입니다.') from None
            from amr_stack.floor_config import load_occupancy_map
            import hashlib
            path = self.building.map_path(name, self.simulation)
            occupancy = load_occupancy_map(path)
            height, width = occupancy.cells.shape
            grid = Grid(width, height, occupancy.resolution, list(occupancy.origin),
                        occupancy.cells.ravel().tolist())
            # Stable across restarts for the same file, distinct between floors and map revisions.
            digest = hashlib.sha256(name.encode() + path.read_bytes() + occupancy.cells.tobytes())
            grid.version = int(digest.hexdigest()[:12], 16)
            grid.floor = floor.name
            self.floor_grids[name] = grid
            return grid

    def current_floor(self):
        floor = self.state.get('floor')
        return floor.get('floor') if floor else None

    def floors_info(self):
        with self.lock:
            if self.building is None:
                return dict(floors=[], default_floor=None, current_floor=None, simulation=self.simulation)
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
                        current_floor=self.current_floor(), simulation=self.simulation)

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
                return None
            pending = self.state['pending_request']
            if pending and pending['request_id'] != request_id:
                return '이전 요청을 처리 중입니다. 잠시 기다려 주세요.'
            if kind in ('goal', 'floor_goal'):
                estop = self.estop_error()
                if estop:
                    return estop
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
            elif kind in ('initialpose', 'relocalize'):
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
            elif kind == 'estop_approve':
                status = self.estop_live()
                if not status:
                    return 'E-stop 상태를 받지 못해 재개를 승인할 수 없습니다.'
                if status.get('approval_required') is not True:
                    return {'ESTOP': 'E-stop이 아직 눌려 있거나 해제가 확정되지 않았습니다.',
                            'ARMING': '이미 재개 승인되어 주행 준비 중입니다.',
                            'READY': '이미 주행 가능 상태입니다.',
                            'BYPASS': 'E-stop 확인이 꺼져 있어 승인이 필요 없습니다.'}.get(
                        status.get('state'), '지금은 재개를 승인할 수 없습니다.')
            return None

    def snapshot(self):
        with self.lock:
            result = deepcopy(self.state)
            result['connection_epoch'] = self.connection_epoch
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
        result['elevator'] = self.elevator_snapshot(now)
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
        return result

    def estop_snapshot(self, now):
        with self.lock:
            status, received = dict(self.estop.status), self.estop.received
        live = bool(status) and 0 <= now-received < self.estop.timeout
        stm32 = status.get('stm32') if isinstance(status.get('stm32'), dict) else {}
        return dict(
            required=self.require_estop, received=bool(status), fresh=live,
            age=max(0., now-received) if math.isfinite(received) else None, timeout=self.estop.timeout,
            state=status.get('state') if live else None,
            healthy=self.estop.healthy(now), approval_required=live and status.get('approval_required') is True,
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
        if kind not in ('goal', 'initialpose', 'cancel', 'relocalize', 'estop_approve'):
            raise ApiError(400, '지원하지 않는 명령입니다.')
        pose = validate_pose(payload) if kind in ('goal', 'initialpose') else None
        floor_name, room = payload.get('floor'), payload.get('room', '')
        if floor_name is not None or room:
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
                    raise ApiError(409, f'{floor_name}은 임시 지도라 실차 층간 이동이 꺼져 있습니다.')
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
            if not self.state['preview']:
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
        def log_message(self, *_args):
            pass

        def respond(self, status, body, mime='application/json', headers=None):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
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
                self.respond(200, shared.snapshot())
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
                with shared.lock:
                    grid = shared.grids.get(name)
                    if grid:
                        png, version = grid.png, grid.version
                    else:
                        png = None
                if png is None:
                    self.respond(404, dict(error='아직 레이어 데이터가 없습니다.'))
                else:
                    self.respond(200, png, 'image/png', {'X-Map-Version': version})
            elif path in ('/', '/index.html', '/app.js', '/geometry.js', '/dashboard.js',
                          '/battery-profile.js', '/style.css', '/favicon.svg'):
                name = 'index.html' if path == '/' else path[1:]
                mime = {'html': 'text/html; charset=utf-8', 'js': 'text/javascript; charset=utf-8',
                        'css': 'text/css; charset=utf-8', 'svg': 'image/svg+xml'}[name.rsplit('.', 1)[1]]
                self.respond(200, (assets/name).read_bytes(), mime)
            else:
                self.respond(404, dict(error='Not found'))

        def do_POST(self):
            try:
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
                payload = json.loads(self.rfile.read(size))
                if not isinstance(payload, dict):
                    raise ValueError('JSON object required')
                kinds = {'/api/goal': 'goal', '/api/initialpose': 'initialpose',
                         '/api/cancel': 'cancel', '/api/relocalize': 'relocalize',
                         '/api/estop_approve': 'estop_approve'}
                if self.path not in kinds:
                    raise ApiError(404, 'Not found')
                result = shared.request(kinds[self.path], payload)
                self.respond(202, result)
            except ApiError as error:
                self.respond(error.status, dict(error=str(error)))
            except (ValueError, TypeError, json.JSONDecodeError) as error:
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
    shared = SharedState(preview=bool(options.preview_map))
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
        from rclpy.executors import SingleThreadedExecutor
        from .ros_bridge import WebBridge
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
        node = WebBridge(shared)
        executor = SingleThreadedExecutor()
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
