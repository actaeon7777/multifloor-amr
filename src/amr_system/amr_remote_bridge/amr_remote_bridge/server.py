"""Central host: cache AMR telemetry, serve the original UI, relay explicit commands."""
import argparse
import base64
from collections import OrderedDict
from copy import deepcopy
from dataclasses import dataclass, field
import hashlib
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import math
from pathlib import Path
import re
import threading
import time
from urllib.parse import urlsplit

from .common import (AGED_FIELDS, COMMANDS, FLOOR_LAYER, LAYERS, LEASED_FIELDS, MAX_COMMAND, MAX_FLOORS,
                     MAX_MESSAGE, PROTOCOL_VERSION, REQUEST_ID, dumps, floor_layer, floor_map_version,
                     identifier, layer_floor, loads, positive, read_token)

DISPLAY_FIELDS = ('scan', 'wheel', 'imu', 'ekf', 'joints', 'command', 'diagnostics', 'power',
                  'vehicle', 'pose', 'localization', 'time', 'battery')
DISPLAY_FIELDS += tuple('motor_'+side+'_'+kind for side in ('left','right')
                        for kind in ('heartbeat','power','temperature'))


def age_items(state):
    for name in (*AGED_FIELDS, 'time_health'):
        if state.get(name) is not None:
            yield state[name]
    telemetry = state.get('telemetry')
    if telemetry is not None:
        if not isinstance(telemetry, dict):
            raise ApiError(400, 'Invalid display telemetry')
        for name in DISPLAY_FIELDS:
            if telemetry.get(name) is not None:
                yield telemetry[name]


def _number(value):
    return type(value) in (int, float) and math.isfinite(value) and value >= 0


def lease_items(state):
    """E-stop / elevator status blocks: a dict whose age is null or a non-negative number."""
    for name in LEASED_FIELDS:
        item = state.get(name)
        if item is None:
            continue
        if (not isinstance(item, dict) or not (item.get('age') is None or _number(item['age']))
                or not (item.get('timeout') is None or _number(item['timeout']))):
            raise ApiError(400, f'Invalid {name} status')
        yield item


def age_lease(item, extra_age):
    """Add the relay delay; a lease that ran out is no longer fresh, never extended."""
    if item.get('age') is None:
        return
    item['age'] += extra_age
    timeout = item.get('timeout')
    if timeout is not None and item['age'] >= timeout:
        item['fresh'] = False
        for key in ('approval_required', 'healthy'):
            if key in item:
                item[key] = False


class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


@dataclass
class Command:
    command_id: str
    kind: str
    signature: dict
    payload: dict
    deadline: float
    map_digest: str = ''
    map_layer: str = 'map'
    status: str = 'QUEUED'
    result: dict | None = None
    event: threading.Event = field(default_factory=threading.Event)


@dataclass
class Robot:
    robot_id: str
    session_id: str = ''
    sequence: int = 0
    last_exchange: float | None = None
    captured: float | None = None
    state: dict | None = None
    local_error: str | None = None
    floors: dict | None = None
    layers: dict = field(default_factory=dict)
    revisions: dict = field(default_factory=dict)
    commands: OrderedDict = field(default_factory=OrderedDict)


class Registry:
    def __init__(self, stale_after=2, command_timeout=3, lease_timeout=3, max_robots=32):
        self.lock = threading.RLock()
        self.robots = {}
        self.stale_after = positive(stale_after)
        self.command_timeout = positive(command_timeout)
        self.lease_timeout = positive(lease_timeout)
        self.max_robots = max_robots

    def _fresh(self, robot, now):
        return (robot.state is not None and robot.captured is not None and
                robot.last_exchange is not None and now - robot.captured < self.stale_after and
                now - robot.last_exchange < self.stale_after)

    @staticmethod
    def _validate_floors(floors):
        """The agent forwards the local /api/floors document; only what the relay relies on is checked."""
        if floors is None:
            return None
        if not isinstance(floors, dict) or not isinstance(floors.get('floors'), list) or len(floors['floors']) > MAX_FLOORS:
            raise ApiError(400, 'Invalid floor list')
        names = set()
        for entry in floors['floors']:
            if not isinstance(entry, dict):
                raise ApiError(400, 'Invalid floor entry')
            name = entry.get('name')
            try:
                identifier(name)
            except ValueError:
                raise ApiError(400, 'Invalid floor name') from None
            if name in names:
                raise ApiError(400, 'Duplicate floor name')
            names.add(name)
            layer = entry.get('map')
            if layer is not None and (not isinstance(layer, dict) or not isinstance(layer.get('version'), int)
                                      or isinstance(layer.get('version'), bool)):
                raise ApiError(400, 'Invalid floor map metadata')
        return deepcopy(floors)

    @staticmethod
    def _source_version(state, floors, name):
        """Version the AMR currently reports for a live or floor layer, or None."""
        floor = layer_floor(name)
        if floor:
            return floor_map_version(floors, floor)
        info = state['layers'].get(name)
        return info['version'] if info else None

    @staticmethod
    def _current_floor(state):
        # Same rule as the UI: while riding, the live map is still the departure floor's.
        floor = state.get('floor') if isinstance(state, dict) else None
        if not isinstance(floor, dict):
            return None
        current = floor.get('floor') or floor.get('transit_from')
        return current if isinstance(current, str) else None

    @staticmethod
    def _finish(command, status, body):
        command.result = dict(http_status=status, body=body)
        command.status = 'RESULT'
        command.event.set()

    def _expire(self, robot, now):
        for command in robot.commands.values():
            if command.result is None and now >= command.deadline:
                sent = command.status == 'SENT'
                self._finish(command, 504 if sent else 408, dict(error=
                    '명령 응답을 확인하지 못했습니다. 자동 재전송하지 않습니다. 현재 상태를 확인해 주세요.'
                    if sent else '전달 전에 명령이 만료되었습니다. 현재 상태를 확인하고 다시 요청해 주세요.'))

    def exchange(self, message):
        if not isinstance(message, dict) or message.get('protocol') != PROTOCOL_VERSION:
            raise ApiError(400, 'Unsupported agent protocol')
        robot_id = identifier(message.get('robot_id'))
        session_id, sequence = message.get('session_id'), message.get('sequence')
        if (not isinstance(session_id, str) or not re.fullmatch(r'[0-9a-f]{32}', session_id) or
                not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 1):
            raise ApiError(400, 'Invalid session or sequence')
        state, age = message.get('state'), message.get('state_age')
        if (not isinstance(age, (int, float)) or isinstance(age, bool) or
                not math.isfinite(age) or not 0 <= age <= 3600):
            raise ApiError(400, 'Invalid telemetry age')
        if state is not None:
            if not isinstance(state, dict) or not isinstance(state.get('layers'), dict):
                raise ApiError(400, 'Invalid telemetry state')
            for name, info in state['layers'].items():
                if (name not in LAYERS or not isinstance(info, dict) or
                        not isinstance(info.get('version'), int) or isinstance(info.get('version'), bool)):
                    raise ApiError(400, 'Invalid layer metadata')
            for item in age_items(state):
                if (not isinstance(item, dict) or not isinstance(item.get('age'), (int, float)) or
                        isinstance(item.get('age'), bool) or not math.isfinite(item['age']) or item['age'] < 0):
                    raise ApiError(400, 'Invalid sensor age')
            list(lease_items(state))  # Raises on a malformed E-stop or elevator block.
            goal = state.get('goal')
            if isinstance(goal, dict) and 'feedback_age' in goal:
                value = goal['feedback_age']
                if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                    raise ApiError(400, 'Invalid goal feedback age')
        updates, results = message.get('layers', {}), message.get('results', [])
        if not isinstance(updates, dict) or not isinstance(results, list) or len(results) > 256:
            raise ApiError(400, 'Invalid exchange contents')
        floors = self._validate_floors(message.get('floors')) if state is not None else None
        decoded = {}
        for name, image in updates.items():
            try:
                known = name in LAYERS or layer_floor(name) is not None
            except ValueError:
                known = False
            if not known or not isinstance(image, dict) or state is None:
                raise ApiError(400, 'Invalid layer update')
            version = self._source_version(state, floors, name)
            if version is None or image.get('version') != version:
                raise ApiError(400, 'Layer image does not match its metadata')
            try:
                png = base64.b64decode(image['png'], validate=True)
            except (ValueError, TypeError, KeyError):
                raise ApiError(400, 'Invalid layer encoding') from None
            if len(png) > 2 * 1024 * 1024 or not png.startswith(b'\x89PNG\r\n\x1a\n'):
                raise ApiError(400, 'Invalid or oversized PNG')
            decoded[name] = dict(source_version=version, png=png,
                                 digest=hashlib.sha256(png).hexdigest())
        for result in results:
            if (not isinstance(result, dict) or not isinstance(result.get('command_id'), str) or
                    not isinstance(result.get('http_status'), int) or
                    isinstance(result.get('http_status'), bool) or not 200 <= result['http_status'] <= 599 or
                    not isinstance(result.get('body'), dict) or len(dumps(result)) > 64 * 1024):
                raise ApiError(400, 'Invalid command result')
        now = time.monotonic()
        with self.lock:
            robot = self.robots.get(robot_id)
            if robot is None:
                if len(self.robots) >= self.max_robots:
                    raise ApiError(429, 'Robot capacity reached')
                robot = self.robots[robot_id] = Robot(robot_id)
            if robot.session_id != session_id:
                if robot.last_exchange is not None and now - robot.last_exchange < self.lease_timeout:
                    raise ApiError(409, 'Another active agent owns this robot_id')
                for command in robot.commands.values():
                    if command.result is None:
                        self._finish(command, 504, dict(error='AMR 클라이언트가 변경되었습니다. 현재 상태를 확인해 주세요.'))
                robot.session_id, robot.sequence = session_id, 0
                robot.layers.clear()
            if sequence <= robot.sequence:
                raise ApiError(409, 'Exchange sequence has already been used')
            robot.sequence, robot.last_exchange = sequence, now
            robot.state = deepcopy(state)
            robot.floors = floors
            robot.captured = now - age if state is not None else None
            robot.local_error = 'Local web API unavailable' if state is None else None
            for name, image in decoded.items():
                old = robot.layers.get(name)
                if (old is None or old['source_version'] != image['source_version'] or
                        old['digest'] != image['digest']):
                    robot.revisions[name] = robot.revisions.get(name, 0) + 1
                image['version'] = robot.revisions[name]
                robot.layers[name] = image
            accepted_results = []
            for result in results:
                command_id = result['command_id']
                command = robot.commands.get(command_id)
                if command is not None and command.status in ('SENT', 'RESULT'):
                    self._finish(command, result['http_status'], result['body'])
                # Old results can be discarded after a host restart; never re-execute them.
                accepted_results.append(command_id)
            self._expire(robot, now)
            commands = []
            if self._fresh(robot, now):
                for command in robot.commands.values():
                    if command.status == 'QUEUED':
                        command.status = 'SENT'
                        commands.append(dict(command_id=command.command_id, kind=command.kind,
                                             payload=command.payload, map_digest=command.map_digest,
                                             map_layer=command.map_layer, ttl=command.deadline - now))
                        break
            needed = []
            if state is not None:
                expected = {name: info['version'] for name, info in state['layers'].items()}
                for entry in (floors or {}).get('floors', ()):
                    if entry.get('map') is not None:
                        expected[floor_layer(entry['name'])] = entry['map']['version']
                needed = [name for name, version in expected.items()
                          if name not in robot.layers or robot.layers[name]['source_version'] != version]
            return dict(protocol=PROTOCOL_VERSION, session_id=session_id, sequence=sequence,
                        commands=commands, accepted_results=accepted_results,
                        accepted_layers=list(decoded), needed_layers=needed)

    def snapshot(self, robot_id):
        with self.lock:
            robot = self.robots.get(robot_id)
            now = time.monotonic()
            if robot is None or not self._fresh(robot, now):
                raise ApiError(503, 'AMR 연결 또는 상태 데이터가 지연되었습니다. 명령을 보낼 수 없습니다.')
            result = deepcopy(robot.state)
            extra_age = now - robot.captured
            for item in age_items(result):
                item['age'] += extra_age
            for item in lease_items(result):
                age_lease(item, extra_age)
            if isinstance(result.get('goal'), dict) and 'feedback_age' in result['goal']:
                result['goal']['feedback_age'] += extra_age
            telemetry = result.get('telemetry') or {}
            # Source leases are evaluated by the AMR at capture time. Preserve
            # those verdicts for the dashboard, independently of the relay lease.
            # End-to-end ages above remain conservative and unchanged for controls.
            source_telemetry = robot.state.get('telemetry') or {}
            for name in DISPLAY_FIELDS:
                if isinstance(telemetry.get(name), dict):
                    telemetry[name]['source_age'] = source_telemetry[name]['age']
            if result.get('localization') and result['localization']['age'] >= 2:
                result['localization']['ready'] = False
            result['layers'] = {name: dict(info, version=robot.layers[name]['version'])
                                for name, info in result['layers'].items()
                                if name in robot.layers and robot.layers[name]['source_version'] == info['version']}
            result['remote'] = dict(robot_id=robot_id, age=extra_age, timeout=self.stale_after)
            return result

    def _current_layer(self, robot, name):
        """The cached image for a layer whose version the AMR still reports, else None."""
        image = robot.layers.get(name)
        if image is None or image['source_version'] != self._source_version(robot.state, robot.floors, name):
            return None
        return image

    def layer(self, robot_id, name):
        with self.lock:
            self.snapshot(robot_id)  # Freshness check only.
            image = self._current_layer(self.robots[robot_id], name)
            if image is None:
                raise ApiError(404, '지도 레이어를 기다리고 있습니다.')
            return image['png'], image['version']

    def floors(self, robot_id):
        """The AMR's floor list with floor map versions translated to host revisions."""
        with self.lock:
            snapshot = self.snapshot(robot_id)
            robot = self.robots[robot_id]
            if robot.floors is None:
                raise ApiError(404, '층 설정이 없습니다.')
            result = deepcopy(robot.floors)
            for entry in result['floors']:
                image = self._current_layer(robot, floor_layer(entry['name'])) if entry.get('map') else None
                # The UI loads a floor's map only once its image has arrived here.
                entry['map'] = dict(entry['map'], version=image['version']) if image else None
            result['current_floor'] = self._current_floor(snapshot) or result.get('current_floor')
            return result

    def queue(self, robot_id, kind, payload):
        if kind not in COMMANDS or not isinstance(payload, dict):
            raise ApiError(400, 'Invalid command')
        request_id = payload.get('request_id')
        if not isinstance(request_id, str) or not REQUEST_ID.fullmatch(request_id):
            raise ApiError(400, '요청 ID가 필요합니다.')
        if kind in ('goal', 'initialpose'):
            for key in ('x', 'y', 'yaw'):
                value = payload.get(key)
                if (not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value)):
                    raise ApiError(400, '유효한 위치와 방향이 필요합니다.')
            if (not isinstance(payload.get('map_version'), int) or isinstance(payload['map_version'], bool)):
                raise ApiError(400, '지도 버전이 필요합니다.')
        elif kind == 'relocalize' and 'map_version' in payload:
            if type(payload['map_version']) is not int:
                raise ApiError(400, '유효한 지도 버전이 필요합니다.')
        floor, room = payload.get('floor'), payload.get('room', '')
        if floor is not None or room:
            if kind != 'goal' or not isinstance(floor, str) or not isinstance(room, str) or len(room) > 32:
                raise ApiError(400, '층과 호실 정보가 올바르지 않습니다.')
            try:
                identifier(floor)
            except ValueError:
                raise ApiError(400, '층과 호실 정보가 올바르지 않습니다.') from None
        now = time.monotonic()
        with self.lock:
            robot = self.robots.get(robot_id)
            if robot is None:
                raise ApiError(503, 'AMR 연결을 기다리고 있습니다.')
            old = robot.commands.get(request_id)
            signature = dict(kind=kind, payload=deepcopy(payload))
            if old is not None:
                if old.signature != signature:
                    raise ApiError(409, '같은 요청 ID에 다른 명령을 사용할 수 없습니다.')
                return old
            state = self.snapshot(robot_id)
            self._expire(robot, now)
            if any(command.result is None for command in robot.commands.values()):
                raise ApiError(409, '이전 원격 요청을 처리 중입니다. 잠시 기다려 주세요.')
            forwarded = deepcopy(payload)
            digest, layer_name = '', 'map'
            if kind == 'goal' and floor is not None and floor != self._current_floor(state):
                # A destination on another floor was chosen on that floor's static map.
                layer_name = floor_layer(floor)
                image = self._current_layer(robot, layer_name)
                if image is None:
                    raise ApiError(409, f'{floor} 지도를 기다리고 있습니다. 잠시 후 다시 선택해 주세요.')
                if image['version'] != payload.get('map_version'):
                    raise ApiError(409, '지도가 갱신되었습니다. 목적지를 다시 선택해 주세요.')
                forwarded['map_version'] = image['source_version']
                digest = image['digest']
            elif kind != 'cancel':
                info = state['layers'].get('map')
                # Relocalization has no pose and can be requested before a map
                # arrives. When one exists it must use the displayed revision.
                no_map_reset = (kind == 'relocalize' and not info
                                and 'map' not in robot.state['layers']
                                and 'map_version' not in payload)
                if not no_map_reset and (not info or info['version'] != payload.get('map_version')):
                    raise ApiError(409, '지도가 갱신되었습니다. 목적지를 다시 선택해 주세요.')
                if info:
                    forwarded['map_version'] = robot.layers['map']['source_version']
                    digest = robot.layers['map']['digest']
            command = Command(request_id, kind, signature, forwarded, now + self.command_timeout,
                              digest, layer_name)
            robot.commands[request_id] = command
            while len(robot.commands) > 256:
                robot.commands.popitem(last=False)
            return command

    def wait_result(self, robot_id, command):
        command.event.wait(max(0, command.deadline - time.monotonic()) + .05)
        with self.lock:
            self._expire(self.robots[robot_id], time.monotonic())
            return deepcopy(command.result)

    def roster(self):
        with self.lock:
            now = time.monotonic()
            return [dict(robot_id=robot.robot_id, online=self._fresh(robot, now),
                         age=now - robot.last_exchange if robot.last_exchange is not None else None,
                         local_api_available=robot.state is not None,
                         navigation=robot.state.get('navigation') if robot.state else None,
                         **self._badges(robot))
                    for robot in self.robots.values()]

    @staticmethod
    def _badges(robot):
        """E-stop state and elevator RF level for the fleet list (None when unknown)."""
        state = robot.state or {}
        estop, elevator = state.get('estop') or {}, state.get('elevator') or {}
        live = estop.get('fresh') is True
        return dict(estop=str(estop['state'])[:16] if live and estop.get('state') else None,
                    estop_required=estop.get('required') is True,
                    elevator_rf=elevator.get('level') if elevator.get('fresh') is True else None)


def find_assets(explicit=None):
    if explicit:
        result = Path(explicit).resolve()
    else:
        result = Path(__file__).resolve().parents[2] / 'amr_web_ui' / 'web'
        if not result.is_dir():
            try:
                from ament_index_python.packages import get_package_share_directory
                result = Path(get_package_share_directory('amr_web_ui')) / 'web'
            except (ImportError, LookupError):
                raise ValueError('Use --assets-dir to select the existing amr_web_ui/web directory') from None
    for name in ('index.html', 'app.js', 'geometry.js', 'style.css', 'favicon.svg'):
        if not (result / name).is_file():
            raise ValueError(f'Missing UI asset: {name}; check --assets-dir')
    return result


def create_server(registry, assets, token, host='127.0.0.1', port=8081, default_robot='amr-01'):
    assets = Path(assets)
    default_robot = identifier(default_robot)
    fleet_assets = Path(__file__).resolve().parents[1] / 'web'
    if not fleet_assets.is_dir():
        from ament_index_python.packages import get_package_share_directory
        fleet_assets = Path(get_package_share_directory('amr_remote_bridge')) / 'web'

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def setup(self):
            super().setup()
            self.connection.settimeout(10)

        def log_message(self, *_args):
            pass

        def respond(self, status, body, mime='application/json', headers=None):
            if not isinstance(body, bytes):
                body = dumps(body)
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' blob: data:; "
                             "style-src 'self' 'unsafe-inline'; frame-ancestors 'none'")
            for name, value in (headers or {}).items():
                self.send_header(name, str(value))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def authenticate(self, agent=False):
            auth = self.headers.get('Authorization', '')
            if agent:
                valid = auth.startswith('Bearer ') and hmac.compare_digest(
                    auth[7:].encode('utf-8'), token.encode('utf-8'))
            else:
                try:
                    user, password = base64.b64decode(auth[6:], validate=True).decode('utf-8').split(':', 1)
                    valid = auth.startswith('Basic ') and user == 'operator' and hmac.compare_digest(
                        password.encode('utf-8'), token.encode('utf-8'))
                except (ValueError, UnicodeError):
                    valid = False
            if not valid:
                # A rejected POST body is not consumed. Close instead of treating it as another request.
                self.close_connection = True
                self.respond(401, dict(error='인증이 필요합니다.'), headers={
                    'WWW-Authenticate': 'Bearer' if agent else 'Basic realm="AMR Central", charset="UTF-8"',
                    'Connection': 'close'})
            return valid

        def route(self):
            path = urlsplit(self.path).path
            parts = path.strip('/').split('/')
            if len(parts) >= 2 and parts[0] == 'robots':
                robot_id = identifier(parts[1])
                return robot_id, '/' + '/'.join(parts[2:])
            return default_robot, path

        def do_GET(self):
            if not self.authenticate():
                return
            try:
                robot_id, path = self.route()
                if path == '/api/robots':
                    self.respond(200, dict(robots=registry.roster()))
                elif path == '/api/state':
                    self.respond(200, registry.snapshot(robot_id))
                elif path == '/api/floors':
                    self.respond(200, registry.floors(robot_id))
                elif path.startswith('/api/floors/') and path.endswith('/map.png'):
                    name = path[len('/api/floors/'):-len('/map.png')]
                    png, version = registry.layer(robot_id, floor_layer(name))
                    self.respond(200, png, 'image/png', {'X-Map-Version': version})
                elif path.startswith('/api/layers/') and path.endswith('.png'):
                    name = path[len('/api/layers/'):-4]
                    if name not in LAYERS:
                        raise ApiError(404, 'Unknown layer')
                    png, version = registry.layer(robot_id, name)
                    self.respond(200, png, 'image/png', {'X-Map-Version': version})
                elif path in ('/fleet', '/fleet.js', '/fleet.css'):
                    name = 'fleet.html' if path == '/fleet' else path[1:]
                    mime = {'html': 'text/html; charset=utf-8', 'js': 'text/javascript; charset=utf-8',
                            'css': 'text/css; charset=utf-8'}[name.rsplit('.', 1)[1]]
                    self.respond(200, (fleet_assets / name).read_bytes(), mime)
                elif path in ('/', '/index.html', '/app.js', '/geometry.js', '/dashboard.js',
                              '/battery-profile.js', '/style.css', '/favicon.svg'):
                    name = 'index.html' if path == '/' else path[1:]
                    content = (assets / name).read_bytes()
                    if name == 'index.html':
                        content = content.decode('utf-8').replace('src="/app.js"',
                            f'src="/robots/{robot_id}/app.js"').replace('<a class="brand" href="#main">',
                            '<a class="brand" href="/fleet">').replace('href="/" aria-label="AMR 홈"',
                            'href="/fleet" aria-label="AMR 홈"').replace('목적지 주행 <span>',
                            f'{robot_id} · 목적지 주행 <span>').encode('utf-8')
                    elif name == 'app.js':
                        content = content.replace(b'/api/', f'/robots/{robot_id}/api/'.encode('ascii'))
                    mime = {'html': 'text/html; charset=utf-8', 'js': 'text/javascript; charset=utf-8',
                            'css': 'text/css; charset=utf-8', 'svg': 'image/svg+xml'}[name.rsplit('.', 1)[1]]
                    self.respond(200, content, mime)
                else:
                    raise ApiError(404, 'Not found')
            except ApiError as exc:
                self.respond(exc.status, dict(error=str(exc)))
            except ValueError as exc:
                self.respond(400, dict(error=str(exc)))

        def do_POST(self):
            agent = urlsplit(self.path).path == '/api/agent/exchange'
            if not self.authenticate(agent=agent):
                return
            try:
                origin = self.headers.get('Origin')
                if origin and urlsplit(origin).netloc != self.headers.get('Host'):
                    raise ApiError(403, '다른 웹사이트에서 보낸 명령은 허용하지 않습니다.')
                if not agent and self.headers.get('X-AMR-Request') != '1':
                    raise ApiError(403, '명령 요청 헤더가 필요합니다.')
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise ApiError(415, 'JSON 요청이 필요합니다.')
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= (MAX_MESSAGE if agent else MAX_COMMAND):
                    raise ApiError(413, '요청 크기가 유효하지 않습니다.')
                body = self.rfile.read(size)
                if len(body) != size:
                    raise ApiError(400, 'Incomplete request body')
                message = loads(body)
                if agent:
                    self.respond(200, registry.exchange(message))
                else:
                    robot_id, path = self.route()
                    if path not in ('/api/goal', '/api/initialpose', '/api/cancel', '/api/relocalize'):
                        raise ApiError(404, 'Not found')
                    command = registry.queue(robot_id, path.rsplit('/', 1)[1], message)
                    result = registry.wait_result(robot_id, command)
                    self.respond(result['http_status'], result['body'])
            except ApiError as exc:
                self.close_connection = True
                self.respond(exc.status, dict(error=str(exc)), headers={'Connection': 'close'})
            except (ValueError, TypeError, KeyError) as exc:
                self.close_connection = True
                self.respond(400, dict(error=str(exc)), headers={'Connection': 'close'})

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server


def main(args=None):
    parser = argparse.ArgumentParser(description='Central AMR host on a private Tailscale network')
    parser.add_argument('--host', default='127.0.0.1', help='Use the host PC Tailscale IP for remote access')
    parser.add_argument('--port', type=int, default=8081)
    parser.add_argument('--token-file', required=True)
    parser.add_argument('--robot-id', default='amr-01', help='Default robot shown at /')
    parser.add_argument('--assets-dir', help='Existing amr_web_ui/web directory')
    parser.add_argument('--stale-after', type=float, default=2)
    parser.add_argument('--command-timeout', type=float, default=3)
    options = parser.parse_args(args)
    try:
        registry = Registry(options.stale_after, options.command_timeout)
        server = create_server(registry, find_assets(options.assets_dir), read_token(options.token_file),
                               options.host, options.port, options.robot_id)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print(f'AMR central host: http://{options.host}:{server.server_port} (operator login required)', flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
