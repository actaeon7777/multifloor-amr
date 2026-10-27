"""Stdlib central console: small robot summaries and three explicit commands."""
import argparse
import base64
from collections import OrderedDict
from copy import deepcopy
from dataclasses import dataclass, field
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import logging
import math
from pathlib import Path
import re
import threading
import time
from urllib.parse import urlsplit

from .common import (COMMANDS, MAX_COMMAND, MAX_MESSAGE, MAX_SUMMARY, PROTOCOL_VERSION,
                     REQUEST_ID, dumps, identifier, loads, positive, read_token)
from .slack_notify import SlackNotifier


class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


# Null is permitted for missing source fields; unknown fields never enter the cache.
SUMMARY_SCHEMA = {
    'floor': str, 'zone': (str, int), 'task': str, 'mode': str, 'navigation': str,
    'robot': dict.fromkeys(('x', 'y', 'yaw', 'age'), 'number'),
    'estop': {'state': str, 'healthy': bool, 'detail': str, 'error': str},
    'hold': {'held': bool, 'since': 'number', 'reason': str},
    'goal': dict.fromkeys(('request_id', 'kind', 'status', 'floor', 'room', 'origin', 'error_message', 'phase'), str),
    'floor_mission': {'phase': str, 'detail': str},
    'battery': {'soc': 'number', 'voltage': 'number', 'valid': bool, 'fresh': bool, 'charge_state': (int, str)},
    'localization': {'ready': bool, 'age': 'number'},
    'notice': {'request_id': str, 'status': str, 'detail': str},
    'pending_request': {'request_id': str, 'kind': str},
}


def validate_summary(summary):
    if summary is None:
        return
    if not isinstance(summary, dict):
        raise ApiError(400, 'Invalid summary: expected object or null')
    try:
        size = len(dumps(summary))
    except (TypeError, ValueError, RecursionError):
        raise ApiError(400, 'Invalid summary JSON') from None
    if size > MAX_SUMMARY:
        raise ApiError(413, 'Summary exceeds 16 KiB')

    def check(value, schema, path):
        if value is None:
            return
        if isinstance(schema, dict):
            if not isinstance(value, dict) or value.keys() - schema.keys():
                raise ApiError(400, f'Invalid summary field: {path}')
            for key, item in value.items():
                check(item, schema[key], path + '.' + key)
        elif schema == 'number':
            if not _number(value) or (path.endswith(('.age', '.since')) and value < 0):
                raise ApiError(400, f'Invalid summary number: {path}')
        elif type(value) not in (schema if isinstance(schema, tuple) else (schema,)):
            raise ApiError(400, f'Invalid summary type: {path}')
        elif isinstance(value, str) and len(value) > 4096:
            raise ApiError(400, f'Summary text too long: {path}')
    check(summary, SUMMARY_SCHEMA, 'summary')


@dataclass
class Command:
    command_id: str
    kind: str
    signature: dict
    payload: dict
    deadline: float
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
    summary: dict | None = None
    local_error: str | None = None
    commands: OrderedDict = field(default_factory=OrderedDict)


class Registry:
    def __init__(self, stale_after=2, command_timeout=3, lease_timeout=3, max_robots=32, notifier=None):
        self.lock = threading.RLock()
        self.robots = {}
        self.stale_after = positive(stale_after)
        self.command_timeout = positive(command_timeout)
        self.lease_timeout = positive(lease_timeout)
        if type(max_robots) is not int or max_robots < 1:
            raise ValueError('max_robots must be a positive integer')
        self.max_robots = max_robots
        self.notifier = notifier
        # Includes terminal-first goals: a later regressive sample must not arm them.
        self.arrivals = OrderedDict()

    def _fresh(self, robot, now):
        return (robot.summary is not None and robot.captured is not None and
                robot.last_exchange is not None and now - robot.captured < self.stale_after and
                now - robot.last_exchange < self.stale_after)

    @staticmethod
    def _finish(command, status, body):
        command.result = dict(http_status=status, body=deepcopy(body))
        command.status = 'RESULT'
        command.event.set()

    def _expire(self, robot, now):
        for command in robot.commands.values():
            if command.result is None and now >= command.deadline:
                sent = command.status == 'SENT'
                self._finish(command, 504 if sent else 408, dict(error=
                    '명령 응답을 확인하지 못했습니다. 자동 재전송하지 않습니다. 현재 상태를 확인해 주세요.'
                    if sent else '전달 전에 명령이 만료되었습니다. 현재 상태를 확인하고 다시 요청해 주세요.'))

    def _arrival(self, robot):
        goal = (robot.summary or {}).get('goal')
        if not isinstance(goal, dict):
            return
        request_id, status = goal.get('request_id'), goal.get('status')
        if not request_id or not status:
            return
        key = (robot.robot_id, request_id)
        terminal = status in ('SUCCEEDED', 'ABORTED', 'CANCELED', 'CANCELLED', 'REJECTED', 'FAILED', 'ERROR')
        seen = self.arrivals.get(key)
        if terminal:
            self.arrivals[key] = 'terminal'
            if (seen == 'active' and status == 'SUCCEEDED' and goal.get('room') and
                    goal.get('origin') != 'return_home' and self.notifier):
                self.notifier.arrival(robot.robot_id, goal['room'], request_id)
        elif seen is None:
            self.arrivals[key] = 'active'
        self.arrivals.move_to_end(key)
        while len(self.arrivals) > 256:
            self.arrivals.popitem(last=False)

    def exchange(self, message):
        if not isinstance(message, dict):
            raise ApiError(400, 'Invalid exchange object')
        if type(message.get('protocol')) is not int or message['protocol'] != PROTOCOL_VERSION:
            raise ApiError(409, 'Unsupported agent protocol; protocol 2 is required')
        if message.keys() - {'protocol', 'robot_id', 'session_id', 'sequence', 'state_age',
                             'summary', 'local_error', 'results'}:
            raise ApiError(400, 'Unexpected protocol 2 exchange fields')
        robot_id = identifier(message.get('robot_id'))
        session_id, sequence = message.get('session_id'), message.get('sequence')
        if (not isinstance(session_id, str) or not re.fullmatch(r'[0-9a-f]{32}', session_id) or
                type(sequence) is not int or sequence < 1):
            raise ApiError(400, 'Invalid session or sequence')
        summary, age = message.get('summary'), message.get('state_age')
        if not _number(age) or not 0 <= age <= 3600:
            raise ApiError(400, 'Invalid telemetry age')
        validate_summary(summary)
        error = message.get('local_error')
        if error is not None and (not isinstance(error, str) or len(error) > 1024):
            raise ApiError(400, 'Invalid local_error')
        results = message.get('results', [])
        if not isinstance(results, list) or len(results) > 256:
            raise ApiError(400, 'Invalid command results')
        for result in results:
            if (not isinstance(result, dict) or not isinstance(result.get('command_id'), str) or
                    not REQUEST_ID.fullmatch(result['command_id']) or
                    type(result.get('http_status')) is not int or not 200 <= result['http_status'] <= 599 or
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
            if sequence <= robot.sequence:
                raise ApiError(409, 'Exchange sequence has already been used')
            robot.sequence, robot.last_exchange = sequence, now
            robot.summary = deepcopy(summary)
            robot.captured = now - age if summary is not None else None
            robot.local_error = error
            self._arrival(robot)
            self._expire(robot, now)
            accepted_results = []
            for result in results:
                command_id = result['command_id']
                command = robot.commands.get(command_id)
                if command is not None and command.status == 'SENT' and command.result is None:
                    self._finish(command, result['http_status'], result['body'])
                accepted_results.append(command_id)
            commands = []
            if self._fresh(robot, now):
                for command in robot.commands.values():
                    if command.status == 'QUEUED':
                        command.status = 'SENT'
                        commands.append(dict(command_id=command.command_id, kind=command.kind,
                                             payload=deepcopy(command.payload), ttl=command.deadline - now))
                        break
            return dict(protocol=PROTOCOL_VERSION, session_id=session_id, sequence=sequence,
                        commands=commands, accepted_results=accepted_results)

    def queue(self, robot_id, kind, payload):
        if not isinstance(kind, str) or kind not in COMMANDS or not isinstance(payload, dict):
            raise ApiError(400, 'Invalid command')
        request_id = payload.get('request_id')
        if (not isinstance(request_id, str) or not REQUEST_ID.fullmatch(request_id) or
                payload != {'request_id': request_id}):
            raise ApiError(400, '요청 ID만 포함한 명령이 필요합니다.')
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
            if not self._fresh(robot, now):
                raise ApiError(503, 'AMR 연결 또는 상태 데이터가 지연되었습니다. 명령을 보낼 수 없습니다.')
            self._expire(robot, now)
            if any(command.result is None for command in robot.commands.values()):
                raise ApiError(409, '이전 원격 요청을 처리 중입니다. 잠시 기다려 주세요.')
            command = Command(request_id, kind, signature, deepcopy(payload), now + self.command_timeout)
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
            result = []
            for robot in self.robots.values():
                online = self._fresh(robot, now)
                summary = deepcopy(robot.summary) if online else None
                if summary:
                    for name in ('robot', 'localization'):
                        item = summary.get(name)
                        if isinstance(item, dict) and _number(item.get('age')):
                            item['age'] += now - robot.captured
                    localization = summary.get('localization')
                    if localization and _number(localization.get('age')) and localization['age'] >= 2:
                        localization['ready'] = False
                result.append(dict(robot_id=robot.robot_id, online=online,
                                   age=now-robot.last_exchange if robot.last_exchange is not None else None,
                                   local_api_available=robot.summary is not None, summary=summary))
            return result


def create_server(registry, token, host='127.0.0.1', port=8081, default_robot='amr-01', body_timeout=10):
    default_robot = identifier(default_robot)
    body_timeout = positive(body_timeout)
    assets = Path(__file__).resolve().parents[1] / 'web'
    if not assets.is_dir():
        from ament_index_python.packages import get_package_share_directory
        assets = Path(get_package_share_directory('amr_remote_bridge')) / 'web'

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def setup(self):
            super().setup()
            self.connection.settimeout(body_timeout)

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
                if self.path.split('?')[0] == '/api/robots':
                    self.respond(200, registry.roster())
                elif self.path.split('?')[0] in ('/', '/console.js', '/console.css'):
                    name = 'console.html' if path == '/' else path[1:]
                    mime = {'html': 'text/html; charset=utf-8', 'js': 'text/javascript; charset=utf-8',
                            'css': 'text/css; charset=utf-8'}[name.rsplit('.', 1)[1]]
                    self.respond(200, (assets / name).read_bytes(), mime)
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
                    if path not in tuple('/api/' + kind for kind in COMMANDS):
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
    parser = argparse.ArgumentParser(description='Lightweight central AMR console')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8081)
    parser.add_argument('--token-file', required=True)
    parser.add_argument('--robot-id', default='amr-01')
    parser.add_argument('--stale-after', type=float, default=2)
    parser.add_argument('--command-timeout', type=float, default=3)
    parser.add_argument('--slack-env')
    parser.add_argument('--slack-rooms')
    options = parser.parse_args(args)
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    notifier = None
    try:
        token = read_token(options.token_file)
        notifier = SlackNotifier(slack_env=options.slack_env, slack_rooms=options.slack_rooms)
        registry = Registry(options.stale_after, options.command_timeout, notifier=notifier)
        server = create_server(registry, token, options.host, options.port, options.robot_id)
    except (ValueError, OSError) as exc:
        if notifier:
            notifier.close()
        parser.error(str(exc))
    print(f'AMR central host: http://{options.host}:{server.server_port}/ (operator login required)', flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        notifier.close()


if __name__ == '__main__':
    main()
