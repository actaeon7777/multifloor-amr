"""Poll the unchanged local web API and initiate every connection to the host."""
import argparse
import base64
from collections import OrderedDict
import hashlib
import logging
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
import math
import signal
import threading
import time
from uuid import uuid4

from .common import (COMMANDS, LAYERS, MAX_COMMAND, MAX_FLOORS, MAX_MESSAGE, PROTOCOL_VERSION,
                     REQUEST_ID, HttpClient, base_url, dumps, floor_layer, floor_map_version,
                     identifier, layer_floor, loads, positive, read_token, valid_layer, CAMERA_MAX_AGE, JPEG_MAGIC, MAX_CAMERA)


LOG = logging.getLogger(__name__)


class Agent:
    def __init__(self, server_url, token, robot_id='amr-01', local_url='http://127.0.0.1:8080',
                 rate=5, timeout=1.5, layer_refresh=60, reconnect_max=10, stop=None, camera_refresh=1.):
        self.server_url = base_url(server_url)
        self.local_url = base_url(local_url, local=True)
        self.robot_id = identifier(robot_id)
        self.token = token
        self.interval = 1 / positive(rate)
        self.timeout = positive(timeout)
        self.layer_refresh = positive(layer_refresh)
        self.camera_refresh = positive(camera_refresh)
        self.camera_sent = -math.inf
        self.camera_digest = None
        self.reconnect_max = positive(reconnect_max)
        self.stop = stop or threading.Event()
        self.http = HttpClient()
        self.session_id = uuid4().hex
        self.sequence = 0
        self.layer_cache = {}
        self.results = OrderedDict()
        self.executed = OrderedDict()
        self.round_trip = 0
        self.layer_attempts = {}
        self.exchange_metrics = {}
        # Multi-floor: the local /api/floors document (None without floor support).
        self.floors = None
        self.floors_checked = None
        self.state_body = None

    def layer_url(self, name):
        floor = layer_floor(name)
        return self.local_url + (f'/api/floors/{floor}/map.png' if floor else f'/api/layers/{name}.png')

    def layer_version(self, name, state):
        floor = layer_floor(name)
        if floor:
            return floor_map_version(self.floors, floor)
        return state['layers'].get(name, {}).get('version')

    def read_floors(self):
        """Refresh the floor list at the layer cadence; a web UI without floors answers 404."""
        now = time.monotonic()
        if self.floors_checked is not None and now - self.floors_checked < self.layer_refresh:
            return
        self.floors_checked = now
        try:
            status, _, body = self.http.request(self.local_url + '/api/floors', timeout=self.timeout,
                                                limit=256 * 1024)
            floors = loads(body) if status == 200 else None
            if floors is not None and not (isinstance(floors, dict) and isinstance(floors.get('floors'), list)
                                           and len(floors['floors']) <= MAX_FLOORS):
                raise ValueError('Invalid local /api/floors response')
        except Exception as exc:
            LOG.warning('Local floor list unavailable (%s); state exchange continues', type(exc).__name__)
            floors = None
        self.floors = floors

    def collect(self):
        def read_state():
            captured = time.monotonic()
            status, _, body = self.http.request(self.local_url + '/api/state', timeout=self.timeout)
            if status != 200:
                raise ValueError(f'Local web API returned HTTP {status}')
            state = loads(body)
            if not isinstance(state, dict) or not isinstance(state.get('layers'), dict):
                raise ValueError('Invalid local /api/state response')
            self.state_body = body
            return state, captured

        started = time.monotonic()
        self.exchange_metrics = dict(phase='local_state', host_http_status=None)
        self.read_floors()
        state, captured = read_state()
        updates = {}
        pending = []
        for name in sorted(state['layers'], key=lambda name: (name != 'map', name)):
            if name not in LAYERS:
                continue
            info = state['layers'][name]
            cached = self.layer_cache.get(name)
            if not cached or cached[0] != info.get('version') or time.monotonic()-cached[1] >= self.layer_refresh:
                pending.append((name, info))
        # Floor maps are static files: upload once per version, or again when the host asks.
        for entry in (self.floors or {}).get('floors', ()):
            if not isinstance(entry, dict) or not isinstance(entry.get('map'), dict):
                continue
            try:
                name = floor_layer(entry.get('name'))
            except ValueError:
                continue
            cached = self.layer_cache.get(name)
            if not cached or cached[0] != entry['map'].get('version'):
                pending.append((name, entry['map']))
        # Large/rolling PNGs must not age a single state through several serial
        # image requests. Serve one pending layer per cycle, least recently tried
        # first, then capture the current state. Images whose version changed are discarded.
        image_started = time.monotonic()
        camera = None
        if not pending and self.camera_due(state):
            camera = self.read_camera()
        if pending:
            # The map is needed to validate coordinate commands after restart.
            name, info = next((item for item in pending if item[0] == 'map'),
                              min(pending, key=lambda item: self.layer_attempts.get(item[0], 0)))
            self.layer_attempts[name] = time.monotonic()
            try:
                status, headers, png = self.http.request(self.layer_url(name), timeout=self.timeout,
                                                         limit=2 * 1024 * 1024)
                actual = next((v for k, v in headers.items() if k.lower() == 'x-map-version'), None)
                if status == 200 and actual == str(info.get('version')) and png.startswith(b'\x89PNG\r\n\x1a\n'):
                    updates[name] = dict(version=info['version'], png=base64.b64encode(png).decode('ascii'))
            except Exception as exc:
                LOG.warning('Local map layer %s unavailable (%s); state exchange continues', name, type(exc).__name__)
        if pending:
            # Preserve freshness/version checks after potentially slow PNG work.
            # Camera-only cycles reuse the already parsed state and raw body.
            state, captured = read_state()
            updates = {name: image for name, image in updates.items()
                       if self.layer_version(name, state) == image['version']}
        self.exchange_metrics = dict(phase='host_exchange', collect_ms=1000*(time.monotonic()-started),
                                     image_ms=1000*(time.monotonic()-image_started),host_http_status=None)
        return state, updates, captured, camera

    def camera_due(self, state):
        """A fresh camera frame exists locally and the relay interval has passed."""
        camera = state.get('camera')
        return (isinstance(camera, dict) and isinstance(camera.get('age'), (int, float))
                and camera['age'] < CAMERA_MAX_AGE
                and time.monotonic() - self.camera_sent >= self.camera_refresh)

    def read_camera(self):
        """The local JPEG snapshot as an envelope entry, or None when unchanged or unavailable."""
        self.camera_sent = time.monotonic()
        try:
            status, _, jpeg = self.http.request(self.local_url + '/api/camera.jpg', timeout=self.timeout,
                                                limit=MAX_CAMERA)
        except Exception as exc:
            LOG.warning('Local camera snapshot unavailable (%s); state exchange continues', type(exc).__name__)
            return None
        if status != 200 or not jpeg.startswith(JPEG_MAGIC):
            return None
        digest = hashlib.sha256(jpeg).hexdigest()
        if digest == self.camera_digest:
            return None
        self.camera_digest = digest
        return dict(jpeg=base64.b64encode(jpeg).decode('ascii'))

    def execute(self, command, round_trip):
        command_id = command.get('command_id')
        if not isinstance(command_id, str) or not REQUEST_ID.fullmatch(command_id):
            raise ValueError('Invalid command_id from host')
        if command_id in self.executed:
            self.results[command_id] = self.executed[command_id]
            return
        kind, payload, ttl = command.get('kind'), command.get('payload'), command.get('ttl')
        # Hosts before floor support omit map_layer: the pose then refers to the live map.
        map_layer = command.get('map_layer', 'map')
        if (kind not in COMMANDS or not isinstance(payload, dict) or
                not isinstance(ttl, (int, float)) or isinstance(ttl, bool) or
                not math.isfinite(ttl) or len(dumps(payload)) > MAX_COMMAND or
                payload.get('request_id') != command_id or not valid_layer(map_layer)):
            raise ValueError('Invalid command from host')
        # Subtract the entire exchange RTT; this is conservative and needs no clock synchronization.
        if self.stop.is_set():
            result = dict(command_id=command_id, http_status=503,
                          body=dict(error='AMR 통신 클라이언트가 종료 중입니다. 명령을 전달하지 않았습니다.'))
        elif ttl <= round_trip:
            result = dict(command_id=command_id, http_status=408,
                          body=dict(error='명령이 만료되었습니다. 현재 상태를 확인하고 다시 요청해 주세요.'))
        else:
            try:
                local_started = time.monotonic()
                if kind in ('goal', 'initialpose') or (kind == 'relocalize' and 'map_version' in payload):
                    status, headers, png = self.http.request(
                        self.layer_url(map_layer), timeout=min(self.timeout, ttl - round_trip),
                        limit=2 * 1024 * 1024)
                    version = next((v for k, v in headers.items() if k.lower() == 'x-map-version'), None)
                    if (status != 200 or version != str(payload.get('map_version')) or
                            hashlib.sha256(png).hexdigest() != command.get('map_digest')):
                        raise MapChanged()
                remaining = ttl - round_trip - (time.monotonic() - local_started)
                if remaining <= 0:
                    raise CommandExpired()
                status, _, body = self.http.request(
                    self.local_url + '/api/' + kind, payload,
                    {'X-AMR-Request': '1'}, timeout=min(self.timeout, remaining),
                    limit=64 * 1024)
                result = dict(command_id=command_id, http_status=status, body=loads(body))
            except MapChanged:
                result = dict(command_id=command_id, http_status=409, body=dict(
                    error='AMR의 지도가 변경되었습니다. 지도를 확인한 뒤 목적지를 다시 선택해 주세요.'))
            except CommandExpired:
                result = dict(command_id=command_id, http_status=408, body=dict(error='명령이 만료되었습니다.'))
            except Exception:
                # A failed HTTP response does not prove the ROS request wasn't received.
                result = dict(command_id=command_id, http_status=504, body=dict(
                    error='로컬 명령 응답을 확인하지 못했습니다. 자동 재전송하지 않습니다. 상태를 확인해 주세요.'))
        self.executed[command_id] = result
        self.results[command_id] = result
        while len(self.executed) > 256:
            self.executed.popitem(last=False)

    def exchange(self):
        error = None
        try:
            state, updates, captured, camera = self.collect()
        except Exception as exc:
            state, updates, captured, camera = None, {}, time.monotonic(), None
            self.layer_cache.clear()
            self.camera_digest = None
            self.floors, self.floors_checked = None, None
            # The central UI must become unavailable, rather than keep old data looking live.
            error = type(exc).__name__ + ': local web API unavailable'
        self.sequence += 1
        envelope = dict(protocol=PROTOCOL_VERSION, robot_id=self.robot_id,
                        session_id=self.session_id, sequence=self.sequence,
                        state_age=time.monotonic() - captured + self.round_trip,
                        layers=updates, results=list(self.results.values()), local_error=error,
                        floors=self.floors if state is not None else None, camera=camera)
        # The state was parsed once for layer/camera metadata. Relay its original
        # JSON unchanged instead of walking the scan again for size and request.
        state_body = self.state_body if state is not None else b'null'
        payload = dumps(envelope)[:-1] + b',"state":' + state_body + b'}'
        self.exchange_metrics.update(payload_bytes=len(payload), state_age=envelope['state_age'])
        if self.exchange_metrics['payload_bytes'] > MAX_MESSAGE:
            raise ValueError('Telemetry exceeds 8 MiB; reduce local map/scan data size')
        started = time.monotonic()
        status, _, body = self.http.request(self.server_url + '/api/agent/exchange', payload,
                                          {'Authorization': 'Bearer ' + self.token}, timeout=self.timeout)
        round_trip = time.monotonic() - started
        self.exchange_metrics.update(host_http_status=status, round_trip_ms=round_trip*1000)
        if status != 200:
            self.layer_cache.clear()
            self.camera_digest = None
            # Deliberately omit the URL, credential and returned body from logs.
            raise ConnectionError(f'Host rejected exchange: HTTP {status}')
        reply = loads(body)
        if (reply.get('protocol') != PROTOCOL_VERSION or reply.get('session_id') != self.session_id or
                reply.get('sequence') != self.sequence):
            raise ValueError('Mismatched exchange acknowledgement')
        self.round_trip = round_trip
        for command_id in reply.get('accepted_results', []):
            self.results.pop(command_id, None)
        for name in reply.get('accepted_layers', []):
            if name in updates:
                self.layer_cache[name] = (updates[name]['version'], time.monotonic())
        for name in reply.get('needed_layers', []):
            self.layer_cache.pop(name, None)
        for command in reply.get('commands', []):
            self.execute(command, time.monotonic() - started)
        return state is not None

    def run(self):
        failures, was_online, previous_local = 0, False, None
        last_metrics_log = 0.
        while not self.stop.is_set():
            started = time.monotonic()
            try:
                local_online = self.exchange()
                if not was_online:
                    LOG.info('Connected to central host as %s', self.robot_id)
                if not local_online and previous_local is not False:
                    LOG.warning('Local AMR web API is unavailable; remote control is disabled (metrics=%s)', self.exchange_metrics)
                if time.monotonic()-last_metrics_log >= 5.:
                    LOG.info('Exchange metrics: %s', self.exchange_metrics)
                    last_metrics_log = time.monotonic()
                previous_local = local_online
                was_online, failures = True, 0
                delay = max(0, self.interval - (time.monotonic() - started))
            except Exception as exc:
                self.layer_cache.clear()
                failures += 1
                if was_online or failures == 1:
                    LOG.warning('Host exchange unavailable (%s, metrics=%s); commands will not be replayed',
                                type(exc).__name__, self.exchange_metrics)
                was_online = False
                delay = min(self.reconnect_max, self.interval * 2 ** min(failures, 8))
            self.stop.wait(delay)


class MapChanged(Exception):
    pass


class CommandExpired(Exception):
    pass


def main(args=None):
    parser = argparse.ArgumentParser(description='Outbound AMR client for a Tailscale central host')
    parser.add_argument('--server-url', required=True, help='http://<host Tailscale IP>:8081')
    parser.add_argument('--token-file', required=True)
    parser.add_argument('--robot-id', default='amr-01')
    parser.add_argument('--local-url', default='http://127.0.0.1:8080')
    parser.add_argument('--rate', type=float, default=5)
    parser.add_argument('--timeout', type=float, default=1.5)
    parser.add_argument('--layer-refresh', type=float, default=60)
    parser.add_argument('--camera-refresh', type=float, default=1,
                        help='Seconds between relayed camera snapshots (JPEG, <= 300 KiB)')
    parser.add_argument('--log-file', type=Path, help='Diagnostic file; defaults to diagnostics/remote_client under the working directory')
    options = parser.parse_args(args)
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    log_file = options.log_file or (Path.cwd() / 'diagnostics/remote_client' /
        (datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S') + '.log'))
    log_file.parent.mkdir(parents=True, exist_ok=True)
    file_handler = logging.FileHandler(log_file, encoding='utf-8')
    file_handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
    LOG.setLevel(logging.INFO)
    LOG.addHandler(file_handler)
    LOG.info('Remote client diagnostic log: %s', log_file)
    try:
        agent = Agent(options.server_url, read_token(options.token_file), options.robot_id,
                      options.local_url, options.rate, options.timeout, options.layer_refresh,
                      camera_refresh=options.camera_refresh)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_args: agent.stop.set())
    try:
        agent.run()
    finally:
        LOG.removeHandler(file_handler)
        file_handler.close()


if __name__ == '__main__':
    main()
