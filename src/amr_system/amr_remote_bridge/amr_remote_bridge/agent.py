"""Poll a local summary and initiate every connection to the central console."""
import argparse
from collections import OrderedDict
from datetime import datetime
import logging
import math
from pathlib import Path
import signal
import threading
import time
from uuid import uuid4
from zoneinfo import ZoneInfo

from .common import (COMMANDS, MAX_MESSAGE, MAX_SUMMARY, PROTOCOL_VERSION, REQUEST_ID,
                     LOCAL_COMMAND_PATHS, valid_command_payload, HttpClient, base_url, dumps, identifier, loads, positive, read_token)
from .summary import summarize

LOG = logging.getLogger(__name__)


class Agent:
    def __init__(self, server_url, token, robot_id='amr-01', local_url='http://127.0.0.1:8080',
                 rate=5, timeout=1.5, reconnect_max=10, stop=None):
        self.server_url = base_url(server_url)
        self.local_url = base_url(local_url, local=True)
        self.robot_id = identifier(robot_id)
        self.token = token
        self.interval = 1 / positive(rate)
        self.timeout = positive(timeout)
        self.reconnect_max = positive(reconnect_max)
        self.stop = stop or threading.Event()
        self.http = HttpClient()
        self.session_id = uuid4().hex
        self.sequence = 0
        self.results = OrderedDict()
        self.executed = OrderedDict()
        self.round_trip = 0
        self.exchange_metrics = {}

    def collect(self):
        captured = time.monotonic()
        # The kiosk snapshot already holds every summarised key and skips scans, paths and
        # camera frames, so the web UI serialises far less at 5 Hz.
        status, _, body = self.http.request(self.local_url + '/api/kiosk/state', timeout=self.timeout,
                                           limit=8 * 1024 * 1024)
        if status != 200:
            raise ValueError(f'Local web API returned HTTP {status}')
        state = loads(body)
        if not isinstance(state, dict):
            raise ValueError('Invalid local /api/state response')
        summary = summarize(state)
        if len(dumps(summary)) > MAX_SUMMARY:
            raise ValueError('Local summary exceeds 16 KiB')
        return summary, captured

    def execute(self, command, round_trip):
        if not isinstance(command, dict):
            raise ValueError('Invalid command from host')
        command_id = command.get('command_id')
        kind, payload, ttl = command.get('kind'), command.get('payload'), command.get('ttl')
        if (not isinstance(command_id, str) or not REQUEST_ID.fullmatch(command_id) or
                not isinstance(kind, str) or kind not in COMMANDS or
                not valid_command_payload(kind, payload) or payload['request_id'] != command_id or
                type(ttl) not in (int, float) or
                not math.isfinite(ttl)):
            raise ValueError('Invalid command from host')
        if command_id in self.executed:
            self.results[command_id] = self.executed[command_id]
            return
        if self.stop.is_set():
            result = dict(command_id=command_id, http_status=503,
                          body=dict(error='AMR 통신 클라이언트가 종료 중입니다. 명령을 전달하지 않았습니다.'))
        elif ttl <= round_trip:
            result = dict(command_id=command_id, http_status=408,
                          body=dict(error='명령이 만료되었습니다. 현재 상태를 확인하고 다시 요청해 주세요.'))
        else:
            try:
                status, _, body = self.http.request(
                    self.local_url + '/api/' + LOCAL_COMMAND_PATHS[kind], payload, {'X-AMR-Request': '1'},
                    timeout=min(self.timeout, ttl - round_trip), limit=64 * 1024)
                result_body = loads(body)
                if not isinstance(result_body, dict) or not 200 <= status <= 599:
                    raise ValueError('Invalid local command response')
                result = dict(command_id=command_id, http_status=status, body=result_body)
            except Exception:
                result = dict(command_id=command_id, http_status=504, body=dict(
                    error='로컬 명령 응답을 확인하지 못했습니다. 자동 재전송하지 않습니다. 상태를 확인해 주세요.'))
        self.executed[command_id] = result
        self.results[command_id] = result
        while len(self.executed) > 256:
            self.executed.popitem(last=False)

    def exchange(self):
        error = None
        self.exchange_metrics = dict(phase='local_state', host_http_status=None)
        try:
            summary, captured = self.collect()
        except Exception as exc:
            summary, captured = None, time.monotonic()
            error = type(exc).__name__ + ': local web API unavailable'
        self.sequence += 1
        envelope = dict(protocol=PROTOCOL_VERSION, robot_id=self.robot_id,
                        session_id=self.session_id, sequence=self.sequence,
                        state_age=time.monotonic() - captured + self.round_trip,
                        summary=summary, results=list(self.results.values()), local_error=error)
        payload = dumps(envelope)
        self.exchange_metrics.update(phase='host_exchange', payload_bytes=len(payload))
        if len(payload) > MAX_MESSAGE:
            raise ValueError('Exchange exceeds protocol size limit')
        started = time.monotonic()
        status, _, body = self.http.request(self.server_url + '/api/agent/exchange', payload,
                                          {'Authorization': 'Bearer ' + self.token}, timeout=self.timeout)
        round_trip = time.monotonic() - started
        self.exchange_metrics.update(host_http_status=status, round_trip_ms=round_trip*1000)
        reply = loads(body)
        if isinstance(reply, dict) and 'protocol' in reply and reply['protocol'] != PROTOCOL_VERSION:
            raise RuntimeError('Unsupported host protocol; protocol 2 is required')
        if status in (400, 409) and isinstance(reply, dict) and 'protocol' in str(reply.get('error', '')).lower():
            raise RuntimeError('Unsupported host protocol; protocol 2 is required')
        if status != 200:
            raise ConnectionError(f'Host rejected exchange: HTTP {status}')
        if not isinstance(reply, dict) or reply.get('protocol') != PROTOCOL_VERSION:
            raise RuntimeError('Unsupported host protocol; protocol 2 is required')
        if reply.get('session_id') != self.session_id or reply.get('sequence') != self.sequence:
            raise ValueError('Mismatched exchange acknowledgement')
        self.round_trip = round_trip
        for command_id in reply.get('accepted_results', []):
            self.results.pop(command_id, None)
        for command in reply.get('commands', []):
            self.execute(command, time.monotonic() - started)
        return summary is not None

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
                failures += 1
                if was_online or failures == 1:
                    LOG.warning('Host exchange unavailable (%s, metrics=%s); commands will not be replayed',
                                str(exc) if isinstance(exc, RuntimeError) else type(exc).__name__, self.exchange_metrics)
                was_online = False
                delay = min(self.reconnect_max, self.interval * 2 ** min(failures, 8))
            self.stop.wait(delay)


def main(args=None):
    parser = argparse.ArgumentParser(description='Outbound AMR client for a Tailscale central host')
    parser.add_argument('--server-url', required=True, help='http://<host Tailscale IP>:8081')
    parser.add_argument('--token-file', required=True)
    parser.add_argument('--robot-id', default='amr-01')
    parser.add_argument('--local-url', default='http://127.0.0.1:8080')
    parser.add_argument('--rate', type=float, default=5)
    parser.add_argument('--timeout', type=float, default=1.5)
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
                      options.local_url, options.rate, options.timeout)
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
