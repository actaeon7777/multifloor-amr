import base64
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import replace
import json
import socket
import time
from urllib.parse import urlsplit

import httpx
import pytest

from amr_remote_bridge.common import MAX_MESSAGE, loads
from app.config import Settings
from app.main import create_app
from .support import BASIC, TOKEN, TCPServer, wait_for


def payload(request_id='transport_goal_001', **values):
    return dict(request_id=request_id, x=.5, y=.5, yaw=.3, map_version=1, **values)


def envelope(harness, **values):
    return dict(protocol=1, robot_id='amr-01', session_id=harness.agent.session_id,
                sequence=harness.agent.sequence + 1, state=None, state_age=0,
                layers={}, results=[], local_error='test outage', **values)


def raw_post(harness, data, path='/api/agent/exchange', headers=None):
    default = {'Authorization': 'Bearer ' + TOKEN, 'Content-Type': 'application/json'}
    default.update(headers or {})
    with httpx.Client(trust_env=False, follow_redirects=False) as client:
        return client.post(harness.url + path, content=data, headers=default)


def test_waiting_robot_ui_and_security_headers(harness):
    for path in ('/', '/fleet', '/robots/amr-01/', '/fleet.js', '/style.css', '/favicon.svg'):
        status, headers, _ = harness.request(path)
        assert status == 200
        assert headers['Cache-Control'] == 'no-store'
        assert headers['X-Content-Type-Options'] == 'nosniff'
        assert "frame-ancestors 'none'" in headers['Content-Security-Policy']
    assert loads(harness.request('/api/robots')[2]) == {'robots': []}
    status, _, body = harness.request('/api/state')
    assert status == 503 and set(loads(body)) == {'error'}


@pytest.mark.parametrize('protocol', [True, 1.0, '1', None, 2])
def test_protocol_does_not_coerce_invalid_types(harness, protocol):
    message = envelope(harness)
    message['protocol'] = protocol
    response = raw_post(harness, json.dumps(message).encode())
    assert response.status_code == 400 and 'error' in response.json()


@pytest.mark.parametrize('bad_number', ['NaN', 'Infinity', '-Infinity', '1e400'])
def test_nonfinite_numbers_anywhere_are_rejected(harness, bad_number):
    body = json.dumps(envelope(harness))[:-1] + ',"extension":{"value":' + bad_number + '}}'
    response = raw_post(harness, body.encode())
    assert response.status_code == 400
    assert not harness.registry.robots


@pytest.mark.parametrize('body', [b'[]', b'null', b'{', b'"hello"', br'{"text":"\ud800"}'])
def test_invalid_json_returns_compatible_error(harness, body):
    response = raw_post(harness, body)
    assert response.status_code == 400 and set(response.json()) == {'error'}


def test_headers_body_limits_and_no_redirect(harness):
    assert raw_post(harness, b'{}', headers={'Content-Type': 'text/plain'}).status_code == 415
    assert raw_post(harness, b'{}', headers={'Origin': 'http://other.test'}).status_code == 403
    response = raw_post(harness, b'{}', path='/api/agent/exchange/')
    assert response.status_code == 401 and 'location' not in response.headers
    response = raw_post(harness, b'{}', headers={'Authorization': BASIC})
    assert response.status_code == 401 and response.headers['www-authenticate'] == 'Bearer'
    assert raw_post(harness, b'{}', path='/api/goal', headers={
        'Authorization': BASIC, 'X-AMR-Request': '1'}).status_code == 400
    for path, limit, authorization in (('/api/goal', 4096, BASIC),
                                        ('/api/agent/exchange', MAX_MESSAGE, 'Bearer ' + TOKEN)):
        address = urlsplit(harness.url)
        with socket.create_connection((address.hostname, address.port), timeout=2) as connection:
            connection.sendall((f'POST {path} HTTP/1.1\r\nHost: {address.netloc}\r\n'
                f'Authorization: {authorization}\r\nContent-Type: application/json\r\n'
                f'X-AMR-Request: 1\r\nContent-Length: {limit + 1}\r\n\r\n').encode())
            assert b' 413 ' in connection.recv(4096)


def test_invalid_robot_coordinates_and_methods(harness):
    harness.agent.exchange()
    assert harness.request('/robots/bad.id/api/state')[0] == 400
    assert harness.request('/api/layers/unknown.png')[0] == 404
    for key, value in (('x', True), ('y', '0'), ('yaw', None), ('map_version', True)):
        body = payload()
        body[key] = value
        assert harness.request('/api/goal', body)[0] == 400
    assert not harness.calls
    with httpx.Client(trust_env=False) as client:
        response = client.put(harness.url + '/api/state', headers={'Authorization': BASIC})
    assert response.status_code == 405 and set(response.json()) == {'error'}


def test_unknown_state_fields_ages_and_missing_images(harness):
    harness.shared.state['extension'] = {'valid': [3, 'keep me']}
    agent = harness.agent
    state, _layers, _captured = agent.collect()
    message = envelope(harness)
    message.update(state=state, state_age=.1, local_error=None)
    response = raw_post(harness, json.dumps(message).encode())
    assert response.status_code == 200
    assert set(response.json()['needed_layers']) == {'map', 'local_costmap'}
    status, _, body = harness.request('/api/state')
    snapshot = loads(body)
    assert status == 200 and snapshot['extension'] == {'valid': [3, 'keep me']}
    assert snapshot['robot']['age'] >= state['robot']['age'] + .1
    assert snapshot['layers'] == {}
    assert harness.request('/api/goal', payload())[0] == 409
    assert harness.request('/api/layers/map.png')[0] == 404


def test_invalid_layer_version_encoding_and_size(harness):
    state, layers, _ = harness.agent.collect()
    for change in ('version', 'encoding', 'size'):
        message = envelope(harness)
        message.update(state=state, layers=deepcopy(layers))
        if change == 'version':
            message['layers']['map']['version'] = 999
        elif change == 'encoding':
            message['layers']['map']['png'] = 'not base64!'
        else:
            png = b'\x89PNG\r\n\x1a\n' + b'x' * (2 * 1024 * 1024)
            message['layers']['map']['png'] = base64.b64encode(png).decode()
        assert raw_post(harness, json.dumps(message).encode()).status_code == 400
    assert not harness.registry.robots


def test_capacity_limit_and_strict_sequence(harness):
    harness.registry.max_robots = 1
    harness.agent.exchange()
    message = envelope(harness)
    message['robot_id'] = 'amr-02'
    assert raw_post(harness, json.dumps(message).encode()).status_code == 429
    message['robot_id'] = 'amr-01'
    for sequence in (True, '2', 0, 1):
        message['sequence'] = sequence
        assert raw_post(harness, json.dumps(message).encode()).status_code in (400, 409)


def test_expiry_and_lost_exchange_response_are_not_replayed(harness):
    harness.agent.exchange()
    with ThreadPoolExecutor(max_workers=1) as executor:
        queued = executor.submit(harness.request, '/api/goal', payload())
        wait_for(lambda: harness.registry.robots['amr-01'].commands)
        assert queued.result(timeout=2)[0] == 408
    harness.agent.exchange()
    assert not harness.calls

    # Host marked SENT, but the response was lost before the Agent could execute it.
    with ThreadPoolExecutor(max_workers=1) as executor:
        pending = executor.submit(harness.request, '/api/goal', payload('lost_reply_goal_001'))
        wait_for(lambda: 'lost_reply_goal_001' in harness.registry.robots['amr-01'].commands)
        state, layers, _ = harness.agent.collect()
        message = envelope(harness)
        message.update(state=state, layers=layers)
        response = raw_post(harness, json.dumps(message).encode())
        assert response.json()['commands'][0]['command_id'] == 'lost_reply_goal_001'
        harness.agent.sequence = message['sequence']
        assert pending.result(timeout=2)[0] == 504
    harness.agent.exchange()
    assert not harness.calls


def test_server_restart_requests_cache_and_acks_old_results(harness):
    harness.agent.layer_refresh = 60  # Force needed_layers recovery rather than periodic refresh.
    harness.agent.exchange()
    with ThreadPoolExecutor(max_workers=1) as executor:
        pending = executor.submit(harness.request, '/api/goal', payload())
        wait_for(lambda: harness.registry.robots['amr-01'].commands)
        harness.agent.exchange()
        assert len(harness.calls) == 1
        assert pending.result(timeout=2)[0] == 504
    assert harness.agent.results
    harness.restart_central()
    assert harness.request('/api/state')[0] == 503
    # The unchanged Agent first omits its acknowledged cached layer; host asks for it again.
    harness.agent.exchange()
    assert not harness.agent.results and not harness.agent.layer_cache
    harness.agent.exchange()
    assert loads(harness.request('/api/state')[2])['layers']['map']['version'] == 1
    assert len(harness.calls) == 1
    assert not harness.registry.robots['amr-01'].commands


def test_session_change_finishes_pending_request_without_delivery(harness):
    harness.agent.exchange()
    with ThreadPoolExecutor(max_workers=1) as executor:
        pending = executor.submit(harness.request, '/api/goal', payload())
        wait_for(lambda: harness.registry.robots['amr-01'].commands)
        time.sleep(.42)
        replacement = harness.make_agent()
        replacement.exchange()
        assert pending.result(timeout=2)[0] == 504
    assert not harness.calls


def test_map_changes_between_queue_and_delivery_are_rejected_locally(harness):
    harness.agent.exchange()
    with ThreadPoolExecutor(max_workers=1) as executor:
        pending = executor.submit(harness.request, '/api/goal', payload('changing_map_goal_001'))
        wait_for(lambda: harness.registry.robots['amr-01'].commands)
        grid = harness.shared.grids['map']
        grid.update(0, 1, 1, 1, [100])
        grid.version = 17  # Same source version, different PNG digest.
        harness.agent.layer_cache.clear()
        harness.agent.exchange()
        harness.agent.exchange()
        assert pending.result(timeout=2)[0] == 409
    assert not harness.calls
    assert loads(harness.request('/api/state')[2])['layers']['map']['version'] == 2


def test_second_robot_command_does_not_enter_default_robot_queue(harness):
    harness.agent.exchange()
    second = harness.make_agent('amr-02')
    second.exchange()
    with ThreadPoolExecutor(max_workers=1) as executor:
        pending = executor.submit(harness.request, '/robots/amr-02/api/goal', payload('robot_two_goal_001'))
        wait_for(lambda: harness.registry.robots['amr-02'].commands)
        assert not harness.registry.robots['amr-01'].commands
        harness.agent.exchange()
        assert not harness.calls and not pending.done()
        second.exchange()
        second.exchange()
        assert pending.result(timeout=2)[0] == 202
    assert len(harness.calls) == 1


def test_body_timeout_keeps_exchange_available(harness):
    harness.central.stop()
    settings = replace(harness.settings, body_timeout=.2)
    harness.central = TCPServer(create_app(settings, harness.registry)).start()
    harness.agent.server_url = harness.url
    address = urlsplit(harness.url)
    with socket.create_connection((address.hostname, address.port), timeout=2) as connection:
        connection.sendall((f'POST /api/agent/exchange HTTP/1.1\r\nHost: {address.netloc}\r\n'
            f'Authorization: Bearer {TOKEN}\r\nContent-Type: application/json\r\n'
            'Content-Length: 1000\r\n\r\n').encode())
        harness.agent.exchange()
        assert harness.request('/api/state')[0] == 200
        assert b' 408 ' in connection.recv(4096)


def test_startup_rejects_bad_token_missing_assets_and_invalid_settings(tmp_path):
    token = tmp_path / 'test.token'
    token.write_text('short')
    settings = replace(Settings(), token_file=token)
    with pytest.raises(ValueError, match='Token file'):
        settings.validate()
    token.write_text(TOKEN)
    for values in ({'assets_dir': tmp_path}, {'fleet_dir': tmp_path}, {'robot_id': 'bad.id'},
                   {'command_timeout': 0}, {'stale_after': float('nan')}, {'max_robots': 0}):
        with pytest.raises(ValueError):
            replace(settings, **values).validate()
