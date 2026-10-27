"""Protocol 2 over loopback HTTP; commands go only to a recording HTTP stub."""
import base64
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

PACKAGES = Path(__file__).resolve().parents[2]
for package in ('amr_remote_bridge', 'amr_web_ui', 'amr_control'):
    sys.path.insert(0, str(PACKAGES / package))

import pytest
from amr_remote_bridge.agent import Agent
from amr_remote_bridge.common import HttpClient, PROTOCOL_VERSION, base_url, dumps, loads
from amr_remote_bridge.server import ApiError, Registry, create_server
from amr_remote_bridge.summary import summarize

TOKEN = 'test-only-secret-' + 'a' * 32
BASIC = 'Basic ' + base64.b64encode(('operator:' + TOKEN).encode()).decode()
SESSION = 'a' * 32


def message(sequence=1, **kwargs):
    return dict(protocol=PROTOCOL_VERSION, robot_id='amr-01', session_id=SESSION, sequence=sequence,
                state_age=0, summary=summarize({}), results=[], local_error=None) | kwargs


def wait_for(predicate, seconds=3):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(.01)
    raise AssertionError('Timed out waiting for relay')


@pytest.fixture
def system():
    # Another worker may be writing these modules concurrently. Never call its command endpoints.
    for attempt in range(3):
        try:
            from amr_web_ui.server import SharedState
            break
        except (ImportError, SyntaxError):
            if attempt == 2:
                raise
            time.sleep(2)
    shared = SharedState(preview=True)
    shared.state.update(robot=dict(x=.5, y=.25, yaw=.3, received_at=time.monotonic()),
                        floor=dict(floor='4F', zone=1), mode='MANUAL', navigation='IDLE',
                        task='401호 배송', hold=dict(held=True, since=0, reason='operator'))
    calls, gets = [], []
    local_state = {'available': True, 'response_status': 202}

    class LocalStub(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def respond(self, status, body):
            body = dumps(body)
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            gets.append(self.path)
            if self.path == '/api/kiosk/state' and local_state['available']:
                self.respond(200, shared.snapshot())
            else:
                self.respond(503, {'error': 'local unavailable'})

        def do_POST(self):
            payload = loads(self.rfile.read(int(self.headers['Content-Length'])))
            calls.append((self.path, payload, self.headers.get('X-AMR-Request'),
                          self.headers.get('Content-Type')))
            self.respond(local_state['response_status'], dict(request_id=payload['request_id'], held=True))

    registry = Registry(stale_after=.8, command_timeout=.5, lease_timeout=.4)
    local = ThreadingHTTPServer(('127.0.0.1', 0), LocalStub)
    central = create_server(registry, TOKEN, port=0)
    threads = []
    for server in (local, central):
        thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
        thread.start()
        threads.append(thread)
    url = f'http://127.0.0.1:{central.server_port}'
    agent = Agent(url, TOKEN, local_url=f'http://127.0.0.1:{local.server_port}', rate=30, timeout=.3)
    agent_thread = None

    def start():
        nonlocal agent_thread
        agent_thread = threading.Thread(target=agent.run, daemon=True)
        agent_thread.start()
        wait_for(lambda: registry.roster() and registry.roster()[0]['online'])

    def request(path, payload=None, headers=None):
        auth = {'Authorization': BASIC}
        if payload is not None:
            auth['X-AMR-Request'] = '1'
        auth.update(headers or {})
        return HttpClient().request(url + path, payload, auth, timeout=2)

    yield SimpleNamespace(shared=shared, registry=registry, agent=agent, start=start, request=request,
                          calls=calls, gets=gets, local_state=local_state, url=url)
    agent.stop.set()
    if agent_thread:
        agent_thread.join(timeout=2)
        assert not agent_thread.is_alive()
    for server, thread in zip((local, central), threads):
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_preview_summary_end_to_end(system):
    system.start()
    status, _, body = system.request('/api/robots')
    assert status == 200
    robot = loads(body)[0]
    assert set(robot) == {'robot_id', 'online', 'age', 'local_api_available', 'summary'}
    assert robot['online'] and robot['local_api_available']
    summary = robot['summary']
    assert summary['floor'] == '4F' and summary['robot']['x'] == .5
    assert summary['task'] == system.shared.snapshot()['task']
    assert summary['hold']['held'] is True
    assert not {'layers', 'camera', 'scan', 'telemetry'} & summary.keys()
    assert len(dumps(summary)) <= 16*1024
    assert set(system.gets) == {'/api/kiosk/state'}
    status, headers, body = system.request('/')
    assert status == 200 and 'AMR 중앙 관제'.encode() in body
    assert headers['Cache-Control'] == 'no-store'
    assert system.request('/console.js')[0] == system.request('/console.css')[0] == 200


@pytest.mark.parametrize('kind', ['stop', 'resume', 'return_home'])
def test_operator_commands_reach_stub_and_return_same_result(system, kind):
    system.start()
    request = {'request_id': f'request_{kind}'}
    status, _, body = system.request('/robots/amr-01/api/' + kind, request)
    assert status == 202 and loads(body) == dict(request, held=True)
    assert system.calls == [('/api/' + kind, request, '1', 'application/json')]
    # Same id at the default_robot route returns the cached result without executing again.
    assert system.request('/api/' + kind, request)[0] == 202
    assert len(system.calls) == 1


def test_local_404_passes_through(system):
    system.start()
    system.local_state['response_status'] = 404
    status, _, body = system.request('/api/return_home', {'request_id': 'home_missing'})
    assert status == 404 and loads(body)['request_id'] == 'home_missing'


@pytest.mark.parametrize('path', ['/api/state', '/api/floors', '/api/camera.jpg', '/api/layers/map.png',
                                  '/fleet', '/fleet.js', '/index.html', '/app.js', '/robots/amr-01/'])
def test_removed_routes(system, path):
    assert system.request(path)[0] == 404


def test_basic_bearer_and_request_guards(system):
    assert system.request('/')[0] == 200
    assert HttpClient().request(system.url + '/')[0] == 401
    assert system.request('/', headers={'Authorization': 'Bearer ' + TOKEN})[0] == 401
    assert system.request('/', headers={'Authorization': 'Basic !!!'})[0] == 401
    assert system.request('/api/agent/exchange', message())[0] == 401
    bearer = {'Authorization': 'Bearer ' + TOKEN}
    assert system.request('/api/agent/exchange', message(), bearer)[0] == 200
    assert system.request('/api/agent/exchange', message(2), {'Authorization': 'Bearer bad'})[0] == 401
    payload = {'request_id': 'request_001'}
    assert system.request('/api/stop', payload, {'X-AMR-Request': ''})[0] == 403
    assert system.request('/api/stop', payload, {'Origin': 'http://other.invalid'})[0] == 403
    assert system.request('/api/goal', payload)[0] == 404


def test_server_protocol_one_rejected_with_409(system):
    status, _, body = system.request('/api/agent/exchange', message(protocol=1),
                                     {'Authorization': 'Bearer ' + TOKEN})
    assert status == 409 and 'protocol 2' in loads(body)['error']


@pytest.mark.parametrize('status,reply', [(200, {'protocol': 1}),
                                          (409, {'error': 'Unsupported agent protocol'}),
                                          (400, {'error': 'Unsupported agent protocol'})])
def test_agent_protocol_one_is_runtime_error(status, reply):
    agent = Agent('http://127.0.0.1:1', TOKEN)
    agent.collect = lambda: (summarize({}), time.monotonic())
    agent.http.request = lambda *_args, **_kwargs: (status, {}, dumps(reply))
    with pytest.raises(RuntimeError, match='protocol 2'):
        agent.exchange()


def test_local_unavailable_and_stale_hide_summary_and_block_commands(system):
    system.start()
    system.local_state['available'] = False
    wait_for(lambda: system.registry.roster()[0]['local_api_available'] is False)
    robot = loads(system.request('/api/robots')[2])[0]
    assert robot['summary'] is None and not robot['online']
    assert system.request('/api/stop', {'request_id': 'request_001'})[0] == 503
    system.local_state['available'] = True
    wait_for(lambda: system.registry.roster()[0]['online'])
    system.agent.stop.set()
    wait_for(lambda: not system.registry.roster()[0]['online'])
    assert system.registry.roster()[0]['summary'] is None
    assert system.request('/api/stop', {'request_id': 'request_002'})[0] == 503


def test_sessions_sequence_and_takeover():
    registry = Registry()
    registry.exchange(message())
    for msg in (message(), message(2, session_id='b'*32)):
        with pytest.raises(ApiError) as error:
            registry.exchange(msg)
        assert error.value.status == 409
    command = registry.queue('amr-01', 'stop', {'request_id': 'request_001'})
    registry.robots['amr-01'].last_exchange -= 4
    registry.exchange(message(1, session_id='b'*32))
    assert command.result['http_status'] == 504
    assert registry.robots['amr-01'].session_id == 'b'*32


def test_queue_conflict_expiry_and_no_replay():
    registry = Registry()
    registry.exchange(message())
    first = registry.queue('amr-01', 'stop', {'request_id': 'request_001'})
    assert registry.queue('amr-01', 'stop', {'request_id': 'request_001'}) is first
    for kind, rid in [('resume', 'request_001'), ('stop', 'request_002')]:
        with pytest.raises(ApiError) as error:
            registry.queue('amr-01', kind, {'request_id': rid})
        assert error.value.status == 409
    first.deadline = time.monotonic() - .01
    assert registry.wait_result('amr-01', first)['http_status'] == 408
    second = registry.queue('amr-01', 'stop', {'request_id': 'request_002'})
    reply = registry.exchange(message(2))
    assert set(reply) == {'protocol', 'session_id', 'sequence', 'commands', 'accepted_results'}
    assert len(reply['commands']) == 1
    assert set(reply['commands'][0]) == {'command_id', 'kind', 'payload', 'ttl'}
    assert reply['commands'][0]['payload'] == {'request_id': 'request_002'}
    assert registry.exchange(message(3))['commands'] == []
    second.deadline = time.monotonic() - .01
    assert registry.wait_result('amr-01', second)['http_status'] == 504
    late = dict(command_id='request_002', http_status=202, body={'held': True})
    assert registry.exchange(message(4, results=[late]))['accepted_results'] == ['request_002']
    assert second.result['http_status'] == 504


def test_operator_wait_returns_504_when_sent_without_result(system):
    system.agent.exchange()
    output = []
    thread = threading.Thread(target=lambda: output.append(system.request('/api/stop', {'request_id': 'request_001'})))
    thread.start()
    wait_for(lambda: bool(system.registry.robots['amr-01'].commands))
    reply = system.registry.exchange(message(2, session_id=system.agent.session_id))
    assert len(reply['commands']) == 1
    thread.join(timeout=2)
    assert output[0][0] == 504 and system.calls == []


@pytest.mark.parametrize('payload', [{}, {'request_id': 'short'}, {'request_id': True},
                                     {'request_id': 'request_001', 'floor': '4F'},
                                     {'request_id': 'request_001', 'x': 0}])
def test_command_payload_is_request_id_only(payload):
    with pytest.raises(ApiError) as error:
        Registry().queue('amr-01', 'stop', payload)
    assert error.value.status == 400


@pytest.mark.parametrize('summary', [[], {'robot': {'x': True}}, {'robot': {'age': -1}},
                                     {'hold': {'held': 1}}, {'goal': {'room': []}},
                                     {'battery': {'soc': '80'}}, {'raw': {}},
                                     {'localization': {'ready': 'yes'}}, {'task': 'x'*17000}])
def test_invalid_summary_rejected(summary):
    with pytest.raises(ApiError) as error:
        Registry().exchange(message(summary=summary))
    assert error.value.status in (400, 413)


@pytest.mark.parametrize('update', [{'state_age': True}, {'state_age': -1}, {'sequence': True},
                                    {'session_id': 'bad'}, {'local_error': []}, {'results': {}},
                                    {'layers': {}}, {'results': [{'command_id': 'request_001',
                                      'http_status': True, 'body': {}}]}])
def test_invalid_envelope_rejected(update):
    with pytest.raises((ApiError, ValueError)):
        Registry().exchange(message(**update))


def test_agent_ttl_dedup_and_unknown_result_ack():
    agent = Agent('http://127.0.0.1:1', TOKEN)
    calls = []
    def request(*args, **kwargs):
        calls.append(args)
        return 202, {}, dumps({'held': True})
    agent.http.request = request
    command = dict(command_id='request_001', kind='stop', payload={'request_id': 'request_001'}, ttl=.01)
    agent.execute(command, .1)
    assert agent.results['request_001']['http_status'] == 408 and calls == []
    command.update(command_id='request_002', payload={'request_id': 'request_002'}, ttl=2)
    agent.execute(command, .01)
    agent.execute(command, .01)
    assert len(calls) == 1
    assert agent.results['request_002']['http_status'] == 202
    registry = Registry()
    reply = registry.exchange(message(results=list(agent.results.values())))
    assert set(reply['accepted_results']) == {'request_001', 'request_002'}


def test_agent_ambiguous_local_failure_is_never_retried():
    agent = Agent('http://127.0.0.1:1', TOKEN)
    calls = []
    def fail(*args, **kwargs):
        calls.append(args)
        raise TimeoutError()
    agent.http.request = fail
    command = dict(command_id='request_001', kind='stop', payload={'request_id': 'request_001'}, ttl=2)
    agent.execute(command, 0)
    agent.execute(command, 0)
    assert len(calls) == 1 and agent.results['request_001']['http_status'] == 504


def test_registry_arrival_transitions_and_bounded_dedupe():
    calls = []
    notifier = SimpleNamespace(arrival=lambda *args: calls.append(args))
    registry = Registry(notifier=notifier)
    sequence = 0
    def goal(rid, status, origin=None, room='401'):
        nonlocal sequence
        sequence += 1
        registry.exchange(message(sequence, summary={'goal': dict(request_id=rid, status=status,
                                                                 room=room, origin=origin)}))
    goal('delivery_01', 'ACCEPTED')
    goal('delivery_01', 'SUCCEEDED')
    goal('delivery_01', 'SUCCEEDED')
    assert calls == [('amr-01', '401', 'delivery_01')]
    goal('late_join_01', 'SUCCEEDED')
    goal('late_join_01', 'ACCEPTED')
    goal('late_join_01', 'SUCCEEDED')
    goal('return_home_01', 'ACCEPTED', 'return_home')
    goal('return_home_01', 'SUCCEEDED', 'return_home')
    goal('no_room_01', 'ACCEPTED', room='')
    goal('no_room_01', 'SUCCEEDED', room='')
    assert len(calls) == 1
    for n in range(300):
        goal(f'other_goal_{n}', 'SUCCEEDED')
    assert len(registry.arrivals) == 256 and len(calls) == 1


def test_stale_capture_and_capacity():
    registry = Registry(max_robots=1)
    registry.exchange(message(state_age=3))
    assert registry.roster()[0]['summary'] is None
    with pytest.raises(ApiError) as error:
        registry.exchange(message(robot_id='other'))
    assert error.value.status == 429


@pytest.mark.parametrize('url', ['http://example.com', 'http://127.0.0.1/path', 'http://user:pass@localhost',
                                'file:///tmp/test', 'http://localhost?secret=x'])
def test_local_api_requires_loopback_base_url(url):
    with pytest.raises(ValueError):
        base_url(url, local=True)


def test_preview_projection_validates_without_http():
    from amr_web_ui.server import SharedState
    preview = SharedState(preview=True)
    summary = summarize(preview.snapshot())
    registry = Registry()
    registry.exchange(message(summary=summary))
    assert registry.roster()[0]['summary'] == summary


def test_agent_summary_envelope_and_result_ack_without_transport():
    from amr_web_ui.server import SharedState
    preview = SharedState(preview=True)
    registry = Registry()
    agent = Agent('http://127.0.0.1:1', TOKEN)
    paths = []
    envelopes = []
    def request(url, payload=None, headers=None, **kwargs):
        paths.append(url)
        if url.endswith('/api/kiosk/state'):
            return 200, {}, dumps(preview.snapshot())
        envelope = loads(payload)
        envelopes.append(envelope)
        return 200, {}, dumps(registry.exchange(envelope))
    agent.http.request = request
    agent.results['request_old'] = dict(command_id='request_old', http_status=202, body={})
    assert agent.exchange()
    assert not agent.results
    assert len(paths) == 2 and paths[0].endswith('/api/kiosk/state')
    assert set(envelopes[0]) == {'protocol', 'robot_id', 'session_id', 'sequence', 'state_age',
                                'summary', 'local_error', 'results'}
    assert envelopes[0]['summary'] == summarize(preview.snapshot())


@pytest.mark.parametrize('method,path,payload,headers,expected', [
    ('GET', '/', None, {}, 200),
    ('GET', '/console.js', None, {}, 200),
    ('GET', '/console.css', None, {}, 200),
    ('GET', '/api/robots', None, {}, 200),
    ('GET', '/api/state', None, {}, 404),
    ('GET', '/fleet', None, {}, 404),
    ('GET', '/', None, {'Authorization': ''}, 401),
    ('GET', '/', None, {'Authorization': 'Bearer ' + TOKEN}, 401),
    ('GET', '/', None, {'Authorization': 'Basic !!!'}, 401),
    ('POST', '/api/agent/exchange', message(), {}, 401),
    ('POST', '/api/agent/exchange', message(), {'Authorization': 'Bearer ' + TOKEN}, 200),
    ('POST', '/api/agent/exchange', message(protocol=1), {'Authorization': 'Bearer ' + TOKEN}, 409),
    ('POST', '/api/stop', {'request_id': 'request_001'}, {}, 503),
    ('POST', '/api/stop', {'request_id': 'request_001'}, {'X-AMR-Request': ''}, 403),
    ('POST', '/api/stop', {'request_id': 'request_001'}, {'Origin': 'http://evil.invalid'}, 403),
    ('POST', '/api/stop', {'request_id': 'request_001'}, {'Content-Type': 'text/plain'}, 415),
    ('POST', '/api/stop', {'request_id': 'request_001'}, {'Content-Length': '999999'}, 413),
    ('POST', '/api/goal', {'request_id': 'request_001'}, {}, 404),
])
def test_http_handler_bytes_without_socket(monkeypatch, method, path, payload, headers, expected):
    """Supplement real TCP tests with raw HTTP parsing/auth/route coverage using byte streams."""
    import io
    import amr_remote_bridge.server as module
    monkeypatch.setattr(module, 'ThreadingHTTPServer',
                        lambda address, handler: SimpleNamespace(handler=handler))
    server = module.create_server(Registry(), TOKEN)
    body = dumps(payload) if payload is not None else b''
    request_headers = {'Host': '127.0.0.1', 'Authorization': BASIC, 'Connection': 'close',
                       'Content-Type': 'application/json', 'X-AMR-Request': '1',
                       'Content-Length': str(len(body))} | headers
    raw = (f'{method} {path} HTTP/1.1\r\n' + ''.join(f'{key}: {value}\r\n'
           for key, value in request_headers.items()) + '\r\n').encode() + body
    output = bytearray()
    stream = SimpleNamespace(makefile=lambda *_args: io.BytesIO(raw),
                             settimeout=lambda _timeout: None, sendall=output.extend)
    server.handler(stream, ('127.0.0.1', 12345), server)
    assert int(bytes(output).split(b' ', 2)[1]) == expected
    if path == '/api/robots' and expected == 200:
        assert loads(bytes(output).split(b'\r\n\r\n', 1)[1]) == []
