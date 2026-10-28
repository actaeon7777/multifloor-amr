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
def system(tmp_path):
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
    agent = Agent(url, TOKEN, local_url=f'http://127.0.0.1:{local.server_port}', rate=30, timeout=.3,
                  spool_dir=tmp_path / 'queue')
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


@pytest.mark.parametrize('kind,path,extra', [('patrol_start', 'patrol/start', {'route': '4F_full'}),
                                          ('patrol_stop', 'patrol/stop', {})])
def test_patrol_commands_reach_local_paths(system, kind, path, extra):
    system.start()
    payload = dict(request_id='request_' + kind, **extra)
    status, _, body = system.request('/robots/amr-01/api/' + kind, payload)
    assert status == 202 and loads(body)['request_id'] == payload['request_id']
    assert system.calls == [('/api/' + path, payload, '1', 'application/json')]
    assert system.request('/robots/amr-01/api/' + kind, payload)[0] == 202
    assert len(system.calls) == 1


@pytest.mark.parametrize('route', ['', '../4F', 'a'*33, None, True, [], '4F full'])
def test_patrol_bad_route_http_never_queued(system, route):
    system.start()
    payload = dict(request_id='bad_route_request', route=route)
    assert system.request('/robots/amr-01/api/patrol_start', payload)[0] == 400
    assert not system.registry.robots['amr-01'].commands and not system.calls


@pytest.mark.parametrize('kind,payload', [
    ('patrol_start', {'request_id': 'missing_route'}),
    ('patrol_start', {'request_id': 'bad_route_01', 'route': '../4F'}),
    ('patrol_start', {'request_id': 'extra_field', 'route': '4F_full', 'x': 1}),
    ('patrol_stop', {'request_id': 'extra_route', 'route': '4F_full'}),
    ('stop', {'request_id': 'old_kind_extra', 'route': '4F_full'}),
])
def test_patrol_payload_strict_on_both_sides(kind, payload):
    with pytest.raises(ApiError) as error:
        Registry().queue('amr-01', kind, payload)
    assert error.value.status == 400
    agent = Agent('http://127.0.0.1:8081', TOKEN)
    with pytest.raises(ValueError, match='Invalid command'):
        agent.execute(dict(command_id=payload['request_id'], kind=kind, payload=payload, ttl=2), 0)
    assert not agent.executed


@pytest.mark.parametrize('kind,path,extra', [('patrol_start', 'patrol/start', {'route': '4F_full'}),
                                          ('patrol_stop', 'patrol/stop', {})])
def test_patrol_relay_without_sockets(kind, path, extra):
    registry = Registry()
    registry.exchange(message())
    payload = dict(request_id='no_socket_patrol', **extra)
    registry.queue('amr-01', kind, payload)
    command = registry.exchange(message(2))['commands'][0]
    agent = Agent('http://127.0.0.1:8081', TOKEN)
    calls = []

    def request(url, body, headers, **_kwargs):
        calls.append((url, body, headers))
        return 202, {}, dumps({'request_id': body['request_id']})

    agent.http.request = request
    agent.execute(command, 0)
    assert calls == [(agent.local_url + '/api/' + path, payload, {'X-AMR-Request': '1'})]
    assert agent.results[payload['request_id']]['http_status'] == 202


@pytest.mark.parametrize('routes', ['4F_full', [None], [False], [1], ['a'*33], ['4F_full']*33])
def test_patrol_summary_routes_schema_rejects_invalid_lists(routes):
    with pytest.raises(ApiError) as error:
        Registry().exchange(message(summary={'patrol': {'routes': routes}}))
    assert error.value.status == 400


from amr_remote_bridge.common import MAX_EVENT_IMAGE, MAX_EVENT_BATCH
from amr_remote_bridge.events import EventStore
from amr_remote_bridge.spool_reader import read_spool
from test_events import event, JPEG


def write_spool(tmp_path, n=1, image=JPEG):
    directory = tmp_path / 'queue'
    directory.mkdir(exist_ok=True)
    snapshot = tmp_path / f'{n}.jpg'
    snapshot.write_bytes(image)
    e = event(n, image_path=str(snapshot))
    path = directory / f'{1700000000000+n:013d}-{e["id"]}.json'
    path.write_bytes(dumps(e))
    return path, e


def test_spool_reader_order_quarantine_tmp_and_bound(tmp_path, caplog):
    second, e2 = write_spool(tmp_path, 2)
    first, e1 = write_spool(tmp_path, 1)
    directory = first.parent
    (directory / '0000000000000-bad.json').write_text('{invalid')
    (directory / 'pending.json.tmp').write_text('{incomplete')
    assert read_spool(directory) == [(first, e1), (second, e2)]
    assert (directory / 'bad/0000000000000-bad.json').exists()
    assert (directory / 'pending.json.tmp').exists()
    assert len(caplog.records) == 1
    read_spool(directory)
    assert len(caplog.records) == 1
    for n in range(3, 25):
        write_spool(tmp_path, n)
    assert len(read_spool(directory, 100)) == 20
    assert read_spool(tmp_path / 'missing') == []


def test_agent_partial_upload_one_image_per_cycle_and_restart(tmp_path):
    first, e1 = write_spool(tmp_path, 1)
    second, e2 = write_spool(tmp_path, 2)
    store = EventStore(tmp_path / 'server')
    calls = []
    def request(url, payload=None, headers=None, **kwargs):
        calls.append(url)
        assert headers['Authorization'] == 'Bearer ' + TOKEN
        assert first.exists() or not url.endswith('/images/' + e1['id'])
        if url.endswith('/events'):
            assert payload['protocol'] == 2 and len(payload['session_id']) == 32
            return 200, {}, dumps(store.insert(payload['robot_id'], payload['events']))
        assert headers['Content-Type'] == 'image/jpeg' and payload == JPEG
        store.store_image(url.rsplit('/', 1)[1], payload)
        return 204, {}, b''
    agent = Agent('http://127.0.0.1:1', TOKEN, spool_dir=first.parent)
    agent.http.request = request
    agent.upload_events()
    assert not first.exists() and second.exists() and len(calls) == 2
    assert Path(e1['image_path']).exists()
    # All state needed to resume is in the spool and the server, even after restart.
    agent = Agent('http://127.0.0.1:1', TOKEN, spool_dir=first.parent)
    agent.http.request = request
    agent.upload_events()
    assert not second.exists() and len(calls) == 4
    assert store.image(e2['id']) == JPEG
    store.close()


@pytest.mark.parametrize('failure', ['offline', 'event_reject', 'image_fail', 'image_reject', 'missing_ack'])
def test_agent_failure_keeps_spool(tmp_path, monkeypatch, failure):
    path, e = write_spool(tmp_path)
    agent = Agent('http://127.0.0.1:1', TOKEN, spool_dir=path.parent)
    def request(url, *args, **kwargs):
        if failure == 'offline' or (failure == 'image_fail' and '/images/' in url):
            raise ConnectionError('server down')
        if failure == 'event_reject' or (failure == 'image_reject' and '/images/' in url):
            return 503, {}, b'{}'
        return 200, {}, dumps(dict(accepted=[] if failure == 'missing_ack' else [e['id']],
                                   want_images=[] if failure == 'missing_ack' else [e['id']]))
    monkeypatch.setattr(agent.http, 'request', request)
    if failure == 'missing_ack':
        agent.upload_events()
    else:
        with pytest.raises(ConnectionError):
            agent.upload_events()
    assert path.exists() and Path(e['image_path']).exists()


def test_agent_lost_image_ack_deduplicates_retry(tmp_path):
    path, e = write_spool(tmp_path)
    store = EventStore(tmp_path / 'server')
    agent = Agent('http://127.0.0.1:1', TOKEN, spool_dir=path.parent)
    def request(url, payload, headers, **kwargs):
        if url.endswith('/events'):
            return 200, {}, dumps(store.insert(payload['robot_id'], payload['events']))
        store.store_image(e['id'], payload)
        raise TimeoutError('ack lost')
    agent.http.request = request
    with pytest.raises(TimeoutError):
        agent.upload_events()
    assert path.exists()
    agent.upload_events()
    assert not path.exists() and store.image(e['id']) == JPEG
    store.close()


def test_agent_oversize_image_and_no_image_delete_after_ack(tmp_path, caplog):
    path, e = write_spool(tmp_path, image=JPEG + b'x'*MAX_EVENT_IMAGE)
    agent = Agent('http://127.0.0.1:1', TOKEN, spool_dir=path.parent)
    calls = []
    def request(url, *args, **kwargs):
        calls.append(url)
        return 200, {}, dumps(dict(accepted=[e['id']], want_images=[e['id']]))
    agent.http.request = request
    agent.upload_events()
    assert not path.exists() and Path(e['image_path']).exists()
    assert len(calls) == 1 and '400 KiB' in caplog.text
    path, e = write_spool(tmp_path, 2)
    e['image_path'] = None
    path.write_bytes(dumps(e))
    agent.http.request = lambda *a, **k: (200, {}, dumps(dict(accepted=[e['id']], want_images=[])))
    agent.upload_events()
    assert not path.exists()


def test_agent_run_exchange_before_events_and_backoff(tmp_path, monkeypatch, caplog):
    agent = Agent('http://127.0.0.1:1', TOKEN, spool_dir=tmp_path, rate=4)
    now, exchanges, uploads, delays = [0.], [], [], []
    monkeypatch.setattr('amr_remote_bridge.agent.time.monotonic', lambda: now[0])
    agent.exchange = lambda: exchanges.append(now[0]) or True
    def upload():
        assert exchanges[-1] == now[0]
        uploads.append(now[0])
        raise ConnectionError('store unavailable')
    agent.upload_events = upload
    def wait(delay):
        delays.append(delay)
        now[0] += delay
        if len(delays) == 120:
            agent.stop.set()
    agent.stop.wait = wait
    agent.run()
    assert exchanges == [n * .25 for n in range(120)]
    assert delays == [.25] * 120
    assert uploads == [0., .5, 1.5, 3.5, 7.5, 15.5, 25.5]
    assert sum('Event upload unavailable' in r.message for r in caplog.records) == 1
    assert 'Host exchange unavailable' not in caplog.text


@pytest.fixture
def event_http(tmp_path, monkeypatch):
    """Raw HTTP handler, including authentication/body framing, without TCP permissions."""
    import io
    import amr_remote_bridge.server as module
    monkeypatch.setattr(module, 'ThreadingHTTPServer',
                        lambda address, handler: SimpleNamespace(handler=handler))
    store = EventStore(tmp_path / 'data')
    server = module.create_server(Registry(), TOKEN, event_store=store)
    def request(path, payload=None, auth='Bearer ' + TOKEN, mime='application/json', headers=None):
        body = payload if isinstance(payload, bytes) else dumps(payload) if payload is not None else b''
        method = 'POST' if payload is not None else 'GET'
        request_headers = {'Host': '127.0.0.1', 'Authorization': auth, 'Connection': 'close',
                           'Content-Type': mime, 'Content-Length': str(len(body))} | (headers or {})
        raw = (f'{method} {path} HTTP/1.1\r\n' + ''.join(f'{key}: {value}\r\n'
               for key, value in request_headers.items()) + '\r\n').encode() + body
        output = bytearray()
        stream = SimpleNamespace(makefile=lambda *_args: io.BytesIO(raw),
                                 settimeout=lambda _timeout: None, sendall=output.extend)
        server.handler(stream, ('127.0.0.1', 12345), server)
        head, body = bytes(output).split(b'\r\n\r\n', 1)
        status = int(head.split(b' ', 2)[1])
        return status, head, body
    yield request, store
    store.close()


def envelope(events=None, **kwargs):
    return dict(protocol=2, robot_id='amr-01', session_id=SESSION,
                events=[event()] if events is None else events) | kwargs


@pytest.mark.parametrize('changes', [
    {'robot_id': '../bad'}, {'session_id': 'invalid'}, {'events': []}, {'events': [event()]*21},
    {'events': [event(kind='security')]}, {'events': [event(text='x'*1025)]},
    {'events': [event(extra={'x': 'x'*8192})]}, {'events': [event(pose=[1, True, 3])]},
    {'unexpected': 1},
])
def test_event_endpoint_validation(event_http, changes):
    request, store = event_http
    assert request('/api/agent/events', envelope(**changes))[0] == 400
    assert store.query()['events'] == []


def test_events_http_auth_size_and_operator_query(event_http):
    request, store = event_http
    assert request('/api/agent/events', envelope(), auth='')[0] == 401
    assert request('/api/agent/events', envelope(), auth=BASIC)[0] == 401
    assert request('/api/agent/events', envelope(protocol=1))[0] == 409
    assert request('/api/agent/events', envelope([event(text='x'*65536)]))[0] == 413
    assert request('/api/agent/events', envelope(), headers={'Content-Length': str(MAX_EVENT_BATCH+1)})[0] == 413
    for _ in range(2):
        status, _, body = request('/api/agent/events', envelope())
        assert status == 200 and loads(body) == dict(accepted=[event()['id']], want_images=[event()['id']])
    for path in ('/api/events', '/api/events/latest_seq', '/api/events/' + event()['id'] + '/image.jpg'):
        assert request(path, auth='')[0] == 401
        assert request(path)[0] == 401
    status, _, body = request('/api/events?after=0&limit=200&robot=amr-01', auth=BASIC)
    assert status == 200 and len(loads(body)['events']) == 1
    cursor = loads(body)['latest_seq']
    assert loads(request('/api/events/latest_seq', auth=BASIC)[2]) == {'latest_seq': cursor}
    assert loads(request(f'/api/events?after={cursor}', auth=BASIC)[2])['events'] == []
    for query in ('after=-1', 'limit=201', 'limit=0', 'robot=..', 'after=no', 'limit=1&limit=2'):
        assert request('/api/events?' + query, auth=BASIC)[0] == 400


def test_events_http_images(event_http):
    request, store = event_http
    id = event()['id']
    upload = '/api/agent/images/' + id
    download = '/api/events/' + id + '/image.jpg'
    assert request(upload, JPEG, mime='image/jpeg', auth='')[0] == 401
    assert request(upload, JPEG, mime='image/jpeg', auth=BASIC)[0] == 401
    assert request(upload, JPEG, mime='image/jpeg')[0] == 404
    request('/api/agent/events', envelope())
    assert request(download, auth=BASIC)[0] == 404
    assert request(upload, JPEG)[0] == 415
    assert request(upload, b'not jpeg', mime='image/jpeg')[0] == 400
    assert request(upload, JPEG + b'x'*MAX_EVENT_IMAGE, mime='image/jpeg')[0] == 413
    assert request('/api/agent/images/../../evil', JPEG, mime='image/jpeg')[0] == 400
    assert request('/api/events/../../evil/image.jpg', auth=BASIC)[0] == 400
    for _ in range(2):
        assert request(upload, JPEG, mime='image/jpeg')[0] == 204
    status, head, body = request(download, auth=BASIC)
    assert status == 200 and body == JPEG
    assert b'Content-Type: image/jpeg' in head
    assert head.count(b'Cache-Control:') == 1 and b'private, max-age=3600' in head
    assert loads(request('/api/events', auth=BASIC)[2])['events'][0]['has_image']


def test_event_http_client_jpeg_transport(tmp_path):
    """Actual loopback HTTP complements the stream-based endpoint tests."""
    path, e = write_spool(tmp_path)
    store = EventStore(tmp_path / 'server')
    central = create_server(Registry(), TOKEN, port=0, event_store=store)
    thread = threading.Thread(target=central.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
    thread.start()
    try:
        agent = Agent(f'http://127.0.0.1:{central.server_port}', TOKEN, spool_dir=path.parent)
        agent.upload_events()
        assert not path.exists() and store.image(e['id']) == JPEG
    finally:
        central.shutdown()
        central.server_close()
        thread.join(timeout=2)
        store.close()


def test_http_client_preserves_jpeg_content_type_without_socket():
    import io
    requests = []
    client = HttpClient()
    def open_request(request, timeout):
        requests.append(request)
        response = io.BytesIO(b'')
        response.status = 204
        response.headers = {}
        return response
    client.opener = SimpleNamespace(open=open_request)
    assert client.request('http://127.0.0.1:1/api/agent/images/' + event()['id'], JPEG,
                          {'Content-Type': 'image/jpeg'})[0] == 204
    assert requests[0].get_header('Content-type') == 'image/jpeg'
    assert requests[0].data == JPEG
    client.request('http://127.0.0.1:1/api/agent/events', envelope())
    assert requests[1].get_header('Content-type') == 'application/json'


@pytest.mark.parametrize('stage', ['events', 'image'])
@pytest.mark.parametrize('status', [400, 401, 403, 404, 413, 415, 408, 429, 503])
def test_agent_server_rejections_quarantine_only_permanent(tmp_path, stage, status):
    path, e = write_spool(tmp_path)
    second, _ = write_spool(tmp_path, 2)
    agent = Agent('http://127.0.0.1:1', TOKEN, spool_dir=path.parent)
    def request(url, *args, **kwargs):
        if stage == 'image' and url.endswith('/events'):
            return 200, {}, dumps(dict(accepted=[e['id']], want_images=[e['id']]))
        return status, {}, b'{}'
    agent.http.request = request
    with pytest.raises(ConnectionError):
        agent.upload_events()
    permanent = 400 <= status < 500 and status not in (408, 429)
    assert path.exists() is not permanent
    assert (path.parent / 'bad' / path.name).exists() is permanent
    assert second.exists() is not (permanent and stage == 'events')
    assert Path(e['image_path']).exists()


@pytest.mark.parametrize('error', [FileNotFoundError, PermissionError, IsADirectoryError])
def test_unreadable_images_do_not_block_queue(tmp_path, monkeypatch, caplog, error):
    records = [write_spool(tmp_path, n) for n in range(1, 22)]
    image_paths = {Path(e['image_path']) for _, e in records[:20]}
    original_open = Path.open
    def open_file(path, *args, **kwargs):
        if path in image_paths:
            raise error('unreadable image')
        return original_open(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', open_file)
    agent = Agent('http://127.0.0.1:1', TOKEN, spool_dir=records[0][0].parent)
    image_uploads = []
    def request(url, payload, *args, **kwargs):
        if url.endswith('/events'):
            ids = [e['id'] for e in payload['events']]
            return 200, {}, dumps(dict(accepted=ids, want_images=ids))
        image_uploads.append(url)
        return 204, {}, b''
    agent.http.request = request
    agent.upload_events()
    assert all(not path.exists() for path, _ in records[:20])
    assert records[20][0].exists() and not image_uploads
    assert error.__name__ in caplog.text
    agent.upload_events()
    assert not records[20][0].exists() and len(image_uploads) == 1


@pytest.mark.parametrize('error', [FileNotFoundError, PermissionError, IsADirectoryError])
def test_spool_open_races_and_errors(tmp_path, monkeypatch, caplog, error):
    path, _ = write_spool(tmp_path)
    second, e2 = write_spool(tmp_path, 2)
    original_open = Path.open
    def open_file(candidate, *args, **kwargs):
        if candidate == path:
            if error is FileNotFoundError:
                candidate.unlink()
            raise error('read failure')
        return original_open(candidate, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', open_file)
    assert read_spool(path.parent) == [(second, e2)]
    if error is FileNotFoundError:
        assert not caplog.records
    else:
        assert (path.parent / 'bad' / path.name).exists()
        assert 'Quarantined spool file' in caplog.text


@pytest.mark.parametrize('error', [FileNotFoundError, PermissionError])
def test_spool_quarantine_races_and_errors(tmp_path, monkeypatch, caplog, error):
    path, _ = write_spool(tmp_path)
    path.write_text('{invalid')
    second, e2 = write_spool(tmp_path, 2)
    def replace(candidate, target):
        if error is FileNotFoundError:
            candidate.unlink()
        raise error('rename failure')
    monkeypatch.setattr(Path, 'replace', replace)
    assert read_spool(path.parent) == [(second, e2)]
    assert bool(caplog.records) is (error is PermissionError)
    if caplog.records:
        assert 'Could not quarantine' in caplog.text


def test_patrol_spool_without_extra_is_uploaded(tmp_path):
    path, e = write_spool(tmp_path)
    e.pop('extra')
    e.update(kind='patrol.started', image_path=None)
    path.write_bytes(dumps(e))
    pending = read_spool(path.parent)
    assert pending[0][1]['extra'] is None
    agent = Agent('http://127.0.0.1:1', TOKEN, spool_dir=path.parent)
    def request(url, payload, *args, **kwargs):
        assert payload['events'][0]['extra'] is None
        return 200, {}, dumps(dict(accepted=[e['id']], want_images=[]))
    agent.http.request = request
    agent.upload_events()
    assert not path.exists()


def test_events_http_kind_filter_and_optional_extra(event_http):
    request, store = event_http
    patrol = event(1, kind='patrol.started', image_path=None)
    patrol.pop('extra')
    assert request('/api/agent/events', envelope([patrol]))[0] == 200
    store.insert('amr-01', [event(2), event(3, kind='security_other.motion')])
    response = request('/api/events?kind_prefix=security.', auth=BASIC)
    assert response[0] == 200
    assert [e['id'] for e in loads(response[2])['events']] == [event(2)['id']]
    for prefix in ('security', '%25', 'security.%25', 'Security.', ''):
        assert request('/api/events?kind_prefix=' + prefix, auth=BASIC)[0] == 400


def test_server_default_data_dir_is_independent_of_cwd(tmp_path, monkeypatch):
    import amr_remote_bridge.server as module
    monkeypatch.chdir(tmp_path)
    monkeypatch.delenv('AMR_DATA_DIR', raising=False)
    monkeypatch.setattr(module, 'read_token', lambda path: TOKEN)
    monkeypatch.setattr(module, 'SlackNotifier', lambda **kwargs: SimpleNamespace(close=lambda: None))
    directories = []
    def make_store(directory, *args):
        directories.append(Path(directory))
        return SimpleNamespace(close=lambda: None)
    monkeypatch.setattr(module, 'EventStore', make_store)
    monkeypatch.setattr(module, 'create_server', lambda *args, **kwargs:
                        SimpleNamespace(server_port=8081, serve_forever=lambda **kw: None,
                                        server_close=lambda: None))
    module.main(['--token-file', 'unused'])
    expected = Path(__file__).resolve().parents[4] / 'Control_Server/data'
    assert directories == [expected] and directories[0].is_absolute()


def test_console_event_order_filter_and_unchanged_dom():
    """Exercise real console JS against a tiny DOM and server response model."""
    import shutil
    import subprocess
    node = shutil.which('node')
    if not node:
        pytest.skip('Node is required for the console regression test')
    source = Path(__file__).resolve().parents[1] / 'web/console.js'
    script = r'''
const vm = require('vm'), fs = require('fs'), assert = require('assert');
class Node {
  constructor() { this.children = []; this.listeners = {}; this.attributes = {}; this.renders = 0; }
  append(...children) { this.children.push(...children); }
  replaceChildren(...children) { this.children = children; this.renders++; }
  addEventListener(name, fn) { this.listeners[name] = fn; }
  setAttribute(name, value) { this.attributes[name] = value; }
}
const nodes = new Map(), requests = [];
const makeEvent = (n, kind) => ({id: n.toString(16).padStart(32, '0'), seq: n,
  wall_time: n, kind, text: 'event ' + n, floor: '4F', robot_id: 'amr-01', severity: 'info', has_image: false});
const events = [makeEvent(1, 'security.motion'), makeEvent(2, 'security.motion')];
for (let n = 3; n <= 62; n++) events.push(makeEvent(n, 'patrol.started'));
let latest = 62;
const context = vm.createContext({
  document: {title: 'AMR', createElement: () => new Node(), addEventListener() {},
    querySelector: id => { if (!nodes.has(id)) nodes.set(id, new Node()); return nodes.get(id); }},
  window: {}, localStorage: {getItem() { return null; }},
  AbortSignal: {timeout() {}}, setInterval() {}, clearInterval() {},
  fetch: async path => {
    requests.push(path);
    const query = new URL(path, 'http://localhost').searchParams;
    return {ok: true, json: async () => path === '/api/robots' ? [] :
      path === '/api/events/latest_seq' ? {latest_seq: latest} : {latest_seq: latest,
        events: events.filter(e => e.seq > Number(query.get('after')) &&
          (!query.get('kind_prefix') || e.kind.startsWith(query.get('kind_prefix'))))
          .sort((a,b) => b.wall_time - a.wall_time).slice(0, Number(query.get('limit')))
          .map(e => ({...e}))}};
  }
});
vm.runInContext(fs.readFileSync(process.argv[1], 'utf8'), context);
const settle = () => new Promise(setImmediate);
(async () => {
  await settle();
  const list = nodes.get('#events');
  assert.equal(list.children.length, 50);
  const renders = list.renders, count = requests.length;
  await vm.runInContext('pollEvents()', context);
  assert.equal(list.renders, renders);
  assert.equal(requests.length, count + 1); // unchanged: only latest_seq
  nodes.get('#events-security').listeners.click();
  await settle();
  assert(requests.some(path => path.includes('after=0') && path.includes('kind_prefix=security.')));
  assert.equal(list.children.length, 2); // older than all 50 patrol events
  const text = row => row.children[0].children[1].textContent;
  assert.deepEqual(list.children.map(text), ['event 2', 'event 1']);
  events[0].has_image = true; events[0].seq = ++latest;
  await vm.runInContext('pollEvents()', context);
  assert.deepEqual(list.children.map(text), ['event 2', 'event 1']);
  assert.equal(list.children[1].children[2].children[0].loading, 'lazy');
  const imageRow = list.children[1], afterImage = list.renders;
  events[0].seq = ++latest; // changed cursor, identical visible rows
  await vm.runInContext('pollEvents()', context);
  assert.equal(list.renders, afterImage);
  assert.strictEqual(list.children[1], imageRow);
  events.push(makeEvent(++latest, 'patrol.started'));
  await vm.runInContext('pollEvents()', context);
  assert.equal(list.renders, afterImage); // unrelated kind: no repaint
  nodes.get('#events-all').listeners.click();
  await settle();
  assert.equal(list.children.length, 50);
  assert.equal(text(list.children[0]), 'event ' + latest);
  console.log('console regression passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
'''
    result = subprocess.run([node, '-e', script, str(source)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
