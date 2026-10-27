"""Kiosk contract over loopback HTTP; no ROS or vehicle IO."""
from contextlib import contextmanager
import json
import re
import threading
import time
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from amr_web_ui.geometry import Grid
from amr_web_ui.server import FLOOR_MISSION_ACTIVE, SharedState, create_http_server


@pytest.fixture
def shared():
    state = SharedState(preview=True)
    floors = {}
    for name, room in [('3F', '302'), ('4F', '402')]:
        floors[name] = SimpleNamespace(rooms={room: SimpleNamespace(goal=(1., 1., .5))}, real_enabled=True)
        state.floor_grids[name] = Grid(4, 4, 1., [0., 0., 0.], [0]*16)
        state.floor_grids[name].version = int(name[0]) * 2**40
    state.building = SimpleNamespace(floors=floors, floor=lambda name: floors[name])
    state.grids['map'] = Grid(4, 4, 1., [0., 0., 0.], [0]*16)
    state.grids['map'].version = 17
    state.state['floor'] = dict(floor='4F', zone=1, confidence='BEACON')
    return state


@pytest.fixture
def http(tmp_path):
    @contextmanager
    def serve(shared):
        server = create_http_server(shared, tmp_path, port=0)
        thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
        thread.start()

        def request(path, payload=None, kiosk=True, headers=None):
            defaults = {'Content-Type': 'application/json', 'X-AMR-Request': '1'}
            if kiosk:
                defaults['X-AMR-Client'] = 'kiosk'
            defaults.update(headers or {})
            req = Request(f'http://127.0.0.1:{server.server_port}'+path,
                          data=None if payload is None else json.dumps(payload).encode(), headers=defaults)
            try:
                response = urlopen(req, timeout=2)
            except HTTPError as error:
                response = error
            with response:
                body = response.read()
                if response.headers.get_content_type() == 'application/json':
                    body = json.loads(body)
                return response.status, body, response.headers

        try:
            yield request
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
    return serve


def pose(shared, **overrides):
    result = dict(request_id='kiosk_request_01', x=1., y=1., yaw=.5,
                  map_version=shared.grids['map'].version)
    result.update(overrides)
    return result


def test_static_kiosk_routes_and_missing_assets(shared, http, tmp_path):
    with http(shared) as request:
        for route, filename, mime in [('/kiosk', 'kiosk.html', 'text/html'),
                                      ('/kiosk.js', 'kiosk.js', 'text/javascript'),
                                      ('/kiosk.css', 'kiosk.css', 'text/css')]:
            assert request(route)[0] == 404
            (tmp_path/filename).write_text('fixture asset')
            status, body, headers = request(route)
            assert status == 200 and body == b'fixture asset'
            assert headers.get_content_type() == mime
            assert headers['Cache-Control'] == 'no-store'
            assert headers['X-Content-Type-Options'] == 'nosniff'
            assert "frame-ancestors 'none'" in headers['Content-Security-Policy']


def test_light_snapshot_uses_same_freshness_and_skips_heavy_parts(shared, http, monkeypatch):
    monkeypatch.setattr(time, 'monotonic', lambda: 100.)
    monkeypatch.setattr(time, 'time', lambda: 1000.)
    shared.kiosk_pin = '1234'
    shared.state.update(robot=dict(x=1., y=1., yaw=0., received_at=99.),
                        global_path=dict(points=[[1., 1.]], received_at=99.),
                        scan=dict(points=[[2., 2.]], received_at=99.),
                        localization=dict(ready=True, stamp=997., received_at=99.),
                        goal=dict(request_id='goal_001', status='ACCEPTED', feedback_received_at=98.))
    expected = set('preview connection_epoch mode navigation action_ready floor_action_ready floor '
                   'floor_mission goal notice pending_request localization localization_reset_pending '
                   'robot global_path tf_error estop telemetry acknowledged_request kiosk map_version'.split())
    with http(shared) as request:
        full = request('/api/state')[1]
        assert full['kiosk'] == {'pin_required': True} and full['acknowledged_request'] is None
        assert not full['localization']['ready']

        def unexpected(*_args):
            pytest.fail('light snapshot built a heavy component')

        for method in ('elevator_snapshot', 'transit_snapshot', 'camera_snapshot'):
            monkeypatch.setattr(shared, method, unexpected)
        monkeypatch.setattr(shared.grids['map'], 'metadata', unexpected)
        status, light, _ = request('/api/kiosk/state')
        assert status == 200 and set(light) == expected and 'scan' not in light
        assert light == {key: full[key] for key in expected}
        assert 'received_at' in shared.state['robot']


@pytest.mark.parametrize('floor,room,kind', [('4F', '402', 'goal'), ('3F', '302', 'floor_goal')])
def test_registered_room_goal_uses_normal_request_path(shared, http, floor, room, kind):
    version = shared.grids['map'].version if kind == 'goal' else shared.floor_grids[floor].version
    payload = pose(shared, floor=floor, room=room, map_version=version)
    with http(shared) as request:
        status, record, _ = request('/api/goal', payload)
        assert status == 202 and record['kind'] == kind and record['status'] == 'PREVIEW'
        assert shared.state['goal']['room'] == room and shared.state['goal']['floor'] == floor
        assert request('/api/goal', payload)[1] == record
        assert request('/api/goal', dict(payload, x=1.+.5e-6, request_id='tolerance_goal'))[0] == 202
        assert request('/api/goal', dict(payload, map_version=-1, request_id='stale_map_goal'))[0] == 409


def test_state_reports_live_map_version_and_null_when_missing(shared, http):
    live_map = shared.grids['map']
    assert live_map is not shared.floor_grids['4F']
    assert live_map.version != shared.floor_grids['4F'].version
    with http(shared) as request:
        for version in (17, 18, None):
            if version is None:
                shared.grids.pop('map')
            else:
                live_map.version = version
            for endpoint in ('/api/state', '/api/kiosk/state'):
                status, snapshot, _ = request(endpoint)
                assert status == 200 and snapshot['map_version'] == version


@pytest.mark.parametrize('kind', ['goal', 'initialpose'])
def test_same_floor_kiosk_commands_require_live_version_not_floor_hash(shared, http, kind):
    with http(shared) as request:
        version = request('/api/kiosk/state')[1]['map_version']
        payload = pose(shared, map_version=version)
        if kind == 'goal':
            payload.update(floor='4F', room='402')
        wrong_version = dict(payload, map_version=shared.floor_grids['4F'].version)
        status, body, _ = request('/api/'+kind, wrong_version)
        assert status == 409 and body['error'] == '지도가 갱신되었습니다. 목적지를 다시 선택해 주세요.'
        assert not shared.requests
        assert request('/api/'+kind, payload)[0] == 202


@pytest.mark.parametrize('preview', [True, False])
def test_active_floor_mission_blocks_only_kiosk_goals(shared, http, preview):
    shared.state['preview'] = preview
    with http(shared) as request:
        for phase in sorted(FLOOR_MISSION_ACTIVE):
            shared.state['floor_mission'] = dict(phase=phase)
            for floor, room in [('4F', '402'), ('3F', '302')]:
                version = shared.grids['map'].version if floor == '4F' else shared.floor_grids[floor].version
                payload = pose(shared, floor=floor, room=room, map_version=version)
                assert request('/api/goal', payload)[0:2] == (
                    409, {'error': '진행 중인 층간 이동이 있습니다.'})
        assert not shared.requests and shared.commands.empty()
        # Existing same-floor operator goal admission is unchanged.
        shared.state.update(action_ready=True, mode='AUTO', robot=dict(received_at=time.monotonic()),
                            localization=dict(ready=True, stamp=time.time(), received_at=time.monotonic()))
        assert request('/api/goal', pose(shared, floor='4F', room='402'), kiosk=False)[0] == 202


@pytest.mark.parametrize('changes', [dict(x=2.), dict(y=2.), dict(yaw=0.), dict(room=None),
                                    dict(floor=None), dict(room='999'), dict(floor='9F'),
                                    dict(x=True), dict(x='1'), dict(x=float('nan')), dict(x=10**400)])
def test_kiosk_arbitrary_or_missing_room_goal_forbidden(shared, http, changes):
    with http(shared) as request:
        status, body, _ = request('/api/goal', pose(shared, floor='4F', room='402') | changes)
        assert status == 403 and body['error'] == '키오스크에서는 등록된 호실로만 보낼 수 있습니다.'
        assert request('/api/goal', pose(shared))[0] == 403
        assert not shared.requests and shared.commands.empty()


@pytest.mark.parametrize('kind', ['initialpose', 'relocalize', 'set_floor'])
def test_kiosk_admin_commands_pin_and_expiry(shared, http, monkeypatch, kind):
    now = [100.]
    monkeypatch.setattr(time, 'monotonic', lambda: now[0])
    payload = pose(shared) if kind == 'initialpose' else dict(request_id='admin_request_01')
    if kind == 'relocalize':
        payload['map_version'] = shared.grids['map'].version
    elif kind == 'set_floor':
        payload['floor'] = '3F'
    with http(shared) as request:
        assert request('/api/'+kind, payload)[0] == 202  # No PIN configured.
        assert request('/api/kiosk/unlock', {})[0] == 200
        shared.kiosk_pin = '비밀1234'
        payload['request_id'] = 'admin_request_02'
        assert request('/api/'+kind, payload)[0:2] == (
            403, {'error': '관리 모드 잠금을 먼저 해제하세요.'})
        assert request('/api/kiosk/unlock', {'pin': 'wrong'})[0:2] == (403, {'error': 'PIN이 맞지 않습니다.'})
        assert request('/api/'+kind, payload, headers={'X-AMR-Kiosk-Token': 'invalid'})[0] == 403
        status, unlocked, _ = request('/api/kiosk/unlock', {'pin': '비밀1234'})
        assert status == 200 and unlocked['expires_in'] == 600
        assert re.fullmatch('[0-9a-f]{32}', unlocked['token'])
        headers = {'X-AMR-Kiosk-Token': unlocked['token']}
        assert request('/api/'+kind, payload, headers=headers)[0] == 202
        now[0] += 600
        assert request('/api/'+kind, payload, headers=headers)[0] == 403


def test_unlock_rate_window_and_bounded_tokens(shared, http, monkeypatch):
    now = [100.]
    monkeypatch.setattr(time, 'monotonic', lambda: now[0])
    shared.kiosk_pin = '1234'
    with http(shared) as request:
        assert request('/api/kiosk/unlock', {'pin': 'bad'})[0] == 403
        now[0] += 60
        for _ in range(4):
            assert request('/api/kiosk/unlock', {'pin': 'bad'})[0] == 403
        assert request('/api/kiosk/unlock', {'pin': 'bad'})[0] == 429
        assert request('/api/kiosk/unlock', {'pin': '1234'})[0] == 429
        now[0] += 29.9
        assert request('/api/kiosk/unlock', {'pin': '1234'})[0] == 429
        now[0] += .1
        tokens = [request('/api/kiosk/unlock', {'pin': '1234'})[1]['token'] for _ in range(9)]
        assert len(set(tokens)) == 9 and len(shared.kiosk_tokens) == 8
        assert request('/api/initialpose', pose(shared), headers={'X-AMR-Kiosk-Token': tokens[0]})[0] == 403
        assert request('/api/initialpose', pose(shared), headers={'X-AMR-Kiosk-Token': tokens[-1]})[0] == 202


@pytest.mark.parametrize('headers,status', [({'X-AMR-Request': ''}, 403),
                                          ({'Origin': 'http://other.example'}, 403),
                                          ({'Content-Type': 'text/plain'}, 415)])
def test_unlock_preserves_post_guards(shared, http, headers, status):
    with http(shared) as request:
        assert request('/api/kiosk/unlock', {'pin': ''}, headers=headers)[0] == status
        assert not shared.kiosk_tokens


def test_non_kiosk_requests_unchanged_and_unknown_kiosk_command_denied(shared, http):
    shared.kiosk_pin = '1234'
    with http(shared) as request:
        assert request('/api/goal', pose(shared, x=2.), kiosk=False)[0] == 202
        assert request('/api/initialpose', pose(shared, request_id='operator_pose_01'), kiosk=False)[0] == 202
        assert request('/api/relocalize', dict(request_id='operator_reset_01',
                       map_version=shared.grids['map'].version), kiosk=False)[0] == 202
        assert request('/api/cancel', {'request_id': 'kiosk_cancel_01'})[0] == 202
        assert request('/api/unknown', {})[0:2] == (403, {'error': '키오스크에서 허용하지 않는 명령입니다.'})
        assert request('/api/unknown', {}, kiosk=False)[0] == 404


@pytest.mark.parametrize('terminal', ['SUCCEEDED', 'FAILED', 'REJECTED', 'CANCELED'])
def test_acknowledge_only_current_terminal_goal_idempotently(shared, http, terminal):
    shared.kiosk_pin = '1234'
    shared.state['preview'] = False
    with http(shared) as request:
        assert request('/api/acknowledge', {'request_id': 'delivery_001'})[0] == 409
        shared.state['goal'] = dict(request_id='delivery_001', status='ACCEPTED')
        assert request('/api/acknowledge', {'request_id': 'delivery_001'})[0] == 409
        shared.state['goal']['status'] = terminal
        assert request('/api/acknowledge', {'request_id': 'old_delivery'})[0] == 409
        for kiosk in (True, False):
            assert request('/api/acknowledge', {'request_id': 'delivery_001'}, kiosk=kiosk)[0:2] == (
                202, {'acknowledged': 'delivery_001'})
        assert request('/api/state')[1]['acknowledged_request'] == 'delivery_001'
        assert request('/api/kiosk/state')[1]['acknowledged_request'] == 'delivery_001'
        shared.state['goal'] = dict(request_id='delivery_002', status=terminal)
        assert request('/api/acknowledge', {'request_id': 'delivery_001'})[0] == 409
        assert request('/api/acknowledge', {})[0] == 409
        assert not shared.requests and shared.commands.empty()


def test_set_floor_validation_preview_queue_completion_and_dedup(shared, http):
    payload = dict(request_id='set_floor_001', floor='3F')
    with http(shared) as request:
        for floor in (None, [], '', 3):
            assert request('/api/set_floor', dict(payload, floor=floor))[0] == 400
        assert request('/api/set_floor', dict(payload, floor='9F'))[0] == 404
        building, shared.building = shared.building, None
        assert request('/api/set_floor', payload)[0] == 404
        shared.building = building
        assert request('/api/set_floor', payload)[1]['status'] == 'PREVIEW'
        assert shared.commands.empty()
        shared.state.update(preview=False, velocity=dict(linear=0., angular=0., received_at=time.monotonic()))
        payload['request_id'] = 'set_floor_002'
        status, record, _ = request('/api/set_floor', payload)
        assert status == 202 and record['status'] == 'QUEUED'
        assert shared.state['pending_request'] == dict(request_id='set_floor_002', kind='set_floor')
        assert request('/api/set_floor', payload)[1] == record
        assert request('/api/set_floor', dict(payload, floor='4F'))[0] == 409
        command = shared.commands.get_nowait()
        assert command['kind'] == 'set_floor' and command['floor'] == '3F' and command['pose'] is None
        assert shared.commands.empty() and not shared.state['localization_reset_pending']
        shared.complete('set_floor_002', 'SUCCEEDED')
        assert shared.state['pending_request'] is None
        assert request('/api/set_floor', payload)[1]['status'] == 'SUCCEEDED'


@pytest.mark.parametrize('state', [dict(navigation='RUNNING'), dict(goal={'status': 'ACCEPTED'}),
                                  dict(velocity=None), dict(velocity=dict(linear=0., angular=0., received_at=-100.)),
                                  dict(velocity=dict(linear=.1, angular=0., received_at=100.)),
                                  dict(velocity=dict(linear=0., angular=.1, received_at=100.)),
                                  dict(floor_mission={'phase': 'RIDING'})])
def test_set_floor_readiness_blocks_unsafe_override(shared, http, monkeypatch, state):
    monkeypatch.setattr(time, 'monotonic', lambda: 100.)
    shared.state.update(preview=False, velocity=dict(linear=0., angular=0., received_at=100.))
    shared.state.update(state)
    with http(shared) as request:
        assert request('/api/set_floor', dict(request_id='set_floor_blocked', floor='3F'))[0] == 409
        assert shared.commands.empty()


@pytest.fixture
def executor_stub(shared):
    """Use bridge methods and generated messages, but never instantiate a ROS node."""
    pytest.importorskip('rclpy')
    from concurrent.futures import Future
    from builtin_interfaces.msg import Time
    from amr_web_ui.ros_bridge import WebBridge

    class Bridge:
        process_command = WebBridge.process_command
        poll_set_floor = WebBridge.poll_set_floor
        on_set_floor_response = WebBridge.on_set_floor_response

        def __init__(self):
            self.shared = shared
            self.waiting_set_floor = self.pending_set_floor = None
            self.ready = False
            self.calls = []
            self.future = Future()
            self.set_floor_client = SimpleNamespace(service_is_ready=lambda: self.ready, call_async=self.call)
            self.client = SimpleNamespace(server_is_ready=lambda: True, send_goal_async=self.send_goal)

        def call(self, request):
            self.calls.append(request.floor)
            return self.future

        def send_goal(self, goal, **_kwargs):
            self.calls.append(goal)
            return self.future

        def set_state(self, **values):
            self.shared.state.update(values)

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))

    return Bridge()


@pytest.mark.parametrize('success', [True, False])
def test_set_floor_async_discovery_and_response_without_ros_node(shared, executor_stub, monkeypatch, success):
    now = [100.]
    monkeypatch.setattr(time, 'monotonic', lambda: now[0])
    shared.state.update(preview=False, velocity=dict(linear=0., angular=0., received_at=100.))
    shared.request('set_floor', dict(request_id='async_floor_001', floor='3F'))
    bridge = executor_stub
    bridge.process_command(shared.commands.get_nowait())
    assert not bridge.calls and shared.state['pending_request']
    now[0] += .5
    bridge.ready = True
    bridge.poll_set_floor()
    assert bridge.calls == ['3F'] and shared.state['pending_request']
    bridge.future.set_result(SimpleNamespace(success=success, message='monitor response'))
    assert shared.requests['async_floor_001']['status'] == ('SUCCEEDED' if success else 'FAILED')
    assert shared.state['notice']['detail'] == 'monitor response'
    assert shared.state['pending_request'] is None


@pytest.mark.parametrize('change', ['unavailable', 'moving', 'mission', 'timeout'])
def test_set_floor_waiting_failures_without_ros_node(shared, executor_stub, monkeypatch, change):
    now = [100.]
    monkeypatch.setattr(time, 'monotonic', lambda: now[0])
    shared.state.update(preview=False, velocity=dict(linear=0., angular=0., received_at=100.))
    shared.request('set_floor', dict(request_id='waiting_floor_01', floor='3F'))
    bridge = executor_stub
    bridge.process_command(shared.commands.get_nowait())
    if change == 'moving':
        bridge.ready = True
        shared.state['velocity']['linear'] = .1
    elif change == 'mission':
        bridge.ready = True
        shared.state['floor_mission'] = dict(phase='RIDING')
    elif change == 'timeout':
        bridge.ready = True
        bridge.poll_set_floor()
        assert bridge.calls == ['3F']
        now[0] += 5.1
    else:
        now[0] += 1
    bridge.poll_set_floor()
    assert shared.requests['waiting_floor_01']['status'] == 'FAILED'
    assert shared.state['pending_request'] is None
    if change == 'unavailable':
        assert shared.state['notice']['detail'] == '층 관리 노드에 연결되지 않았습니다.'
    if change == 'timeout':
        assert bridge.future.cancelled()
    else:
        assert not bridge.calls


@pytest.mark.parametrize('with_room', [True, False])
def test_same_floor_goal_retains_room_without_ros_node(shared, executor_stub, with_room):
    shared.state.update(preview=False, action_ready=True, mode='AUTO',
                        robot=dict(received_at=time.monotonic()),
                        localization=dict(ready=True, stamp=time.time(), received_at=time.monotonic()))
    payload = pose(shared)
    if with_room:
        payload.update(floor='4F', room='402')
    shared.request('goal', payload)
    executor_stub.process_command(shared.commands.get_nowait())
    goal = shared.state['goal']
    assert goal['status'] == 'SENDING'
    if with_room:
        assert goal['floor'] == '4F' and goal['room'] == '402'
    else:
        assert not goal.get('floor') and not goal.get('room')


@pytest.mark.parametrize('environment', ['1234', '', None])
def test_pin_environment_only_without_logging(monkeypatch, capsys, environment):
    from amr_web_ui import server
    captured = []

    def stop(**_kwargs):
        raise KeyboardInterrupt

    def fake_server(shared, *_args):
        captured.append(shared)
        return SimpleNamespace(server_port=8080, serve_forever=stop, server_close=lambda: None)

    if environment is None:
        monkeypatch.delenv('AMR_KIOSK_PIN', raising=False)
    else:
        monkeypatch.setenv('AMR_KIOSK_PIN', environment)
    monkeypatch.setattr(server, 'load_preview', lambda *_args: None)
    monkeypatch.setattr(server.SharedState, 'set_building', lambda *_args, **_kwargs: None)
    monkeypatch.setattr(server, 'create_http_server', fake_server)
    server.main(['--preview-map', 'unused'])
    assert captured[0].kiosk_pin == (environment or '')
    output = capsys.readouterr()
    assert '1234' not in output.out+output.err


def test_delivery_announcement_keys():
    from amr_web_ui.server import delivery_announcement
    goal = lambda status, **kw: dict(room='401', floor='4F', kind='goal', status=status, **kw)  # noqa: E731
    assert delivery_announcement(goal('ACCEPTED'), 'SENDING') == 'delivery_start'
    assert delivery_announcement(goal('ACCEPTED'), 'CANCELING') is None  # Cancel refused: not a new start.
    assert delivery_announcement(goal('ACCEPTED'), 'ACCEPTED') is None
    assert delivery_announcement(goal('SUCCEEDED'), 'ACCEPTED') == 'room_arrival'
    assert delivery_announcement(goal('CANCELED'), 'CANCELING') == 'delivery_cancel'
    assert delivery_announcement(goal('FAILED'), 'ACCEPTED') == 'robot_error'
    assert delivery_announcement(goal('FAILED', error_message='E-stop으로 자율주행을 중단했습니다.'),
                                 'ACCEPTED') is None
    assert delivery_announcement(goal('FAILED', error_message='수동 조작(LB)으로 자율주행을 중단했습니다.'),
                                 'ACCEPTED') is None
    assert delivery_announcement(dict(goal('FAILED'), kind='floor_goal'), 'ACCEPTED') is None
    assert delivery_announcement(dict(goal('SUCCEEDED'), room=''), 'ACCEPTED') is None  # Map click.


def test_acknowledged_delivery_announces_received_once(shared, http):
    calls = []
    shared.announce = lambda key, goal: calls.append((key, goal['room']))
    shared.state['preview'] = False
    with http(shared) as request:
        shared.state['goal'] = dict(request_id='delivery_001', status='SUCCEEDED', room='401')
        for _ in range(2):
            assert request('/api/acknowledge', {'request_id': 'delivery_001'})[0] == 202
        shared.state['goal'] = dict(request_id='delivery_002', status='FAILED', room='401')
        assert request('/api/acknowledge', {'request_id': 'delivery_002'})[0] == 202
    assert calls == [('received', '401')]
