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
    state.building = SimpleNamespace(home=SimpleNamespace(floor='4F', room='402'), floors=floors, floor=lambda name: floors[name])
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
                   'robot global_path tf_error estop telemetry acknowledged_request kiosk map_version hold task'.split())
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


@pytest.mark.parametrize('preview', [True, False])
def test_operator_hold_http_idempotency_and_local_persistence(shared, http, preview):
    shared.state.update(preview=preview, goal=dict(request_id='old_goal_001', status='SENDING'))
    with http(shared) as request:
        for kind in ('stop', 'resume', 'return_home'):
            assert request('/api/'+kind, {'request_id': 'central_001'})[0] == 403
            assert request('/api/'+kind, {'request_id': 'bad'}, kiosk=False)[0] == 400
        status, body, _ = request('/api/stop', {'request_id': 'central_stop'}, kiosk=False)
        assert (status, body) == (202, dict(request_id='central_stop', held=True, canceled='old_goal_001', pending=False))
        held = dict(shared.state['hold'])
        assert held['held'] and held['since'] > 0 and held['reason'] == 'central'
        assert request('/api/stop', {'request_id': 'central_stop'}, kiosk=False)[1] == body
        assert request('/api/resume', {'request_id': 'central_stop'}, kiosk=False)[0] == 409
        assert request('/api/stop', {'request_id': 'central_stop_again'}, kiosk=False)[0] == 202
        assert shared.state['hold'] == held and shared.hold_sequence == 1
        for endpoint in ('/api/state', '/api/kiosk/state'):
            snapshot = request(endpoint)[1]
            assert snapshot['hold'] == held and snapshot['task'] == '관리자 정지'
        shared.reset_connection('synthetic link reset')
        assert shared.state['hold'] == held
        status, body, _ = request('/api/resume', {'request_id': 'central_resume'}, kiosk=False)
        assert (status, body) == (202, dict(request_id='central_resume', held=False))
        assert request('/api/resume', {'request_id': 'central_resume'}, kiosk=False)[1] == body
        assert shared.state['hold'] == dict(held=False, since=None, reason='central', pending=False)
        assert shared.state['goal'] is None and shared.commands.empty()


def make_ready(shared):
    shared.state.update(preview=False, mode='AUTO', action_ready=True, floor_action_ready=True,
        robot=dict(received_at=time.monotonic()), tf_error=None,
        localization=dict(ready=True, stamp=time.time(), received_at=time.monotonic()))


@pytest.mark.parametrize('preview', [True, False])
@pytest.mark.parametrize('home_floor,home_room,kind', [('4F', '402', 'goal'), ('3F', '302', 'floor_goal')])
def test_return_home_http_uses_room_goal_and_same_id(shared, http, preview, home_floor, home_room, kind):
    shared.building.home = SimpleNamespace(floor=home_floor, room=home_room)
    make_ready(shared)
    shared.state['preview'] = preview
    shared.state['hold'] = dict(held=True, since=time.time(), reason='central')
    shared.state['goal'] = dict(request_id='previous_goal', status='ACCEPTED')
    with http(shared) as request:
        payload = dict(request_id='return_home_001')
        status, body, _ = request('/api/return_home', payload, kiosk=False)
        assert (status, body) == (202, dict(request_id=payload['request_id'], home=dict(floor=home_floor, room=home_room)))
        assert request('/api/return_home', payload, kiosk=False)[1] == body
        assert not shared.state['hold']['held']
        if preview:
            goal = shared.state['goal']
            assert shared.pending_return is None and shared.commands.empty()
        else:
            assert 'previous_goal' in shared.cancel_goal_ids
            assert shared.next_return_command() is None
            shared.state['goal']['status'] = 'CANCELED'
            # The hold release and the goal race otherwise: dispatch waits 0.5 s after the request.
            assert shared.next_return_command() is None and shared.pending_return is not None
            goal = shared.next_return_command(now=shared.pending_return['started'] + 1)
            assert shared.pending_return is None
            assert goal['map_version'] == (shared.grids['map'].version if kind == 'goal'
                                           else shared.floor_grids[home_floor].version)
        assert goal['request_id'] == payload['request_id'] and goal['origin'] == 'return_home'
        assert (goal['kind'], goal['floor'], goal['room']) == (kind, home_floor, home_room)


def test_return_missing_home_and_timeout(shared, http):
    shared.state['preview'] = False
    with http(shared) as request:
        home = shared.building.home
        shared.building.home = None
        assert request('/api/return_home', dict(request_id='missing_home'), kiosk=False)[:2] == (
            404, {'error': '복귀 지점이 설정되지 않았습니다.'})
        shared.building.home = home
        assert request('/api/return_home', dict(request_id='timeout_home'), kiosk=False)[0] == 202
        assert shared.next_return_command() is None
        assert shared.next_return_command(shared.pending_return['started']+30.) is None
        assert shared.pending_return is None and shared.state['pending_request'] is None
        assert shared.state['notice']['status'] == 'FAILED'
        assert shared.state['notice']['detail'] == '복귀를 시작하지 못했습니다: 제어 모드 수신을 기다리고 있습니다.'


@pytest.mark.parametrize('new_kind', ['stop', 'return_home', 'goal'])
def test_pending_return_superseded(shared, new_kind):
    make_ready(shared)
    shared.request('return_home', dict(request_id='old_return'))
    payload = pose(shared, request_id='new_command') if new_kind == 'goal' else dict(request_id='new_command')
    shared.request(new_kind, payload)
    assert shared.requests['old_return']['status'] == 'CANCELED'
    assert (shared.pending_return or {}).get('request_id') == ('new_command' if new_kind == 'return_home' else None)


def test_stop_works_when_normal_queue_full_and_discards_queued_goals(shared):
    make_ready(shared)
    for i in range(32):
        shared.commands.put_nowait(dict(kind='goal', request_id=f'queued_{i:03d}'))
    shared.request('stop', dict(request_id='stop_full_queue'))
    assert shared.commands.empty() and shared.state['hold']['held']
    shared.complete('stop_full_queue', 'SUCCEEDED')
    assert shared.readiness_error('goal').startswith('관리자가')
    shared.request('resume', dict(request_id='resume_full_queue'))
    assert shared.commands.empty()


@pytest.mark.parametrize('changes,expected', [
    ({'estop': {'error': 'fault'}, 'hold': {'held': True}}, '비상 정지'),
    ({'estop': {'state': 'ESTOP'}}, '비상 정지'),
    ({'hold': {'held': True}, 'mode': 'MANUAL'}, '관리자 정지'),
    ({'mode': 'MANUAL'}, '수동 조작'),
    ({'goal': {'status': 'ACCEPTED', 'origin': 'return_home', 'room': '401'}}, '복귀 중 (401호)'),
    ({'goal': {'status': 'ACCEPTED', 'origin': 'return_home', 'room': '401'}, 'floor_mission': {'phase': 'RIDING'}}, '복귀 중 (401호) · 이동'),
    ({'goal': {'status': 'CANCELING', 'room': '402'}, 'floor_mission': {'phase': 'NAV_TO_HALL'}}, '402호 배송 중 · 엘리베이터 홀 이동 (취소 중)'),
    ({'goal': {'status': 'SENDING', 'room': '402'}}, '402호 배송 중'),
    ({'goal': {'status': 'ACCEPTED'}}, '지정 위치로 이동 중'),
    ({'goal': {'status': 'CANCELING'}}, '지정 위치로 이동 중 (취소 중)'),
    ({'goal': {'status': 'SUCCEEDED', 'room': '402', 'request_id': 'done'}}, '402호 도착 · 수령 대기'),
    ({'goal': {'status': 'FAILED', 'room': '402', 'request_id': 'done'}}, '402호 배송 실패'),
    ({'goal': {'status': 'REJECTED', 'room': '402', 'request_id': 'done'}}, '402호 배송 실패'),
    ({'goal': {'status': 'CANCELED', 'room': '402', 'request_id': 'done'}}, '배송 취소됨'),
    ({'goal': {'status': 'SUCCEEDED', 'room': '402', 'request_id': 'done'}, 'acknowledged_request': 'done'}, '대기'),
    ({'goal': {'status': 'SUCCEEDED', 'origin': 'return_home', 'room': '401'}}, '대기'),
    ({'localization': None}, '위치 확인 필요'),
    ({}, '대기'),
])
def test_task_summary(changes, expected):
    from amr_web_ui.server import task_summary
    state = dict(mode='AUTO', localization=dict(ready=True)) | changes
    before = json.dumps(state)
    assert task_summary(state) == expected
    assert json.dumps(state) == before


def test_operator_announcement_suppression_and_acknowledge(shared, http):
    from amr_web_ui.server import delivery_announcement
    assert delivery_announcement(dict(room='401', kind='goal', status='FAILED',
        error_message='관리자가 원격으로 정지했습니다.'), 'ACCEPTED') is None
    for status in ('ACCEPTED', 'SUCCEEDED', 'CANCELED', 'FAILED', 'REJECTED'):
        assert delivery_announcement(dict(room='401', kind='goal', origin='return_home', status=status), 'SENDING') is None
    calls = []
    shared.announce = lambda *args: calls.append(args)
    shared.state['goal'] = dict(room='401', origin='return_home', status='SUCCEEDED', request_id='home_done')
    with http(shared) as request:
        assert request('/api/acknowledge', dict(request_id='home_done'))[0] == 202
    assert not calls


@pytest.fixture
def operator_bridge(shared):
    """Exercise real bridge callbacks using futures, without creating any ROS node."""
    pytest.importorskip('rclpy')
    from concurrent.futures import Future
    from types import MethodType
    from amr_web_ui.ros_bridge import WebBridge
    from builtin_interfaces.msg import Time
    bridge = SimpleNamespace(shared=shared, goal_id=None, goal_handle=None, emitted=[], announcements=[], sent=[])
    for name in ('process_command', 'send_floor_goal', 'on_goal_response', 'on_result', 'on_floor_result',
                 'goal_state', 'cancel_operator_goal', 'on_operator_cancel_response', 'poll_operator_commands'):
        setattr(bridge, name, MethodType(getattr(WebBridge, name), bridge))
    bridge.set_state = lambda **kw: shared.state.update(kw)
    bridge.announce = lambda *args: bridge.announcements.append(args)
    bridge.hold_pub = SimpleNamespace(publish=lambda m: bridge.emitted.append(json.loads(m.data)))
    bridge.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))
    bridge.on_feedback = lambda *_args: None

    def send_goal(goal, **_kwargs):
        response, result = Future(), Future()
        cancellations = []

        def cancel():
            future = Future()
            cancellations.append(future)
            future.set_result(SimpleNamespace(goals_canceling=[1]))
            return future
        handle = SimpleNamespace(accepted=True, get_result_async=lambda: result, cancel_goal_async=cancel)
        bridge.sent.append(SimpleNamespace(goal=goal, response=response, result=result,
                                          handle=handle, cancellations=cancellations))
        return response
    bridge.client = bridge.floor_client = SimpleNamespace(server_is_ready=lambda: True, send_goal_async=send_goal)
    make_ready(shared)
    return bridge


@pytest.mark.parametrize('floor,room', [('4F', '402'), ('3F', '302')])
def test_stop_sending_goal_cancels_same_handle_even_after_resume(shared, operator_bridge, floor, room):
    bridge = operator_bridge
    version = shared.grids['map'].version if floor == '4F' else shared.floor_grids[floor].version
    shared.request('goal', pose(shared, floor=floor, room=room, map_version=version))
    bridge.process_command(shared.commands.get_nowait())
    assert shared.state['goal']['status'] == 'SENDING'
    shared.request('stop', dict(request_id='stop_sending'))
    shared.request('resume', dict(request_id='resume_sending'))
    bridge.poll_operator_commands()
    assert [m['held'] for m in bridge.emitted] == [True, False]
    assert [m['sequence'] for m in bridge.emitted] == [1, 2]
    sent = bridge.sent[0]
    assert not sent.cancellations
    sent.response.set_result(sent.handle)
    assert len(sent.cancellations) == 1 and shared.state['goal']['status'] == 'CANCELING'
    bridge.poll_operator_commands()
    assert len(sent.cancellations) == 1


@pytest.mark.parametrize('floor,room', [('4F', '402'), ('3F', '302')])
def test_return_dispatch_after_old_goal_terminal_without_ros_node(shared, operator_bridge, floor, room):
    from action_msgs.msg import GoalStatus
    bridge = operator_bridge
    shared.request('goal', pose(shared))
    bridge.process_command(shared.commands.get_nowait())
    old = bridge.sent[0]
    old.response.set_result(old.handle)
    shared.building.home = SimpleNamespace(floor=floor, room=room)
    shared.request('return_home', dict(request_id='bridge_return_home'))
    bridge.poll_operator_commands()
    assert len(old.cancellations) == 1 and len(bridge.sent) == 1
    old.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED,
                                        result=SimpleNamespace(error_code=0, error_msg='')))
    bridge.poll_operator_commands()
    assert len(bridge.sent) == 1  # still inside the 0.5 s hold-release settle time
    shared.pending_return['started'] -= 1
    bridge.poll_operator_commands()
    assert len(bridge.sent) == 2
    goal = shared.state['goal']
    assert goal['request_id'] == 'bridge_return_home' and goal['origin'] == 'return_home'
    assert goal['status'] == 'SENDING' and goal['room'] == room and goal['floor'] == floor
    home = bridge.sent[-1]
    home.response.set_result(home.handle)
    assert not bridge.announcements
    result = SimpleNamespace(error_code=0, error_msg='') if floor == '4F' else SimpleNamespace(
        success=True, error_code='', message='')
    home.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=result))
    assert shared.state['goal']['status'] == 'SUCCEEDED'
    assert not bridge.announcements


def test_hold_snapshot_precedence_and_return_deadline_without_http(shared):
    make_ready(shared)
    shared.request('stop', dict(request_id='snapshot_hold'))
    shared.complete('snapshot_hold', 'SUCCEEDED')
    for kiosk in (False, True):
        snapshot = shared.snapshot(kiosk=kiosk)
        assert snapshot['hold']['held'] and snapshot['task'] == '관리자 정지'
    shared.state['mode'] = 'MANUAL'
    assert shared.readiness_error('goal').startswith('관리자가')
    shared.require_estop = True
    assert not shared.readiness_error('goal').startswith('관리자가')
    shared.require_estop = False
    shared.request('return_home', dict(request_id='timeout_without_http'))
    started = shared.pending_return['started']
    assert shared.next_return_command(started+29.) is None
    assert shared.pending_return
    assert shared.next_return_command(started+30.) is None
    assert shared.pending_return is None
    assert shared.state['notice']['status'] == 'FAILED'
    assert shared.state['notice']['detail'].startswith('복귀를 시작하지 못했습니다: 수동 조작')


def test_home_floors_info_default_and_configured():
    from amr_stack.floor_config import load_floor_config
    shared = SharedState(preview=True)
    assert shared.floors_info()['home'] is None
    from pathlib import Path
    stack = Path(__file__).resolve().parents[2] / 'amr_stack'
    shared.set_building(load_floor_config(stack / 'config/floors.yaml', package_root=stack), simulation=True)
    assert shared.floors_info()['home'] == dict(floor='4F', room='401')


def test_preview_hold_also_blocks_new_goal_without_ros(shared):
    from amr_web_ui.server import ApiError
    shared.request('stop', dict(request_id='preview_hold'))
    with pytest.raises(ApiError, match='관리자가'):
        shared.request('goal', pose(shared))
    shared.request('resume', dict(request_id='preview_release'))
    assert shared.request('goal', pose(shared))['status'] == 'PREVIEW'


def test_stop_during_elevator_segment_is_deferred_until_the_corridor(shared):
    from amr_web_ui.server import task_summary
    shared.state['preview'] = False
    shared.state['goal'] = dict(request_id='ride_goal_01', kind='floor_goal', status='ACCEPTED',
                                floor='3F', room='302')
    shared.state['floor_mission'] = dict(phase='RIDING')
    response = shared.request('stop', dict(request_id='stop_in_ride'))
    assert response['pending'] is True and response['canceled'] is None and response['held'] is False
    assert shared.state['hold']['pending'] and not shared.state['hold']['held']
    assert 'ride_goal_01' not in shared.cancel_goal_ids
    assert shared.operator_commands[-1]['hold'] is None  # nothing published yet
    assert '하차 후 정지 예정' in task_summary(shared.snapshot())
    assert shared.next_hold_event() is None
    for phase in ('SWITCH_MAP', 'ALIGHTING', 'RELOCALIZE'):
        shared.state['floor_mission'] = dict(phase=phase)
        assert shared.next_hold_event() is None
    shared.state['floor_mission'] = dict(phase='NAV_TO_GOAL')
    event = shared.next_hold_event()
    assert event['held'] is True and shared.state['hold'] == dict(
        held=True, since=shared.state['hold']['since'], reason='central', pending=False)
    assert 'ride_goal_01' in shared.cancel_goal_ids
    assert shared.next_hold_event() is None
    # Resume while a hold is still pending simply drops it.
    shared.state['hold'] = dict(held=False, since=1., reason='central', pending=True)
    shared.request('resume', dict(request_id='resume_pending'))
    assert shared.state['hold']['pending'] is False and shared.next_hold_event() is None
    # Hall phases and plain corridor goals stop at once.
    shared.state['floor_mission'] = dict(phase='WAIT_DOOR_OPEN')
    response = shared.request('stop', dict(request_id='stop_at_hall'))
    assert response['pending'] is False and response['canceled'] == 'ride_goal_01'
    assert shared.state['hold']['held']


def test_kiosk_exit_signals_only_the_launcher(shared, http, monkeypatch, tmp_path):
    import os
    import signal
    import subprocess
    from amr_web_ui import server
    killed = []
    monkeypatch.setattr(server.os, 'kill', lambda pid, sig: killed.append((pid, sig)))
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    pid_file = tmp_path/f'amr-kiosk-{os.getuid()}.pid'
    monkeypatch.setattr(server, 'kiosk_browser_pid', lambda: None)
    with http(shared) as request:
        assert request('/api/kiosk/exit', {})[0:2] == (409, {'error': '키오스크 화면을 찾지 못했습니다.'})
        pid_file.write_text(f'{os.getpid()}\n')  # A reused PID that is not the launcher.
        assert request('/api/kiosk/exit', {})[0] == 409
        script = tmp_path/'kiosk/launch.sh'
        script.parent.mkdir()
        script.write_text('sleep 30\n')
        launcher = subprocess.Popen(['bash', str(script)])
        try:
            pid_file.write_text(f'{launcher.pid}\n')
            shared.kiosk_pin = '1234'
            assert request('/api/kiosk/exit', {})[0:2] == (403, {'error': '관리 모드 잠금을 먼저 해제하세요.'})
            token = request('/api/kiosk/unlock', {'pin': '1234'})[1]['token']
            assert request('/api/kiosk/exit', {}, headers={'X-AMR-Kiosk-Token': token})[0:2] == (
                202, {'closing': True})
            assert killed == [(launcher.pid, signal.SIGTERM)]
        finally:
            monkeypatch.undo()
            launcher.kill()
            launcher.wait()


def test_kiosk_exit_falls_back_to_orphaned_browser(shared, http, monkeypatch, tmp_path):
    import signal
    from amr_web_ui import server
    killed = []
    monkeypatch.setattr(server.os, 'kill', lambda pid, sig: killed.append((pid, sig)))
    monkeypatch.setattr(server, 'kiosk_launcher_pid', lambda: None)
    monkeypatch.setattr(server, 'kiosk_browser_pid', lambda: 4321)
    with http(shared) as request:
        assert request('/api/kiosk/exit', {})[0:2] == (202, {'closing': True})
    assert killed == [(4321, signal.SIGTERM)]


def test_kiosk_browser_pid_skips_child_processes(tmp_path):
    from amr_web_ui.server import kiosk_browser_pid
    for pid, argv in [('10', ['chrome', '--type=renderer', '--class=amr-kiosk']),
                      ('11', ['chrome', '--class=amr-kiosk-probe']), ('self', ['x']), ('13', ['bash', '-c', '--class=amr-kiosk']),
                      
                      ('12', ['/s/chrome', '--user-data-dir=/p/amr-kiosk', '--class=amr-kiosk', 'http://k'])]:
        (tmp_path/pid).mkdir()
        (tmp_path/pid/'cmdline').write_bytes(b'\0'.join(a.encode() for a in argv) + b'\0')
    assert kiosk_browser_pid(tmp_path) == 12
    (tmp_path/'12'/'cmdline').write_bytes(b'/s/chrome --user-data-dir=/p/amr-kiosk --class=amr-kiosk http://k\0')
    assert kiosk_browser_pid(tmp_path) == 12  # Chromium's rewritten process title.
    (tmp_path/'12'/'cmdline').unlink()
    assert kiosk_browser_pid(tmp_path) is None
