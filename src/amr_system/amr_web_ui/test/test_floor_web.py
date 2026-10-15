"""Multi-floor destinations: floor list, other-floor maps and cross-floor goal admission."""
import io
import json
import time
from urllib.error import HTTPError
from urllib.request import urlopen

from PIL import Image
import pytest

from amr_stack.floor_config import load_floor_config
from amr_web_ui.server import ApiError, SharedState, create_http_server, load_preview


@pytest.fixture
def shared():
    building = load_floor_config()
    state = SharedState(preview=True)
    load_preview(state, building.map_path('4F', simulation=True))
    state.set_building(building, simulation=True)
    state.state['floor'] = dict(zone=1, floor='4F', confidence='BEACON')
    return state


def goal(shared, floor, room='', version=None, x=33.73, y=-.2, request='floor_goal_01'):
    payload = dict(request_id=request, x=x, y=y, yaw=-1.57, floor=floor, room=room,
                   map_version=version if version is not None else shared.floor_grid(floor).version)
    return shared.request('goal', payload)


def test_floor_list_rooms_and_maps(shared):
    info = shared.floors_info()
    assert [f['name'] for f in info['floors']] == ['3F', '4F', '5F']
    assert info['current_floor'] == '4F' and info['default_floor'] == '4F'
    third = info['floors'][0]
    assert third['provisional'] and third['selectable'] and not third['real_enabled']
    assert [r['id'] for r in third['rooms']][:2] == ['301', '302']
    assert third['map']['width'] == 1014 and third['map']['resolution'] == .05
    versions = {f['name']: f['map']['version'] for f in info['floors']}
    assert len(set(versions.values())) == 3  # identical copies still get distinct versions


def test_cross_floor_goal_is_routed_to_the_floor_mission(shared):
    record = goal(shared, '3F', '302')
    assert record['status'] == 'PREVIEW' and record['kind'] == 'floor_goal'
    assert shared.state['goal']['floor'] == '3F' and shared.state['goal']['room'] == '302'
    same = goal(shared, '4F', request='floor_goal_02', version=shared.grids['map'].version)
    assert same['kind'] == 'goal'


def test_cross_floor_goal_validation(shared):
    with pytest.raises(ApiError, match='지도가 갱신'):
        goal(shared, '5F', version=1)
    with pytest.raises(ApiError, match='벽이나 미확인'):
        goal(shared, '5F', x=20., y=-1.17)
    with pytest.raises(ApiError, match='알 수 없는 층'):
        goal(shared, '9F', version=0)
    with pytest.raises(ApiError, match='층과 호실'):
        shared.request('initialpose', dict(request_id='floor_goal_03', x=1, y=1, yaw=0, floor='3F'))
    shared.state['floor'] = None
    with pytest.raises(ApiError, match='현재 층 정보'):
        goal(shared, '3F', request='floor_goal_04')


def test_real_vehicle_blocks_provisional_floors_and_unready_missions(shared):
    shared.state['preview'] = False
    shared.simulation = False
    with pytest.raises(ApiError, match='임시 지도'):
        goal(shared, '3F')
    shared.simulation = True
    now = time.time()
    shared.update_localization(json.dumps(dict(state='READY', ready=True, detail='ok', source='manual',
                                               stamp=now, generation=1, session='s')))
    shared.state.update(action_ready=True, robot=dict(x=0, y=0, yaw=0, received_at=time.monotonic()))
    assert shared.readiness_error('floor_goal') == '층간 이동 관리자에 연결되지 않았습니다.'
    shared.state['floor_action_ready'] = True
    shared.state['floor'] = dict(zone=2, floor='4F', confidence='BEACON')
    assert '현재 층이 확인되지' in shared.readiness_error('floor_goal')
    shared.state['floor'] = dict(zone=1, floor='4F', confidence='BEACON')
    shared.state['floor_mission'] = dict(phase='RIDING')
    assert '진행 중인 층간 이동' in shared.readiness_error('floor_goal')
    shared.state['floor_mission'] = dict(phase='SUCCEEDED')
    assert shared.readiness_error('floor_goal') is None
    record = goal(shared, '3F', '305', request='floor_goal_05')
    assert record['status'] == 'QUEUED'
    command = shared.commands.get_nowait()
    assert (command['kind'], command['floor'], command['room']) == ('floor_goal', '3F', '305')


def test_http_floor_endpoints(shared):
    server = create_http_server(shared, '/nonexistent', port=0)
    import threading
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{server.server_port}'
    try:
        info = json.load(urlopen(base + '/api/floors'))
        assert info['current_floor'] == '4F'
        response = urlopen(base + '/api/floors/5F/map.png')
        assert int(response.headers['X-Map-Version']) == shared.floor_grid('5F').version
        assert Image.open(io.BytesIO(response.read())).size == (1014, 462)
        with pytest.raises(HTTPError) as error:
            urlopen(base + '/api/floors/9F/map.png')
        assert error.value.code == 404
    finally:
        server.shutdown()
        server.server_close()
