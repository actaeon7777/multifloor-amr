"""Multi-floor destinations: floor list, other-floor maps and cross-floor goal admission."""
import dataclasses
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
    state.state['floor'] = dict(zone=1, floor='4F', confidence='CONFIRMED')
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
    assert not third['provisional'] and third['selectable'] and third['real_enabled']
    assert all(f['real_enabled'] for f in info['floors'])
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


def test_real_vehicle_blocks_disabled_floors_and_unready_missions(shared):
    shared.state['preview'] = False
    shared.simulation = False
    # Every mapped floor is enabled now; disable one to keep the refusal covered.
    building = shared.building
    shared.building = dataclasses.replace(building, floors={
        **building.floors, '5F': dataclasses.replace(building.floors['5F'], real_enabled=False)})
    with pytest.raises(ApiError, match='실차 층간 이동이 꺼져'):
        goal(shared, '5F')
    shared.building = building
    shared.simulation = True
    now = time.time()
    shared.update_localization(json.dumps(dict(state='READY', ready=True, detail='ok', source='manual',
                                               stamp=now, generation=1, session='s')))
    shared.state.update(action_ready=True, mode='AUTO',
                        robot=dict(x=0, y=0, yaw=0, received_at=time.monotonic()))
    assert shared.readiness_error('floor_goal') == '층간 이동 관리자에 연결되지 않았습니다.'
    shared.state['floor_action_ready'] = True
    shared.state['floor'] = dict(zone=2, floor='4F', confidence='CONFIRMED')
    assert '현재 층이 확인되지' in shared.readiness_error('floor_goal')
    shared.state['floor'] = dict(zone=1, floor='4F', confidence='CONFIRMED')
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


def test_pass_floors_are_not_destinations(shared):
    names = [f['name'] for f in shared.floors_info()['floors']]
    assert names == ['3F', '4F', '5F']


def mission_status(phase, **overrides):
    status = dict(phase=phase, detail='', goal_floor='3F', goal_room='302', start_floor='4F',
                  current_floor='4F', zone=1, confidence='CONFIRMED', door=1,
                  counters={'button_retries': 1, 'bad': 'x', 'flag': True}, timings={'NAV_TO_HALL': 12.5, 'x': 'y'},
                  elapsed=3.)
    status.update(overrides)
    return json.dumps(status)


def test_floor_transit_mission_steps_and_event_log(shared):
    assert shared.snapshot()['transit']['mission'] is None
    assert shared.update_floor_mission(mission_status('SUCCEEDED'))  # latched result of an older mission
    assert not shared.update_floor_mission('garbage') and not shared.update_floor_mission('{"phase": 3}')
    assert shared.update_floor_mission(mission_status('NAV_TO_HALL'))
    mission = shared.state['floor_mission']
    assert mission['counters'] == {'button_retries': 1}  # bool and str values dropped
    assert mission['timings'] == {'NAV_TO_HALL': 12.5} and mission['door'] == 1
    shared.update_floor_mission(mission_status('NAV_TO_HALL', door=3))  # same phase: no new event
    shared.update_floor_mission(mission_status('CALL_ELEVATOR'))
    shared.update_elevator_request(1, '4F', '3F', 2, 21)
    shared.update_elevator_request(2, '', '3F', 0, 22)
    transit = shared.snapshot()['transit']
    request = transit['request']
    assert (request['sequence'], request['kind'], request['target_floor']) == (22, 'CAR_FLOOR', '3F')
    assert request['age'] >= 0
    assert transit['mission']['phases'] == ['NAV_TO_HALL', 'CALL_ELEVATOR']
    assert transit['mission']['phase_age'] >= 0 and transit['mission']['elapsed'] >= 3.
    kinds = [e['kind'] for e in transit['events']]
    assert kinds == ['phase', 'phase', 'request', 'request']
    json.dumps(transit, allow_nan=False)
    shared.update_floor_mission(mission_status('FAILED_SAFE', detail='DOOR_TIMEOUT'))
    shared.update_floor_mission(mission_status('NAV_TO_HALL'))  # a new mission starts a fresh log
    transit = shared.snapshot()['transit']
    assert [e['kind'] for e in transit['events']] == ['phase'] and transit['request']['kind'] == 'CAR_FLOOR'
    assert transit['mission']['phases'] == ['NAV_TO_HALL']


def test_floor_transit_door_is_logged_once_it_settles(shared, monkeypatch):
    now = [100.]
    monkeypatch.setattr(time, 'monotonic', lambda: now[0])
    shared.update_door(3, .9, 40)
    shared.update_door(3, .9, 40)  # idle: shown, not logged
    assert shared.snapshot()['transit']['door']['state'] == 3 and not shared.transit_events
    shared.update_floor_mission(mission_status('WAIT_DOOR_OPEN'))
    for state, step in ((1, 0.), (1, .2), (3, .1), (3, .3), (3, .3), (3, .1)):
        now[0] += step
        shared.update_door(state, .5, 40)
    assert [e['state'] for e in shared.transit_events if e['kind'] == 'door'] == [3]
    now[0] += 5
    assert shared.snapshot()['transit']['door']['age'] == 5


def test_one_unreadable_floor_map_disables_only_that_floor(monkeypatch):
    """A malformed map (any exception type) fails its own floor, not the floor list."""
    from amr_stack import floor_config
    building = load_floor_config()
    real = floor_config.load_occupancy_map

    def load(path):
        if path == building.map_path('3F', simulation=True):
            raise KeyError('resolution')  # e.g. a YAML key missing from 3F
        return real(path)

    monkeypatch.setattr(floor_config, 'load_occupancy_map', load)
    state = SharedState(preview=True)
    state.set_building(building, simulation=True)
    info = state.floors_info()
    maps = {f['name']: f['map'] for f in info['floors']}
    assert maps['3F'] is None and maps['4F'] is not None and maps['5F'] is not None
    for _ in range(3):  # the stored error is text: every request raises a fresh ApiError
        with pytest.raises(ApiError) as error:
            state.floor_grid('3F')
        assert error.value.status == 503 and 'KeyError' in str(error.value)


def test_pass_floors_exposed_for_arrival_picker(shared):
    info = shared.floors_info()
    assert info['pass_floors'] == [dict(name=f.name, level=f.level) for f in
        sorted(shared.building.pass_floors.values(), key=lambda f: f.level)]
    assert not {f['name'] for f in info['pass_floors']} & {f['name'] for f in info['floors']}
