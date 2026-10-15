import io
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from PIL import Image
import pytest

from amr_web_ui.geometry import Grid, transform_xy, validate_pose, world_to_cell
from amr_web_ui.server import ApiError, SharedState, create_http_server, load_preview


def test_rotated_map_and_cell_boundaries():
    info = dict(origin=[10, -2, math.pi/2], resolution=.5)
    assert world_to_cell(9.75, -1.25, info) == (1, 0)
    grid = Grid(2, 2, .5, [10, -2, math.pi/2], [0, 100, -1, 0])
    assert grid.cell(9.75, -1.75) == 0
    assert grid.cell(9.75, -1.25) == 100
    with pytest.raises(ValueError):
        grid.cell(10.25, -1.75)


def test_restart_retires_old_ready_and_clears_queued_commands():
    shared = SharedState()
    first = dict(state='READY', ready=True, detail='ready', source='auto',
                 stamp=time.time(), generation=999, session='old')
    shared.update_localization(json.dumps(first))
    shared.state['robot'] = dict(x=10, y=20, yaw=0, received_at=time.monotonic())
    shared.state['scan'] = dict(points=[[1, 2]], received_at=time.monotonic())
    shared.requests['queued_goal'] = dict(status='QUEUED', signature=None)
    shared.commands.put(dict(kind='goal', request_id='queued_goal'))
    shared.update_localization(json.dumps({**first, 'session': 'new', 'generation': 0,
                                          'stamp': time.time()}))
    assert shared.localization_session == 'new'
    assert shared.state['robot'] is None and shared.state['scan'] is None
    assert shared.commands.empty() and shared.requests['queued_goal']['status'] == 'FAILED'
    shared.update_localization(json.dumps({**first, 'stamp': time.time(), 'generation': 1000}))
    assert shared.localization_session == 'new'


def test_png_orientation_and_partial_costmap_update():
    grid = Grid(2, 2, 1., [0, 0, 0], [0, 100, -1, 50])
    image = Image.open(io.BytesIO(grid.png))
    assert list(image.getdata()) == [188, 140, 249, 30]
    grid.update(0, 0, 1, 1, [100])
    assert grid.cell(.5, .5) == 100
    with pytest.raises(ValueError):
        grid.update(1, 1, 2, 2, [0]*4)
    cost = Grid(2, 1, 1., [0, 0, 0], [0, 100], costmap=True)
    assert Image.open(io.BytesIO(cost.png)).getpixel((0, 0))[3] == 0


def test_lookup_table_preserves_every_occupancy_pixel():
    values = list(range(-128, 128))
    grid = Grid(256, 1, 1., [0, 0, 0], values)
    assert list(Image.open(io.BytesIO(grid.png)).getdata()) == [
        188 if value < 0 else round(249-min(100, value)*2.19) for value in values]
    cost = Grid(256, 1, 1., [0, 0, 0], values, costmap=True)
    expected = [(0, 0, 0, 0) if value <= 0 else
                (159, 39, 111, 200) if value >= 99 else
                (90+int(value*1.1), 95, 220, 30+int(value*1.2)) for value in values]
    assert list(Image.open(io.BytesIO(cost.png)).getdata()) == expected


def test_full_quaternion_scan_projection():
    q = SimpleNamespace(x=0, y=0, z=math.sin(math.pi/4), w=math.cos(math.pi/4))
    tf = SimpleNamespace(rotation=q, translation=SimpleNamespace(x=2, y=3))
    assert transform_xy(1, 0, tf) == pytest.approx([2, 4])


@pytest.mark.parametrize('value', [float('nan'), float('inf'), True, '1', None])
def test_invalid_pose(value):
    with pytest.raises(ValueError):
        validate_pose(dict(x=value, y=0, yaw=0))


def live_state():
    shared = SharedState()
    shared.grids['map'] = Grid(3, 2, 1., [0, 0, 0], [0, 100, -1, 0, 0, 0])
    shared.state.update(action_ready=True, robot=dict(x=0, y=0, yaw=0, received_at=time.monotonic()),
                        velocity=dict(linear=0, angular=0, received_at=time.monotonic()))
    shared.update_localization(localization_status())
    return shared


def localization_status(state='READY', generation=1, **values):
    return json.dumps(dict(state=state, ready=state == 'READY', detail='상태 안내', source='auto',
                           stamp=time.time(), generation=generation, session='test_session', **values))


def test_command_validation_idempotency_and_pending_request():
    shared = live_state()
    payload = dict(request_id='request_001', x=.5, y=.5, yaw=0, map_version=1)
    first = shared.request('goal', payload)
    assert shared.request('goal', payload) == first
    assert shared.commands.qsize() == 1
    with pytest.raises(ApiError) as exc:
        shared.request('goal', dict(payload, request_id='request_002'))
    assert exc.value.status == 409
    shared.complete('request_001', 'ACCEPTED')
    with pytest.raises(ApiError):
        shared.request('goal', dict(payload, x=1.5))
    for x in (1.5, 2.5, 4):
        with pytest.raises((ApiError, ValueError)):
            shared.request('goal', dict(payload, request_id='request_002', x=x))
    with pytest.raises(ApiError):
        shared.request('goal', dict(payload, request_id='request_003', map_version=0))
    shared.state['robot']['received_at'] -= 5
    with pytest.raises(ApiError):
        shared.request('goal', dict(payload, request_id='request_004'))


def test_http_assets_commands_and_origin(tmp_path):
    shared = SharedState(preview=True)
    shared.grids['map'] = Grid(2, 2, 1, [0, 0, 0], [0]*4)
    assets = Path(__file__).resolve().parents[1]/'web'
    server = create_http_server(shared, assets, port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f'http://127.0.0.1:{server.server_port}'
    try:
        assert b'AMR' in urlopen(url).read()
        assert json.load(urlopen(url+'/api/state'))['preview']
        image = urlopen(url+'/api/layers/map.png')
        assert image.headers['X-Map-Version'] == '1'
        assert Image.open(io.BytesIO(image.read())).size == (2, 2)
        headers = {'Content-Type':'application/json', 'X-AMR-Request':'1'}
        payload = json.dumps(dict(request_id='http_request_1', x=.5, y=.5, yaw=0, map_version=1)).encode()
        result = json.load(urlopen(Request(url+'/api/goal', data=payload, headers=headers)))
        assert result['status'] == 'PREVIEW'
        assert shared.commands.empty()
        for bad_headers in ({'Content-Type':'application/json'}, dict(headers, Origin='https://elsewhere.test')):
            with pytest.raises(HTTPError) as exc:
                urlopen(Request(url+'/api/goal', data=payload, headers=bad_headers))
            assert exc.value.code == 403
        with pytest.raises(HTTPError) as exc:
            urlopen(url+'/../../server.py')
        assert exc.value.code == 404
    finally:
        server.shutdown(); server.server_close(); thread.join()


def test_saved_map_preview_uses_real_map():
    filename = Path(__file__).resolve().parents[2]/'amr_stack/maps/G_4_floor/4_floor_0522.yaml'
    shared = SharedState(preview=True)
    load_preview(shared, filename)
    assert shared.grids['map'].info['width'] == 1014
    assert {0, 100} <= set(shared.grids['map'].data) <= {-1, 0, 100}


@pytest.mark.parametrize('state', ['WAITING_DATA', 'SEARCHING', 'VERIFYING', 'WAITING_INITIAL_POSE',
                                   'STARTING_NAVIGATION', 'FAILED'])
def test_supervisor_readiness_is_required_even_with_map_tf_and_action(state):
    shared = live_state()
    shared.update_localization(localization_status(state))
    with pytest.raises(ApiError, match='위치 확인'):
        shared.request('goal', dict(request_id='blocked_goal', x=.5, y=.5, yaw=0, map_version=1))
    assert shared.commands.empty()


def test_tracking_warning_is_visible_without_changing_supervisor_readiness():
    shared = live_state()
    warning = '지도와 LiDAR 스캔이 충분히 정렬되지 않았습니다.'
    shared.update_localization(localization_status(
        tracking_warning=warning, prior_verified=True,
        tracking_bad_samples=2, tracking_bad_duration=.2,
    ))
    status = shared.snapshot()['localization']
    assert status['tracking_warning'] == warning
    assert status['prior_verified'] is True
    assert status['tracking_bad_samples'] == 2
    assert status['tracking_bad_duration'] == .2
    assert status['ready'] is True
    assert shared.readiness_error('goal') is None
    shared.request('goal', dict(request_id='warning_goal', x=.5, y=.5, yaw=0, map_version=1))
    assert shared.commands.qsize() == 1


def test_tracking_recovery_state_still_blocks_goals_and_clears_old_warning():
    shared = live_state()
    shared.update_localization(localization_status(
        'VERIFYING', prior_verified=True, tracking_warning='스캔 정렬 저하',
        tracking_bad_samples=3, tracking_bad_duration=.3,
    ))
    assert not shared.snapshot()['localization']['ready']
    with pytest.raises(ApiError, match='위치 확인'):
        shared.request('goal', dict(request_id='blocked_warning', x=.5, y=.5, yaw=0, map_version=1))
    assert shared.commands.empty()
    # Legacy publishers may omit optional diagnostics; stale warnings must not linger.
    shared.update_localization(localization_status())
    status = shared.snapshot()['localization']
    assert status['ready'] is True
    assert status['tracking_warning'] == ''
    assert status['tracking_bad_samples'] == 0
    assert status['tracking_bad_duration'] == 0.0


@pytest.mark.parametrize('diagnostic', [
    dict(tracking_warning=[]), dict(prior_verified='true'),
    dict(tracking_bad_samples=True), dict(tracking_bad_samples=-1),
    dict(tracking_bad_samples=1.5), dict(tracking_bad_duration=True),
    dict(tracking_bad_duration=-.1), dict(tracking_bad_duration=float('nan')),
])
def test_malformed_tracking_diagnostics_cannot_grant_readiness(diagnostic):
    shared = live_state()
    shared.update_localization(localization_status(**diagnostic))
    assert not shared.snapshot()['localization']['ready']
    assert shared.readiness_error('goal')


def test_tracking_warning_is_bounded_and_delayed_warning_cannot_overwrite_recovery():
    shared = live_state()
    old_warning = localization_status(tracking_warning='x' * 1500, prior_verified=True)
    shared.update_localization(old_warning)
    assert len(shared.snapshot()['localization']['tracking_warning']) == 1000
    shared.update_localization(localization_status(tracking_warning='', prior_verified=True))
    shared.update_localization(old_warning)
    assert shared.snapshot()['localization']['tracking_warning'] == ''


def test_stale_status_and_tf_fail_closed_on_http_and_execution_check():
    shared = live_state()
    payload = dict(request_id='queued_goal', x=.5, y=.5, yaw=0, map_version=1)
    shared.request('goal', payload)
    assert shared.readiness_error('goal', 'queued_goal') is None
    shared.state['localization']['received_at'] -= 3
    assert not shared.snapshot()['localization']['ready']
    assert shared.readiness_error('goal', 'queued_goal')
    shared.update_localization(localization_status())
    shared.state['localization']['stamp'] -= 3
    assert not shared.snapshot()['localization']['ready']
    assert shared.readiness_error('goal', 'queued_goal')
    shared.update_localization(localization_status())
    shared.state['tf_error'] = '변환 대기'
    assert shared.readiness_error('goal', 'queued_goal')


@pytest.mark.parametrize('text', ['garbage', '[]', 'null', '{"state": []}',
                                  '{"state":"READY","ready":true,"detail":"missing metadata"}'])
def test_malformed_localization_status_revokes_ready(text):
    shared = live_state()
    shared.update_localization(text)
    assert not shared.snapshot()['localization']['ready']
    assert shared.readiness_error('goal')


@pytest.mark.parametrize('kind', ['initialpose', 'relocalize'])
@pytest.mark.parametrize('navigation', ['WAITING_AUTO', 'WAITING_NAV2', 'RUNNING', 'PAUSED', 'STOPPING'])
def test_position_reset_refuses_pending_or_paused_navigation(kind, navigation):
    shared = live_state()
    shared.state['navigation'] = navigation
    with pytest.raises(ApiError, match='목표'):
        shared.request(kind, dict(request_id='reset_blocked', map_version=1, x=.5, y=.5, yaw=0))


@pytest.mark.parametrize('status', ['SENDING', 'ACCEPTED', 'CANCELING'])
def test_position_reset_refuses_current_web_goal_even_if_navigation_is_idle(status):
    shared = live_state()
    shared.state['goal'] = dict(request_id='other_goal', status=status)
    with pytest.raises(ApiError, match='목표'):
        shared.request('relocalize', dict(request_id='reset_blocked', map_version=1))


@pytest.mark.parametrize('velocity', [None, dict(linear=0, angular=0, received_at=0),
                                    dict(linear=.03, angular=0, received_at=time.monotonic()),
                                    dict(linear=0, angular=.04, received_at=time.monotonic())])
def test_position_reset_requires_fresh_stationary_odometry(velocity):
    shared = live_state()
    shared.state['velocity'] = velocity
    with pytest.raises(ApiError):
        shared.request('relocalize', dict(request_id='reset_stationary', map_version=1))
    assert shared.commands.empty()


def test_reset_is_idempotent_and_stale_ready_cannot_release_barrier():
    shared = live_state()
    payload = dict(request_id='auto_locate', map_version=1)
    first = shared.request('relocalize', payload)
    assert shared.request('relocalize', payload) == first
    assert shared.commands.qsize() == 1
    assert not shared.snapshot()['localization']['ready']
    shared.complete('auto_locate', 'STARTED')
    for state in ('READY', 'VERIFYING', 'READY'):
        shared.update_localization(localization_status(state, generation=1))
        assert shared.state['localization_reset_pending']
        assert shared.readiness_error('goal')
    shared.update_localization(localization_status('SEARCHING', generation=2))
    assert not shared.state['localization_reset_pending']
    assert shared.readiness_error('goal')
    shared.update_localization(localization_status(generation=2))
    assert shared.readiness_error('goal') is None


def test_manual_pose_blocks_ready_until_supervisor_consumes_new_generation():
    shared = live_state()
    shared.request('initialpose', dict(request_id='manual_locate', map_version=1, x=.5, y=.5, yaw=0))
    shared.complete('manual_locate', 'PUBLISHED')
    shared.update_localization(localization_status())
    assert shared.readiness_error('goal')
    shared.update_localization(localization_status(generation=2))
    assert shared.readiness_error('goal') is None


def test_old_ready_cannot_reopen_after_new_attempt_clears_reset_barrier():
    shared = live_state()
    shared.request('relocalize', dict(request_id='new_attempt_reset', map_version=1))
    shared.complete('new_attempt_reset', 'STARTED')
    shared.update_localization(localization_status('SEARCHING', generation=2))
    assert not shared.state['localization_reset_pending']
    shared.update_localization(localization_status(generation=1))
    assert shared.state['localization']['state'] == 'SEARCHING'
    assert shared.readiness_error('goal')
    shared.update_localization(localization_status(generation=2))
    assert shared.readiness_error('goal') is None


def test_out_of_order_heartbeat_cannot_reopen_same_generation():
    shared = live_state()
    old_ready = localization_status(generation=2)
    shared.update_localization(localization_status('VERIFYING', generation=2))
    shared.update_localization(old_ready)
    assert shared.state['localization']['state'] == 'VERIFYING'
    assert shared.readiness_error('goal')


def test_malformed_status_does_not_forget_generation_watermark():
    shared = live_state()
    shared.update_localization(localization_status('SEARCHING', generation=2))
    shared.update_localization('malformed')
    shared.update_localization(localization_status(generation=1))
    assert shared.state['localization']['state'] == 'FAILED'
    assert shared.readiness_error('goal')
    shared.update_localization(localization_status(generation=2))
    assert shared.readiness_error('goal') is None


def test_reset_after_malformed_status_keeps_last_valid_generation_barrier():
    shared = live_state()
    shared.update_localization('malformed')
    shared.begin_localization_reset()
    shared.update_localization(localization_status('VERIFYING', generation=1))
    shared.update_localization(localization_status(generation=1))
    assert shared.state['localization_reset_pending']
    assert shared.readiness_error('goal')
    shared.update_localization(localization_status(generation=2))
    assert shared.readiness_error('goal') is None


def test_relocalize_map_version_is_checked_without_pose_payload():
    shared = live_state()
    with pytest.raises(ApiError, match='지도'):
        shared.request('relocalize', dict(request_id='stale_map_reset', map_version=0))
    shared.grids.clear()
    assert shared.request('relocalize', dict(request_id='no_map_reset'))['status'] == 'QUEUED'
