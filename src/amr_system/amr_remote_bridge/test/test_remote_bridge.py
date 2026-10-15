"""Real loopback HTTP tests: original AMR API -> outbound agent -> central UI API."""
import base64
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import threading
import time

from PIL import Image
import pytest

from amr_web_ui.geometry import Grid
from amr_web_ui.server import SharedState, create_http_server
from amr_remote_bridge.agent import Agent
from amr_remote_bridge.common import HttpClient, PROTOCOL_VERSION, base_url, loads
from amr_remote_bridge.server import ApiError, Registry, create_server, find_assets


TOKEN = 'test-only-secret-' + 'a' * 32
ASSETS = Path(__file__).resolve().parents[2] / 'amr_web_ui' / 'web'
BASIC = 'Basic ' + base64.b64encode(('operator:' + TOKEN).encode()).decode()


def wait_for(predicate, seconds=3):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            result = predicate()
            if result:
                return result
        except (ApiError, KeyError):
            pass
        time.sleep(.01)
    raise AssertionError('Timed out waiting for relay state')


@pytest.fixture
def system():
    shared = SharedState(preview=True)
    shared.grids['map'] = Grid(3, 2, 1, [0, 0, 0], [0, 100, -1, 0, 0, 0])
    # Deliberately different source/public revisions to verify command translation.
    shared.grids['map'].version = 17
    shared.grids['local_costmap'] = Grid(2, 2, .5, [0, 0, 0], [0, 0, 0, 100], costmap=True)
    shared.state.update(robot=dict(x=.5, y=.5, yaw=0, received_at=time.monotonic()),
                        scan=dict(points=[[.1, .2, 4]], received_at=time.monotonic()),
                        velocity=dict(linear=.2, angular=.1, received_at=time.monotonic()),
                        global_path=dict(points=[[.5, .5], [1, 1]], received_at=time.monotonic()),
                        mode='MANUAL', navigation='IDLE')
    calls = []
    original = shared.request

    def record(kind, payload):
        calls.append((kind, deepcopy(payload)))
        return original(kind, payload)

    shared.request = record
    registry = Registry(stale_after=.35, command_timeout=.7, lease_timeout=.4)
    local = create_http_server(shared, ASSETS, port=0)
    central = create_server(registry, ASSETS, TOKEN, port=0)
    threads = []
    for server in (local, central):
        thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .02}, daemon=True)
        thread.start()
        threads.append(thread)
    host_url = f'http://127.0.0.1:{central.server_port}'
    agent = Agent(host_url, TOKEN, local_url=f'http://127.0.0.1:{local.server_port}',
                  rate=20, timeout=.3, layer_refresh=.08)
    agent_thread = None

    def start_agent():
        nonlocal agent_thread
        agent_thread = threading.Thread(target=agent.run, daemon=True)
        agent_thread.start()
        wait_for(lambda: len(registry.snapshot('amr-01')['layers']) == len(shared.grids))

    def request(path, payload=None, headers=None):
        auth = {'Authorization': BASIC}
        if payload is not None:
            auth['X-AMR-Request'] = '1'
        auth.update(headers or {})
        status, result_headers, body = HttpClient().request(host_url + path, payload, auth, timeout=2)
        return status, result_headers, body

    yield dict(shared=shared, registry=registry, local=local, central=central, agent=agent,
               start=start_agent, request=request, calls=calls, url=host_url)
    agent.stop.set()
    if agent_thread:
        agent_thread.join(timeout=2)
        assert not agent_thread.is_alive()
    for server, thread in zip((local, central), threads):
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def payload(request_id='remote_request_001', **kwargs):
    return dict(request_id=request_id, x=.5, y=.5, yaw=.3, map_version=1, **kwargs)


def test_telemetry_images_and_original_ui_are_served(system):
    system['start']()
    status, _, body = system['request']('/robots/amr-01/api/state')
    state = loads(body)
    assert status == 200 and state['preview']
    assert state['scan']['points'] == [[.1, .2, 4]]
    assert state['global_path']['points'] == [[.5, .5], [1, 1]]
    assert state['layers']['map']['version'] == 1
    assert state['layers']['local_costmap']['version'] == 1
    assert state['robot']['age'] >= 0
    status, headers, png = system['request']('/robots/amr-01/api/layers/map.png')
    assert status == 200 and headers['X-Map-Version'] == '1'
    assert png == system['shared'].grids['map'].png
    assert Image.open(io.BytesIO(png)).size == (3, 2)
    status, _, html = system['request']('/robots/amr-01/')
    assert status == 200 and b'/robots/amr-01/app.js' in html
    assert b'href="/fleet"' in html
    status, _, js = system['request']('/robots/amr-01/app.js')
    assert status == 200 and b'/robots/amr-01/api/state' in js
    assert b'fetch(`/robots/amr-01/api/${kind}`' in js
    assert system['request']('/geometry.js')[0] == 200
    assert system['request']('/dashboard.js')[0] == 200
    assert system['request']('/battery-profile.js')[0] == 200
    assert system['request']('/robots/amr-01/dashboard.js')[0] == 200
    assert system['request']('/fleet')[0] == 200


def test_all_three_commands_and_request_deduplication(system):
    system['start']()
    command = payload()
    status, _, body = system['request']('/robots/amr-01/api/goal', command)
    assert status == 202 and loads(body)['status'] == 'PREVIEW'
    assert system['calls'] == [('goal', dict(command, map_version=17))]
    assert system['request']('/robots/amr-01/api/goal', command)[0] == 202
    assert len(system['calls']) == 1
    status, _, _ = system['request']('/api/goal', dict(command, x=1))
    assert status == 409 and len(system['calls']) == 1
    assert system['request']('/api/initialpose', payload('remote_initial_001'))[0] == 202
    assert system['request']('/api/cancel', dict(request_id='remote_cancel_001'))[0] == 202
    wait_for(lambda: system['registry'].snapshot('amr-01')['goal'] is None)
    assert [call[0] for call in system['calls']] == ['goal', 'initialpose', 'cancel']
    assert system['shared'].commands.empty()  # Preview: no ROS or vehicle commands.


def test_local_validation_errors_reach_the_browser(system):
    system['start']()
    assert system['request']('/api/goal', dict(payload(), x=1.5))[0] == 422
    assert system['shared'].state['goal'] is None
    assert system['request']('/api/goal', dict(payload('remote_version_001'), map_version=99))[0] == 409


def test_live_api_queue_receipt_is_not_reported_as_ros_acceptance(system):
    shared = system['shared']
    shared.state.update(preview=False, action_ready=True,
                        robot=dict(x=.5, y=.5, yaw=0, received_at=time.monotonic()))
    shared.update_localization(json.dumps(dict(
        state='READY', ready=True, detail='Synthetic verified pose',
        stamp=time.time(), session='relay-test', generation=1)))
    system['start']()
    status, _, body = system['request']('/api/goal', payload())
    assert status == 202 and loads(body)['status'] == 'QUEUED'
    queued = shared.commands.get_nowait()
    assert queued['map_version'] == 17
    assert queued['kind'] == 'goal' and queued['request_id'] == 'remote_request_001'
    # No ROS executor was started. Only the original web API queue has received the request.
    assert shared.state['goal'] is None
    wait_for(lambda: system['registry'].snapshot('amr-01')['pending_request'])
    shared.complete('remote_request_001', 'REJECTED', 'Fake Nav2 rejection')
    notice = wait_for(lambda: system['registry'].snapshot('amr-01')['notice'])
    assert notice['status'] == 'REJECTED' and notice['detail'] == 'Fake Nav2 rejection'


def test_relocalize_translates_map_version_and_deduplicates(system):
    system['start']()
    command = dict(request_id='remote_relocalize_01', map_version=1)
    status, _, body = system['request']('/robots/amr-01/api/relocalize', command)
    assert status == 202 and loads(body)['status'] == 'PREVIEW'
    assert system['calls'] == [('relocalize', dict(command, map_version=17))]
    assert system['request']('/api/relocalize', command)[0] == 202
    assert len(system['calls']) == 1
    assert system['request']('/api/relocalize', dict(command, map_version=2))[0] == 409
    assert system['request']('/api/relocalize', dict(request_id='bad_map_version_01', map_version=True))[0] == 400
    assert system['shared'].commands.empty()


def test_relocalize_without_map_and_map_arrival_race(system):
    system['shared'].grids.clear()
    system['start']()
    command = dict(request_id='relocalize_no_map_01')
    assert system['request']('/api/relocalize', command)[0] == 202
    assert system['calls'] == [('relocalize', command)]
    # A map appearing after the relay snapshot must be checked by the local API.
    system['shared'].grids['map'] = Grid(3, 2, 1, [0, 0, 0], [0] * 6)
    system['agent'].execute(dict(command_id='relocalize_map_race', kind='relocalize', ttl=1,
                                payload=dict(request_id='relocalize_map_race'), map_digest=''), 0)
    assert system['agent'].results['relocalize_map_race']['http_status'] == 409


def test_relocalize_rejects_changed_map_digest(system):
    system['agent'].execute(dict(command_id='relocalize_digest_01', kind='relocalize', ttl=1,
                                payload=dict(request_id='relocalize_digest_01', map_version=17),
                                map_digest='wrong'), 0)
    assert system['agent'].results['relocalize_digest_01']['http_status'] == 409
    assert not system['calls']


def test_localization_age_includes_relay_delay(system):
    shared = system['shared']
    shared.update_localization(json.dumps(dict(
        state='READY', ready=True, detail='Synthetic verified pose',
        stamp=time.time(), session='age-test', generation=1)))
    system['agent'].exchange()
    registry = system['registry']
    first = registry.snapshot('amr-01')['localization']['age']
    with registry.lock:
        # Keep transport fresh while the localization sample approaches expiry.
        registry.robots['amr-01'].state['localization']['age'] = 1.99
        registry.robots['amr-01'].captured -= .02
    state = registry.snapshot('amr-01')
    assert state['localization']['age'] > first
    assert state['localization']['age'] >= 2
    assert state['localization']['ready'] is False


def test_vehicle_source_lease_is_separate_from_relay_age(system):
    shared = system['shared']
    def sample(**values):
        return dict(age=0.,valid=True,fresh=True,hz=None,**values)
    shared.telemetry.receive_vehicle(dict(
        vehicle=sample(session='vehicle-test',epoch=0,mode='AUTO'),
        wheel=sample(linear=.32),joints=sample(left_rpm=60.,right_rpm=30.),
        localization=sample(state='READY',ready=True),time=sample(healthy=True,timeout=.6)),
        dict(value=0.,epoch=0,valid=True),False)
    system['agent'].exchange()
    registry = system['registry']
    first = registry.snapshot('amr-01')['telemetry']['wheel']['age']
    with registry.lock:
        registry.robots['amr-01'].captured -= .3
        registry.robots['amr-01'].state['time_health'] = dict(age=.1,healthy=True,timeout=.6)
        registry.robots['amr-01'].state['goal'] = dict(feedback_age=.1)
    state = registry.snapshot('amr-01')
    assert state['telemetry']['wheel']['age'] >= first+.3
    assert state['telemetry']['joints']['left_rpm'] == 60.
    assert state['telemetry']['vehicle']['age'] >= .3
    assert state['telemetry']['localization']['age'] >= .3
    assert state['telemetry']['time']['age'] >= .3
    assert state['time_health']['age'] >= .4
    assert state['goal']['feedback_age'] >= .4
    with registry.lock:
        registry.stale_after = 1.
        registry.robots['amr-01'].captured -= .25
    delayed = registry.snapshot('amr-01')
    assert delayed['remote']['timeout'] == 1.
    assert delayed['telemetry']['vehicle']['age'] >= .55
    for name in ('vehicle','wheel','time'):
        assert delayed['telemetry'][name]['fresh']
        assert delayed['telemetry'][name]['source_age'] < .5
    assert delayed['time_health']['age'] >= .65
    # An actual source failure remains a failure even with a live transport.
    with registry.lock:
        registry.robots['amr-01'].state['telemetry']['wheel']['fresh'] = False
    assert not registry.snapshot('amr-01')['telemetry']['wheel']['fresh']
    # Repeated reads never renew the transport or the original source age.
    with registry.lock:
        registry.robots['amr-01'].captured -= .5
    with pytest.raises(ApiError) as expired:
        registry.snapshot('amr-01')
    assert expired.value.status == 503


def test_live_goal_requires_localization_and_relocalize_requires_stop(system):
    shared = system['shared']
    shared.state.update(preview=False, action_ready=True)
    system['start']()
    assert system['request']('/api/goal', payload('unverified_goal_01'))[0] == 409
    assert system['request']('/api/relocalize', dict(request_id='moving_reset_01', map_version=1))[0] == 409
    shared.state['velocity'] = dict(linear=0., angular=0., received_at=time.monotonic())
    status, _, body = system['request']('/api/relocalize', dict(request_id='stopped_reset_01', map_version=1))
    assert status == 202 and loads(body)['status'] == 'QUEUED'
    assert shared.commands.get_nowait()['kind'] == 'relocalize'
    assert shared.state['localization_reset_pending'] is True


def test_operator_auth_agent_auth_and_origin_are_enforced(system):
    raw = HttpClient()
    assert raw.request(system['url'] + '/api/state')[0] == 401
    assert system['request']('/api/state', headers={'Authorization': 'Bearer ' + TOKEN})[0] == 401
    assert system['request']('/api/agent/exchange', {})[0] == 401
    assert system['request']('/api/agent/exchange', {}, {'Authorization': 'Bearer bad'})[0] == 401
    assert system['request']('/api/goal', payload(), {'Origin': 'https://other.test'})[0] == 403
    assert system['request']('/api/goal', payload(), {'X-AMR-Request': ''})[0] == 403
    assert system['request']('/../../agent.py')[0] == 404


def test_stale_telemetry_blocks_commands_and_increases_sensor_age(system):
    system['agent'].exchange()
    first = system['registry'].snapshot('amr-01')['robot']['age']
    time.sleep(.08)
    assert system['registry'].snapshot('amr-01')['robot']['age'] > first + .06
    time.sleep(.3)
    assert system['request']('/api/state')[0] == 503
    assert system['request']('/api/goal', payload())[0] == 503
    assert not system['calls']


def test_local_api_outage_is_not_reported_as_live_telemetry(system, monkeypatch):
    system['agent'].exchange()
    def fail():
        raise ConnectionError('local API unavailable')
    monkeypatch.setattr(system['agent'], 'collect', fail)
    assert not system['agent'].exchange()
    assert system['request']('/api/state')[0] == 503
    roster = loads(system['request']('/api/robots')[2])['robots']
    assert roster[0]['online'] is False and roster[0]['local_api_available'] is False


def test_expired_queued_command_is_never_delivered_on_reconnect(system):
    system['agent'].exchange()
    registry = system['registry']
    registry.command_timeout = .06
    command = registry.queue('amr-01', 'goal', payload())
    assert registry.wait_result('amr-01', command)['http_status'] == 408
    system['agent'].exchange()
    assert not system['calls']
    assert command.status == 'RESULT'


def test_delivered_command_with_lost_ack_is_never_replayed(system):
    agent, registry = system['agent'], system['registry']
    agent.exchange()
    registry.command_timeout = .15
    command = registry.queue('amr-01', 'goal', payload())
    agent.exchange()  # Command executed locally, but its result has not yet been uploaded.
    assert len(system['calls']) == 1
    agent.results.clear()  # Simulate loss of the in-memory acknowledgement.
    assert registry.wait_result('amr-01', command)['http_status'] == 504
    agent.exchange()
    agent.exchange()
    assert len(system['calls']) == 1


def test_agent_command_expiry_dedup_and_map_digest(system):
    agent, shared = system['agent'], system['shared']
    command = dict(command_id='direct_command_001', kind='goal', ttl=.01,
                   payload=dict(payload('direct_command_001'), map_version=17),
                   map_digest=hashlib.sha256(shared.grids['map'].png).hexdigest())
    agent.execute(command, .02)
    assert agent.results[command['command_id']]['http_status'] == 408
    assert not system['calls']
    command.update(command_id='direct_command_002', payload=dict(payload('direct_command_002'), map_version=17),
                   ttl=1, map_digest='wrong')
    agent.execute(command, 0)
    assert agent.results[command['command_id']]['http_status'] == 409
    assert not system['calls']
    command.update(command_id='direct_command_003', payload=dict(payload('direct_command_003'), map_version=17),
                   map_digest=hashlib.sha256(shared.grids['map'].png).hexdigest())
    agent.execute(command, 0)
    agent.execute(command, 0)
    assert len(system['calls']) == 1


def test_stopping_agent_does_not_forward_a_new_command(system):
    agent = system['agent']
    agent.stop.set()
    agent.execute(dict(command_id='stopped_command_01', kind='cancel', ttl=1,
                       payload=dict(request_id='stopped_command_01')), 0)
    assert agent.results['stopped_command_01']['http_status'] == 503
    assert not system['calls']


def test_changed_map_with_same_source_version_gets_new_public_version(system):
    agent, shared = system['agent'], system['shared']
    agent.exchange()
    grid = shared.grids['map']
    grid.update(0, 1, 1, 1, [100])
    grid.version = 17  # Simulate a local web restart reusing its version counter.
    time.sleep(.09)
    agent.exchange()
    assert system['registry'].snapshot('amr-01')['layers']['map']['version'] == 2
    assert system['request']('/api/goal', payload())[0] == 409


def test_duplicate_robot_sessions_and_out_of_order_exchange(system):
    system['agent'].exchange()
    other = Agent(system['url'], TOKEN, local_url=system['agent'].local_url)
    with pytest.raises(ConnectionError):
        other.exchange()
    state = system['shared'].snapshot()
    old = dict(protocol=PROTOCOL_VERSION, robot_id='amr-01', session_id=system['agent'].session_id,
               sequence=1, state=state, state_age=0, layers={}, results=[])
    with pytest.raises(ApiError) as error:
        system['registry'].exchange(old)
    assert error.value.status == 409
    time.sleep(.42)
    other.exchange()
    # The source map revision repeats, but the host revision must remain distinct across sessions.
    assert system['registry'].snapshot('amr-01')['layers']['map']['version'] == 2


def test_second_robot_routes_remain_isolated(system):
    agent = Agent(system['url'], TOKEN, 'amr-02', system['agent'].local_url)
    agent.exchange()
    assert system['request']('/robots/amr-02/api/state')[0] == 200
    assert system['request']('/robots/amr-01/api/state')[0] == 503
    js = system['request']('/robots/amr-02/app.js')[2]
    assert b'/robots/amr-02/api/state' in js and b'/robots/amr-01/api/' not in js


@pytest.mark.parametrize('url', ['http://example.test:8080', 'http://127.0.0.1/path',
                                'http://user:password@127.0.0.1', 'ftp://127.0.0.1'])
def test_local_adapter_cannot_be_redirected_to_an_external_service(url):
    with pytest.raises(ValueError):
        base_url(url, local=True)


def test_assets_are_resolved_without_ros():
    assert find_assets() == ASSETS


def test_one_failed_image_request_does_not_mark_local_state_unavailable(system):
    agent = system['agent']
    original = agent.http.request
    def fail_image(url, *args, **kwargs):
        if '/api/layers/' in url:
            raise TimeoutError('Synthetic image delay')
        return original(url, *args, **kwargs)
    agent.http.request = fail_image
    assert agent.exchange() is True
    assert system['registry'].snapshot('amr-01')['layers'] == {}
    assert agent.exchange_metrics['host_http_status'] == 200


def test_image_work_refreshes_state_and_uploads_one_layer_per_cycle(system):
    agent = system['agent']
    original = agent.http.request
    fetched = []
    def slow_image(url, *args, **kwargs):
        if '/api/layers/' in url:
            fetched.append(url)
            time.sleep(.12)
        return original(url, *args, **kwargs)
    agent.http.request = slow_image
    assert agent.exchange() is True
    assert len(fetched) == 1
    assert agent.exchange_metrics['image_ms'] >= 120
    assert agent.exchange_metrics['state_age'] < .06
    assert system['registry'].snapshot('amr-01')['robot']['age'] >= .12


def test_amr_publisher_expiry_is_not_hidden_by_live_relay(system):
    shared = system['shared']
    shared.telemetry.receive_vehicle(dict(
        vehicle=dict(age=0.,valid=True,fresh=True,session='stopped',epoch=0),
        wheel=dict(age=0.,valid=True,fresh=True,hz=100.,linear=.3)),
        dict(value=0.,epoch=0,valid=True),False,now=time.monotonic()-1.)
    system['agent'].exchange()
    state=system['registry'].snapshot('amr-01')
    assert state['remote']['age'] < .35
    assert not state['telemetry']['vehicle']['fresh']
    assert not state['telemetry']['wheel']['fresh']
    assert state['telemetry']['wheel']['source_age'] >= 1.
    assert state['telemetry']['wheel']['hz'] is None
