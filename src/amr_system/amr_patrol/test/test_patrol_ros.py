"""Opt-in, one executor, synthetic inputs and fake navigation; no hardware."""
import json
import os
from pathlib import Path
import time

import pytest

if os.environ.get('AMR_RUN_PATROL_TESTS') != '1':
    pytest.skip('Opt in with AMR_RUN_PATROL_TESTS=1, isolated domain 198', allow_module_level=True)
assert os.environ.get('ROS_DOMAIN_ID') == '198'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'

import rclpy
from amr_elevator_interfaces.msg import FloorState
from amr_vehicle_interfaces.msg import VehicleStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from std_msgs.msg import String
import yaml

from amr_patrol.patrol_node import PatrolManager
from amr_stack.floor_config import load_floor_config


class Backend(Node):
    def __init__(self):
        super().__init__('fake_patrol_navigation')
        self.runs = []
        self.reject = False
        self.attempts = []
        self.server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose', self.execute,
            goal_callback=self.goal,
            handle_accepted_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def goal(self, _):
        self.attempts.append(time.monotonic())
        return GoalResponse.REJECT if self.reject else GoalResponse.ACCEPT

    def accept(self, handle):
        self.runs.append(dict(handle=handle, done=Future()))
        handle.execute()

    async def execute(self, handle):
        return await next(run['done'] for run in self.runs if run['handle'] is handle)

    def finish(self, index, success=True):
        run = self.runs[index]
        if run['done'].done():
            return
        if run['handle'].is_cancel_requested:
            run['handle'].canceled()
        elif success:
            run['handle'].succeed()
        else:
            run['handle'].abort()
        run['done'].set_result(NavigateToPose.Result())


class Harness:
    def __init__(self, path, health_enabled=True, **settings):
        rclpy.init()
        building = load_floor_config(Path(__file__).resolve().parents[2] / 'amr_stack/config/floors.yaml')
        params = dict(config=str(path), tick_period=.02, **settings)
        self.manager = PatrolManager(building=building, parameter_overrides=[
            Parameter(key, value=value) for key, value in params.items()])
        self.backend = Backend()
        self.probe = Node('patrol_test_probe')
        self.executor = SingleThreadedExecutor()
        for node in (self.manager, self.backend, self.probe):
            self.executor.add_node(node)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.floor_pub = self.probe.create_publisher(FloorState, '/floor/state', retained)
        self.hold_pub = self.probe.create_publisher(String, '/operator/hold', retained)
        self.estop_pub = self.probe.create_publisher(String, '/estop/status', retained)
        self.vehicle_pub = self.probe.create_publisher(VehicleStatus, '/vehicle/status', 10)
        self.nav_pub = self.probe.create_publisher(String, '/navigation/status', retained)
        self.command_pub = self.probe.create_publisher(String, '/patrol/command', 10)
        self.statuses, self.observations, self.events = [], [], []
        self.probe.create_subscription(String, '/patrol/status',
                                       lambda m: self.statuses.append(json.loads(m.data)), retained)
        self.probe.create_subscription(String, '/patrol/observation',
                                       lambda m: self.observations.append(json.loads(m.data)), 10)
        self.probe.create_subscription(String, '/robot/event',
                                       lambda m: self.events.append(json.loads(m.data)), 10)
        self.held, self.healthy, self.health_enabled = False, True, health_enabled
        self.floor, self.confidence, self.zone = '4F', 'CONFIRMED', FloorState.ZONE_OUTSIDE
        self.battery, self.nav_state = 28., 'IDLE'
        self.sequence, self.command_sequence = 0, 0
        self.timer = self.probe.create_timer(.05, self.publish_inputs)
        self.until(lambda: self.manager.client.server_is_ready()
                   and self.command_pub.get_subscription_count() > 0
                   and self.manager.floor_state is not None
                   and self.manager.estop_healthy() and self.manager.battery_v == 28.
                   and bool(self.statuses))

    def publish_inputs(self):
        self.floor_pub.publish(FloorState(floor=self.floor, confidence=self.confidence, zone=self.zone))
        self.hold_pub.publish(String(data=json.dumps(dict(held=self.held))))
        self.sequence += 1
        if self.health_enabled:
            self.estop_pub.publish(String(data=json.dumps(dict(
                session='patrol_test', sequence=self.sequence, epoch=0, stamp=time.time(),
                healthy=self.healthy, latched=not self.healthy,
                state='READY' if self.healthy else 'ESTOP'))))
        self.vehicle_pub.publish(VehicleStatus(battery_voltage_available=True,
                                               battery_voltage_v=self.battery))
        self.nav_pub.publish(String(data=self.nav_state))

    def until(self, predicate, timeout=6.):
        end = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < end, (self.statuses[-1:] or [], len(self.backend.runs))
            self.executor.spin_once(timeout_sec=.01)

    def spin(self, seconds=.15):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)

    def command(self, command='start', route='test'):
        self.command_sequence += 1
        request = f'request_{self.command_sequence:08d}'
        self.command_pub.publish(String(data=json.dumps(dict(command=command, route=route, request_id=request))))
        self.until(lambda: self.statuses[-1]['last_command'] is not None
                   and self.statuses[-1]['last_command']['request_id'] == request)
        return self.statuses[-1]['last_command']

    def close(self):
        self.manager.abort('시험 종료')
        self.spin(.1)
        for index in range(len(self.backend.runs)):
            self.backend.finish(index)
        self.spin(.1)
        self.manager.client.destroy()
        self.backend.server.destroy()
        self.executor.shutdown()
        for node in (self.probe, self.backend, self.manager):
            node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def factory(tmp_path):
    path = tmp_path / 'patrol.yaml'
    path.write_text(yaml.safe_dump(dict(schema_version=1, default_dwell_s=0., routes={
        'test': dict(floor='4F', end='home', waypoints=[
            dict(x=1., y=2., yaw=.5, dwell_s=.12, note='관측'),
            dict(x=2., y=2., note='통과'),
        ])})))
    instances = []

    def create(**settings):
        instance = Harness(path, **settings)
        instances.append(instance)
        return instance

    yield create
    for instance in reversed(instances):
        instance.close()


def test_status_observations_return_and_retained_status(factory):
    h = factory()
    assert h.command()['accepted']
    h.until(lambda: len(h.backend.runs) == 1)
    h.backend.finish(0)
    h.until(lambda: len(h.observations) == 2)
    assert [o['event'] for o in h.observations] == ['begin', 'end']
    assert all(o['index'] == 1 and o['yaw'] == .5 for o in h.observations)
    h.until(lambda: len(h.backend.runs) == 2)
    h.backend.finish(1)
    h.until(lambda: len(h.backend.runs) == 3)
    goal = h.backend.runs[2]['handle'].request.pose.pose.position
    assert (goal.x, goal.y) == h.manager.patrol.config.home_pose[:2]
    h.backend.finish(2)
    h.until(lambda: h.statuses[-1]['phase'] == 'DONE')
    phases = [s['phase'] for s in h.statuses]
    assert all(p in phases for p in ('NAV', 'DWELL', 'RETURN', 'DONE'))
    assert any(s['phase'] == 'RETURN' and s['index'] == 3 for s in h.statuses)
    h.until(lambda: any(e['kind'] == 'patrol.finished' for e in h.events))
    assert all(e['id'] and e['floor'] == '4F' and e['image_path'] is None for e in h.events)
    late = []
    h.probe.create_subscription(String, '/patrol/status', lambda m: late.append(json.loads(m.data)),
                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    h.until(lambda: bool(late))
    assert late[-1]['phase'] == 'DONE' and not late[-1]['active']


@pytest.mark.parametrize('trigger,reason', [('hold', '관리자 정지'), ('stop', '순찰 정지 요청'),
                                           ('estop', 'E-stop'), ('stale_estop', 'E-stop')])
def test_abort_cancels_without_return(factory, trigger, reason):
    h = factory()
    assert h.command()['accepted']
    h.until(lambda: len(h.backend.runs) == 1)
    if trigger == 'hold':
        h.held = True
    elif trigger == 'stop':
        assert h.command('stop')['accepted']
    elif trigger == 'estop':
        h.healthy = False
    else:
        h.health_enabled = False
    h.until(lambda: h.statuses[-1]['phase'] == 'ABORTED')
    assert h.statuses[-1]['detail'] == reason
    h.until(lambda: h.backend.runs[0]['handle'].is_cancel_requested)
    h.backend.finish(0)
    h.held, h.healthy, h.health_enabled = False, True, True
    h.spin(.3)
    assert len(h.backend.runs) == 1 and h.statuses[-1]['phase'] == 'ABORTED'


def test_bad_commands_are_acknowledged_without_motion(factory):
    h = factory()
    assert not h.command(route='missing')['accepted']
    assert not h.command(command='unknown')['accepted']
    h.command_pub.publish(String(data='{broken'))
    h.until(lambda: h.statuses[-1]['last_command']['request_id'] is None)
    assert not h.statuses[-1]['last_command']['accepted']
    assert not h.backend.runs
    h.nav_state = 'RUNNING'
    h.until(lambda: h.manager.nav_state == 'RUNNING')
    assert not h.command()['accepted']


def test_hold_before_goal_ack_cancels_late_handle(factory, monkeypatch):
    h = factory()
    original = h.manager.client.send_goal_async
    delayed, actual = Future(), []

    def send(*args, **kwargs):
        actual.append(original(*args, **kwargs))
        return delayed

    monkeypatch.setattr(h.manager.client, 'send_goal_async', send)
    assert h.command()['accepted']
    h.until(lambda: actual and actual[0].done() and len(h.backend.runs) == 1)
    h.held = True
    h.until(lambda: h.statuses[-1]['phase'] == 'ABORTED')
    delayed.set_result(actual[0].result())
    h.until(lambda: h.backend.runs[0]['handle'].is_cancel_requested)
    h.backend.finish(0)
    h.spin()
    assert len(h.backend.runs) == 1


def test_low_battery_waits_for_cancel_before_return(factory):
    h = factory(battery_low_s=.2)
    assert h.command()['accepted']
    h.until(lambda: len(h.backend.runs) == 1)
    h.battery = 25.
    h.until(lambda: h.statuses[-1]['phase'] == 'RETURN')
    h.until(lambda: h.backend.runs[0]['handle'].is_cancel_requested)
    h.spin(.2)
    assert len(h.backend.runs) == 1
    h.backend.finish(0)
    h.until(lambda: len(h.backend.runs) == 2)
    h.backend.finish(1)
    h.until(lambda: h.statuses[-1]['phase'] == 'DONE')
    assert h.statuses[-1]['detail'] == '배터리 부족으로 조기 복귀'


def test_timeout_cancels_then_retries_same_goal(factory):
    h = factory(nav_timeout_s=.2)
    assert h.command()['accepted']
    h.until(lambda: len(h.backend.runs) == 1)
    h.until(lambda: h.backend.runs[0]['handle'].is_cancel_requested)
    h.spin(.25)
    assert len(h.backend.runs) == 1
    h.backend.finish(0)
    h.until(lambda: len(h.backend.runs) == 2)
    assert h.backend.runs[0]['handle'].request.pose.pose == h.backend.runs[1]['handle'].request.pose.pose


def test_active_status_has_heartbeat(factory):
    h = factory()
    assert h.command()['accepted']
    h.until(lambda: len(h.backend.runs) == 1)
    before = len(h.statuses)
    h.until(lambda: len(h.statuses) > before, timeout=2.)
    assert h.statuses[-1]['active'] and h.statuses[-1]['elapsed_s'] > 0


def test_rejected_goals_skip_then_abort(factory):
    h = factory(max_retries=0, max_consecutive_failures=2)
    h.backend.reject = True
    assert h.command()['accepted']
    h.until(lambda: h.statuses[-1]['phase'] == 'ABORTED')
    assert h.statuses[-1]['skipped'] == [1, 2]
    assert not h.backend.runs
    h.until(lambda: len([e for e in h.events if e['kind'] == 'patrol.waypoint_skipped']) == 2)


def test_success_outside_feedback_tolerance_retries(factory):
    h = factory()
    assert h.command()['accepted']
    h.until(lambda: len(h.backend.runs) == 1 and h.manager.run.handle is not None)
    feedback = NavigateToPose.Feedback()
    feedback.current_pose.header.frame_id = 'map'
    feedback.current_pose.pose.position.x = 20.
    h.backend.runs[0]['handle'].publish_feedback(feedback)
    h.until(lambda: h.manager.run.feedback_xy is not None)
    h.backend.finish(0)
    h.until(lambda: len(h.backend.runs) == 2)
    assert h.manager.patrol.index == 1 and h.manager.patrol.retries == 1
    assert not h.observations


@pytest.mark.parametrize('floor,zone', [('3F', FloorState.ZONE_OUTSIDE),
                                       ('4F', FloorState.ZONE_IN_ELEVATOR)])
def test_floor_guard_aborts_and_cancels_mid_patrol(factory, floor, zone):
    h = factory()
    assert h.command()['accepted']
    h.until(lambda: len(h.backend.runs) == 1)
    h.floor, h.zone = floor, zone
    h.until(lambda: h.statuses[-1]['phase'] == 'ABORTED')
    h.until(lambda: h.backend.runs[0]['handle'].is_cancel_requested)
    assert h.statuses[-1]['detail'] == '현재 층을 확인할 수 없어 순찰 중단'
    h.backend.finish(0)
    h.spin()
    assert len(h.backend.runs) == 1


def test_rejections_are_spaced_and_manual_wait_does_not_send(factory):
    h = factory(retry_delay_s=.3)
    h.backend.reject = True
    assert h.command()['accepted']
    h.until(lambda: len(h.backend.attempts) == 2)
    assert h.backend.attempts[1] - h.backend.attempts[0] >= .29
    h.nav_state = 'MANUAL'
    h.until(lambda: h.statuses[-1]['detail'] == '수동 조작 중: 순찰 대기')
    before = (len(h.backend.attempts), h.manager.patrol.retries, h.manager.patrol.failures)
    h.spin(.4)
    assert before == (len(h.backend.attempts), h.manager.patrol.retries, h.manager.patrol.failures)
    h.backend.reject = False
    h.nav_state = 'IDLE'
    h.until(lambda: len(h.backend.runs) == 1)


@pytest.mark.parametrize('state', ['RUNNING', 'WAITING_NAV2', 'MANEUVER'])
def test_other_navigation_after_failure_never_preempts_back(factory, state):
    h = factory(retry_delay_s=.2)
    assert h.command()['accepted']
    h.until(lambda: len(h.backend.runs) == 1)
    h.nav_state = state
    h.until(lambda: h.manager.nav_state == state)
    h.backend.finish(0, success=False)
    h.until(lambda: h.statuses[-1]['phase'] == 'ABORTED')
    assert h.statuses[-1]['detail'] == '다른 주행 명령으로 순찰 중단'
    h.spin(.3)
    assert len(h.backend.attempts) == 1


def test_simulation_can_patrol_without_estop_stream(factory):
    h = factory(require_estop=False, health_enabled=False)
    assert h.manager.estop.status == {}
    assert h.command()['accepted']
    h.until(lambda: len(h.backend.runs) == 1)
    h.spin(.7)
    assert h.manager.patrol.active


@pytest.mark.parametrize('server_missing', [False, True])
def test_inactive_stuck_run_is_released_and_late_result_is_ignored(factory, monkeypatch, server_missing):
    h = factory(cancel_result_timeout_s=.2)
    assert h.command()['accepted']
    h.until(lambda: len(h.backend.runs) == 1)
    assert h.command('stop')['accepted']
    h.until(lambda: h.backend.runs[0]['handle'].is_cancel_requested)
    if server_missing:
        monkeypatch.setattr(h.manager.client, 'server_is_ready', lambda: False)
    h.until(lambda: h.manager.run is None)
    if server_missing:
        monkeypatch.undo()
    assert h.command()['accepted']
    h.until(lambda: len(h.backend.runs) == 2)
    current = h.manager.run
    h.backend.finish(0)
    h.spin(.1)
    assert h.manager.run is current and h.manager.patrol.active


def test_active_cancel_wait_is_not_released_by_inactive_timeout(factory):
    h = factory(nav_timeout_s=.1, retry_delay_s=.1, cancel_result_timeout_s=.1)
    assert h.command()['accepted']
    h.until(lambda: len(h.backend.runs) == 1)
    h.until(lambda: h.backend.runs[0]['handle'].is_cancel_requested)
    h.spin(.25)
    assert h.manager.run is not None and h.manager.patrol.active
    assert len(h.backend.attempts) == 1


@pytest.mark.parametrize('signal_name', ['SIGINT', 'SIGTERM'])
def test_main_keeps_context_alive_through_cancel_drain(monkeypatch, signal_name):
    # Exercise the signal/cleanup order without launching another ROS process.
    import signal
    from amr_patrol import patrol_node as module
    from rclpy.signals import SignalHandlerOptions

    calls, handlers = [], {}
    alive = [False]

    def init(**kwargs):
        assert kwargs['signal_handler_options'] == SignalHandlerOptions.NO
        alive[0] = True

    def shutdown():
        calls.append('context_shutdown')
        alive[0] = False

    def install(signum, handler):
        old = handlers.get(signum)
        handlers[signum] = handler
        return old

    class FakeNode:
        run = object()

        def abort(self, reason):
            assert alive[0] and reason == '순찰 노드 종료'
            assert self.shutting_down
            calls.append('cancel')

        def destroy_node(self):
            calls.append('destroy_node')

    node = FakeNode()

    class FakeExecutor:
        def create_future(self):
            return Future()

        def add_node(self, _):
            pass

        def wake(self):
            pass

        def spin_until_future_complete(self, _, timeout_sec):
            assert timeout_sec <= .1
            handlers[getattr(signal, signal_name)]()

        def spin_once(self, timeout_sec):
            assert alive[0] and timeout_sec <= 1.
            calls.append('cancel_response')
            node.run = None

        def shutdown(self):
            calls.append('executor_shutdown')

    monkeypatch.setattr(module.rclpy, 'init', init)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: alive[0])
    monkeypatch.setattr(module.rclpy, 'shutdown', shutdown)
    monkeypatch.setattr(module.signal, 'signal', install)
    monkeypatch.setattr(module, 'PatrolManager', lambda: node)
    monkeypatch.setattr('rclpy.executors.SingleThreadedExecutor', FakeExecutor)
    try:
        import rclpy.experimental
    except ImportError:
        pass
    else:
        monkeypatch.setattr(rclpy.experimental, 'EventsExecutor', FakeExecutor, raising=False)
    module.main()
    assert calls == ['cancel', 'cancel_response', 'executor_shutdown', 'destroy_node', 'context_shutdown']
    assert all(handler is None for handler in handlers.values())
