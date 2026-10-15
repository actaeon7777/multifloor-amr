"""Action races and velocity gating using real ROS, but no hardware or Nav2 motion."""
import json
import os
import time

import pytest

if os.environ.get('AMR_RUN_NAVIGATION_TESTS') != '1':
    pytest.skip('Opt in with AMR_RUN_NAVIGATION_TESTS=1, isolated domain 190', allow_module_level=True)

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from std_msgs.msg import String

from amr_stack.navigation_manager import NavigationManager


class Backend(Node):
    def __init__(self):
        super().__init__('fake_navigation_backend')
        self.runs = []
        self.emit = True
        self.bad_command = False
        self.publisher = self.create_publisher(Twist, '/cmd_vel_navigation', 1)
        self.server = ActionServer(
            self, NavigateToPose, '/navigation/navigate_to_pose', self.execute,
            handle_accepted_callback=self.accept,
            cancel_callback=lambda h: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.timer = self.create_timer(.03, self.publish)

    def accept(self, handle):
        self.runs.append({'handle': handle, 'done': Future()})
        handle.execute()

    async def execute(self, handle):
        run = next(r for r in self.runs if r['handle'] is handle)
        return await run['done']

    def finish(self, index, success=True, result=None):
        run = self.runs[index]
        if run['handle'].is_cancel_requested:
            run['handle'].canceled()
        elif success:
            run['handle'].succeed()
        else:
            run['handle'].abort()
        run['done'].set_result(result if result is not None else NavigateToPose.Result())

    def publish(self):
        if self.emit and any(not r['done'].done() for r in self.runs):
            msg = Twist()
            msg.linear.x = float('nan') if self.bad_command else .3
            self.publisher.publish(msg)


class Harness:
    def __init__(self, ready=True):
        assert os.environ['ROS_DOMAIN_ID'] == '190'
        assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
        rclpy.init()
        self.manager = NavigationManager()
        self.backend = Backend()
        self.probe = Node('navigation_manager_test_probe')
        self.executor = SingleThreadedExecutor()
        for node in (self.manager, self.backend, self.probe):
            self.executor.add_node(node)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode = self.probe.create_publisher(String, '/control_mux/mode', qos)
        self.localization = self.probe.create_publisher(String, '/localization/status', qos)
        self.initial_pose = self.probe.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.status_enabled = ready
        self.status_ready = ready
        self.tracking_warning = ''
        self.generation = time.time_ns()
        self.status_timer = self.probe.create_timer(.1, self.publish_localization)
        self.topic_goal = self.probe.create_publisher(PoseStamped, '/goal_pose', 1)
        self.outputs = []
        self.probe.create_subscription(Twist, '/cmd_vel_auto',
                                      lambda m: self.outputs.append((time.monotonic(), m.linear.x)), 100)
        self.client = ActionClient(self.probe, NavigateToPose, '/navigate_to_pose')
        self.until(lambda: self.client.server_is_ready() and self.mode.get_subscription_count() > 0)
        if ready:
            self.until(self.manager.localization_ready)

    def publish_localization(self):
        if self.status_enabled:
            self.localization.publish(String(data=json.dumps({
                'state': 'READY' if self.status_ready else 'VERIFYING',
                'ready': self.status_ready, 'generation': self.generation,
                'session': 'navigation_manager_test',
                'detail': 'Synthetic test heartbeat', 'source': 'test',
                'tracking_warning': self.tracking_warning,
                'stamp': time.time()})))

    def set_ready(self, ready, new_generation=False):
        self.status_enabled = True
        self.status_ready = ready
        if new_generation:
            self.generation = max(time.time_ns(), self.generation + 1)
        self.publish_localization()
        self.until(lambda: self.manager.localization_ready() == ready)

    def until(self, predicate, timeout=6):
        end = time.monotonic()+timeout
        while not predicate():
            assert time.monotonic() < end, (self.manager.last_status, len(self.backend.runs))
            self.executor.spin_once(timeout_sec=.01)

    def spin(self, seconds=.3):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)

    def wait(self, future):
        self.until(future.done)
        return future.result()

    def set_mode(self, value):
        self.mode.publish(String(data=value))
        self.until(lambda: self.manager.mode == value)

    def goal(self, x=2., frame='map'):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame
        goal.pose.pose.position.x = x
        goal.pose.pose.orientation.w = 1.
        handle = self.wait(self.client.send_goal_async(goal))
        if handle.accepted:
            return handle, handle.get_result_async()
        return handle, None

    def assert_zero(self, seconds=.25):
        # Drain commands published before the transition, then inspect new samples.
        self.spin(.08)
        start = len(self.outputs)
        self.spin(seconds)
        assert len(self.outputs) > start
        assert all(v == 0. for _, v in self.outputs[start:])

    def close(self):
        self.manager.server.destroy()
        self.backend.server.destroy()
        self.executor.shutdown()
        for node in (self.probe, self.backend, self.manager):
            node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def h():
    harness = Harness()
    try:
        yield harness
    finally:
        harness.close()


def test_pause_resume_replacement_cancel_and_terminal_results(h):
    handle, result = h.goal()
    h.spin()
    assert not h.backend.runs
    assert h.manager.last_status == 'WAITING_AUTO'
    h.assert_zero()

    h.set_mode('AUTO')
    h.until(lambda: h.outputs and h.outputs[-1][1] == .3)
    assert len(h.backend.runs) == 1
    # Controller silence and invalid commands may never replay the last speed.
    h.backend.emit = False
    h.spin(.25)
    h.assert_zero()
    h.backend.emit = True
    h.backend.bad_command = True
    h.assert_zero()
    h.backend.bad_command = False
    h.until(lambda: h.outputs[-1][1] == .3)

    h.set_mode('MANUAL')
    h.until(lambda: h.backend.runs[0]['handle'].is_cancel_requested)
    assert not result.done()
    h.assert_zero()  # Backend deliberately keeps producing old commands.
    h.set_mode('AUTO')
    h.spin(.2)
    assert len(h.backend.runs) == 1  # Wait for terminal result, not just cancel ACK.
    h.assert_zero()
    h.backend.finish(0)
    h.until(lambda: len(h.backend.runs) == 2)
    assert h.backend.runs[1]['handle'].request.pose.pose.position.x == 2.
    h.until(lambda: h.outputs[-1][1] == .3)

    replacement, replacement_result = h.goal(3.)
    assert h.wait(result).status == GoalStatus.STATUS_ABORTED
    h.until(lambda: h.backend.runs[1]['handle'].is_cancel_requested)
    h.assert_zero()
    assert len(h.backend.runs) == 2
    h.backend.finish(1)
    h.until(lambda: len(h.backend.runs) == 3)
    assert h.backend.runs[2]['handle'].request.pose.pose.position.x == 3.
    h.wait(replacement.cancel_goal_async())
    h.until(lambda: h.backend.runs[2]['handle'].is_cancel_requested)
    h.assert_zero()
    h.backend.finish(2)
    assert h.wait(replacement_result).status == GoalStatus.STATUS_CANCELED
    h.set_mode('MANUAL'); h.set_mode('AUTO'); h.spin()
    assert len(h.backend.runs) == 3

    _, success = h.goal(1.)
    h.until(lambda: len(h.backend.runs) == 4)
    h.backend.finish(3)
    assert h.wait(success).status == GoalStatus.STATUS_SUCCEEDED
    h.assert_zero()
    h.set_mode('MANUAL'); h.set_mode('AUTO'); h.spin()
    assert len(h.backend.runs) == 4
    _, failure = h.goal(4.)
    h.until(lambda: len(h.backend.runs) == 5)
    h.backend.finish(4, success=False)
    assert h.wait(failure).status == GoalStatus.STATUS_ABORTED
    h.set_mode('MANUAL'); h.set_mode('AUTO'); h.spin()
    assert len(h.backend.runs) == 5
    h.assert_zero()


def test_paused_goal_replacement_cancel_topic_input_and_invalid_goal(h):
    old, old_result = h.goal(1.)
    new, new_result = h.goal(2.)
    assert h.wait(old_result).status == GoalStatus.STATUS_ABORTED
    h.wait(new.cancel_goal_async())
    assert h.wait(new_result).status == GoalStatus.STATUS_CANCELED
    h.set_mode('AUTO'); h.spin()
    assert not h.backend.runs
    bad, _ = h.goal(frame='odom')
    assert not bad.accepted
    bad, _ = h.goal(x=float('nan'))
    assert not bad.accepted
    h.set_mode('MANUAL')
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = 5.
    pose.pose.orientation.w = 1.
    h.until(lambda: h.topic_goal.get_subscription_count() > 0)
    h.topic_goal.publish(pose)
    h.until(lambda: h.manager.desired is not None)
    h.spin(); assert not h.backend.runs
    h.set_mode('AUTO')
    h.until(lambda: len(h.backend.runs) == 1)
    assert h.backend.runs[0]['handle'].request.pose.pose.position.x == 5.
    h.backend.finish(0)
    h.until(lambda: h.manager.desired is None)


def test_pause_before_backend_goal_acknowledgement(h):
    _, result = h.goal()
    # Exercise the actual mode callback before the internal action response arrives.
    h.manager.on_mode(String(data='AUTO'))
    h.manager.tick()
    assert h.manager.run is not None and h.manager.run.handle is None
    h.manager.on_mode(String(data='MANUAL'))
    h.manager.on_mode(String(data='AUTO'))
    h.until(lambda: h.backend.runs and h.backend.runs[0]['handle'].is_cancel_requested)
    h.assert_zero()
    assert len(h.backend.runs) == 1 and not result.done()
    h.backend.finish(0)
    h.until(lambda: len(h.backend.runs) == 2)
    h.backend.finish(1)
    assert h.wait(result).status == GoalStatus.STATUS_SUCCEEDED


def test_cancel_topic_goal_without_nav2_rviz_panel(h):
    """Standard RViz SetGoal plus the documented cancel-all service."""
    cancel = h.probe.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
    h.until(cancel.service_is_ready)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.orientation.w = 1.0
    for mode in ('MANUAL', 'AUTO'):
        h.set_mode(mode)
        h.until(lambda: h.topic_goal.get_subscription_count() > 0)
        h.topic_goal.publish(pose)
        h.until(lambda: h.manager.desired is not None)
        if mode == 'AUTO':
            h.until(lambda: len(h.backend.runs) == 1)
        response = h.wait(cancel.call_async(CancelGoal.Request()))
        assert response.return_code == CancelGoal.Response.ERROR_NONE
        assert len(response.goals_canceling) == 1
        if mode == 'AUTO':
            h.until(lambda: h.backend.runs[0]['handle'].is_cancel_requested)
            h.backend.finish(0)
        h.until(lambda: h.manager.desired is None and h.manager.run is None)
        h.assert_zero()
    h.set_mode('MANUAL')
    h.set_mode('AUTO')
    h.spin()
    assert len(h.backend.runs) == 1
    h.assert_zero()


def test_missing_status_rejects_public_action_and_topic_goals():
    h = Harness(ready=False)
    try:
        h.set_mode('AUTO')
        handle, _ = h.goal()
        assert not handle.accepted
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.w = 1.0
        h.until(lambda: h.topic_goal.get_subscription_count() > 0)
        h.topic_goal.publish(pose)
        h.spin()
        assert h.manager.desired is None and not h.backend.runs
        h.assert_zero()
        h.set_ready(True)
        _, result = h.goal()
        h.until(lambda: len(h.backend.runs) == 1)
        h.backend.finish(0)
        assert h.wait(result).status == GoalStatus.STATUS_SUCCEEDED
    finally:
        h.close()


def test_readiness_loss_before_backend_ack_cancels_without_resuming(h):
    _, result = h.goal()
    h.manager.on_mode(String(data='AUTO'))
    h.manager.tick()
    assert h.manager.run is not None and h.manager.run.handle is None
    h.set_ready(False, new_generation=True)
    assert h.wait(result).status == GoalStatus.STATUS_ABORTED
    h.until(lambda: h.backend.runs and h.backend.runs[0]['handle'].is_cancel_requested)
    h.assert_zero()
    rejected, _ = h.goal()
    assert not rejected.accepted
    h.set_ready(True)
    h.backend.finish(0)
    h.spin()
    assert len(h.backend.runs) == 1 and h.manager.desired is None
    h.assert_zero()


@pytest.mark.parametrize('mode', ['MANUAL', 'AUTO'])
def test_stale_heartbeat_aborts_goal_and_recovery_never_resumes_it(h, mode):
    h.set_mode(mode)
    _, result = h.goal()
    if mode == 'AUTO':
        h.until(lambda: h.outputs and h.outputs[-1][1] == .3)
    h.status_enabled = False
    h.spin(h.manager.settings['localization_status_timeout'] + .15)
    assert h.wait(result).status == GoalStatus.STATUS_ABORTED
    assert h.manager.desired is None
    h.assert_zero()
    rejected, _ = h.goal()
    assert not rejected.accepted
    if mode == 'AUTO':
        h.until(lambda: h.backend.runs[0]['handle'].is_cancel_requested)
    # A live heartbeat in the same generation may recover the gate, but cannot
    # recover an old user goal. Backend cancellation must still reach terminal.
    h.set_ready(True)
    if mode == 'AUTO':
        h.backend.finish(0)
    h.set_mode('AUTO')
    h.spin()
    assert len(h.backend.runs) == (1 if mode == 'AUTO' else 0)
    h.assert_zero()


@pytest.mark.parametrize('unready_first', [False, True])
def test_initialpose_requires_new_ready_generation_for_either_callback_order(h, unready_first):
    h.set_mode('AUTO')
    _, result = h.goal()
    h.until(lambda: h.outputs and h.outputs[-1][1] == .3)
    previous_generation = h.generation
    if unready_first:
        h.set_ready(False, new_generation=True)
    h.until(lambda: h.initial_pose.get_subscription_count() > 0)
    h.initial_pose.publish(PoseWithCovarianceStamped())
    h.until(lambda: h.manager.initial_pose_generation_barrier == previous_generation)
    assert h.wait(result).status == GoalStatus.STATUS_ABORTED
    h.until(lambda: h.backend.runs[0]['handle'].is_cancel_requested)
    h.assert_zero()
    if not unready_first:
        # Repeated pre-initialpose READY heartbeats may not release the gate.
        h.spin()
        assert not h.manager.localization_ready()
        rejected, _ = h.goal()
        assert not rejected.accepted
        h.set_ready(True, new_generation=True)
    else:
        h.set_ready(True)
    h.backend.finish(0)
    h.spin()
    assert len(h.backend.runs) == 1 and h.manager.desired is None
    h.assert_zero()
    _, new_result = h.goal(3.)
    h.until(lambda: len(h.backend.runs) == 2)
    h.backend.finish(1)
    assert h.wait(new_result).status == GoalStatus.STATUS_SUCCEEDED


def test_new_ready_generation_aborts_goal_even_if_unready_message_was_missed(h):
    h.set_mode('AUTO')
    _, result = h.goal()
    h.until(lambda: h.outputs and h.outputs[-1][1] == .3)
    h.generation = max(time.time_ns(), h.generation + 1)
    h.publish_localization()
    assert h.wait(result).status == GoalStatus.STATUS_ABORTED
    h.until(lambda: h.backend.runs[0]['handle'].is_cancel_requested)
    h.backend.finish(0)
    h.spin()
    assert len(h.backend.runs) == 1 and h.manager.desired is None
    h.assert_zero()


def test_invalid_stale_or_old_generation_status_cannot_enable_navigation(h):
    h.status_enabled = False
    generation = h.generation
    for payload in (
            '{}', '{', '[]',
            json.dumps({'state': 'READY', 'ready': 'true',
                        'generation': generation, 'stamp': time.time()}),
            json.dumps({'state': 'READY', 'ready': True,
                        'generation': generation, 'stamp': time.time() - 10}),
            json.dumps({'state': 'READY', 'ready': True,
                        'generation': generation, 'stamp': time.time() + 10})):
        h.manager.on_localization_status(String(data=payload))
        assert not h.manager.localization_ready()
        rejected, _ = h.goal()
        assert not rejected.accepted
    h.manager.on_localization_status(String(data=json.dumps({
        'state': 'VERIFYING', 'ready': False, 'session': 'navigation_manager_test',
        'generation': generation + 1, 'stamp': time.time()})))
    h.manager.on_localization_status(String(data=json.dumps({
        'state': 'READY', 'ready': True, 'session': 'navigation_manager_test',
        'generation': generation, 'stamp': time.time()})))
    assert not h.manager.localization_ready()
    assert h.manager.desired is None and not h.backend.runs
    h.assert_zero()


def test_new_session_aborts_old_goal_and_accepts_lower_generation(h):
    h.set_mode('AUTO')
    _, result = h.goal()
    h.until(lambda: len(h.backend.runs) == 1)
    h.status_enabled = False
    def status(session, generation):
        h.manager.on_localization_status(String(data=json.dumps(dict(
            state='READY', ready=True, session=session, generation=generation, stamp=time.time()))))
    status('new_controller', 0)
    h.until(result.done)
    assert h.manager.desired is None and h.manager.localization_ready()
    status('navigation_manager_test', h.generation+100)
    assert h.manager.localization_generation == 0
    h.assert_zero()


@pytest.mark.parametrize('miss_latched_message', [False, True])
def test_time_fault_aborts_goal_and_requires_new_localization_generation(h, miss_latched_message):
    h.manager.require_time_health = True
    def health(sequence, good, latched=False, epoch=0):
        h.manager.on_time_health(String(data=json.dumps(dict(
            session='time_guard_test', sequence=sequence, epoch=epoch, stamp=time.time(),
            healthy=good, latched=latched))))
    health(1, True)
    h.set_mode('AUTO')
    _, result = h.goal()
    h.until(lambda: len(h.backend.runs) == 1)
    health(2, miss_latched_message, not miss_latched_message, epoch=1)
    h.until(result.done)
    health(3, True, epoch=1)
    h.publish_localization()
    h.spin(.05)
    assert not h.manager.localization_ready()
    h.set_ready(True, new_generation=True)
    health(4, True, epoch=1)
    h.until(h.manager.localization_ready)
    assert h.manager.desired is None
    h.assert_zero()


@pytest.mark.parametrize('warning', ['스캔 일치율 68%/70%', '위치 표준편차 0.352m/0.350m'])
def test_ready_quality_warning_keeps_the_same_goal_and_velocity(h, warning):
    h.set_mode('AUTO')
    _, result = h.goal()
    h.until(lambda: h.outputs and h.outputs[-1][1] == .3)
    h.tracking_warning = warning
    h.publish_localization()
    h.spin(1.1)
    assert h.manager.localization_ready()
    assert len(h.backend.runs) == 1
    assert not h.backend.runs[0]['handle'].is_cancel_requested
    assert not result.done()
    assert h.outputs[-1][1] == .3
    h.backend.finish(0)
    assert h.wait(result).status == GoalStatus.STATUS_SUCCEEDED


def test_backend_failure_details_reach_public_result(h):
    h.set_mode('AUTO')
    handle, future = h.goal()
    h.until(lambda: len(h.backend.runs) == 1)
    result = NavigateToPose.Result(error_code=101, error_msg='Synthetic planner failure')
    h.backend.finish(0, success=False, result=result)
    response = h.wait(future)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.error_code == 101
    assert response.result.error_msg == 'Synthetic planner failure'


def test_localization_abort_explains_readiness_loss(h):
    h.set_mode('AUTO')
    handle, future = h.goal()
    h.until(lambda: len(h.backend.runs) == 1)
    h.set_ready(False)
    response = h.wait(future)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert '위치 준비 조건 해제' in response.result.error_msg
    assert 'Synthetic test heartbeat' in response.result.error_msg
    h.until(lambda: h.backend.runs[0]['handle'].is_cancel_requested)
    h.backend.finish(0)
