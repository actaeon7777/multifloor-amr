"""HTTP -> real goal manager -> fake Nav2, in an isolated domain without vehicle IO."""
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

if os.environ.get('AMR_RUN_WEB_UI_TESTS') != '1':
    pytest.skip('Opt in with AMR_RUN_WEB_UI_TESTS=1, isolated ROS domain 193', allow_module_level=True)

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, JointState, LaserScan
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from amr_stack.navigation_manager import NavigationManager
from amr_vehicle_interfaces.msg import VehicleStatus
from amr_vehicle_status.node import VehicleStatusNode
from amr_web_ui.ros_bridge import WebBridge
from amr_web_ui.server import SharedState, create_http_server


class Scene(Node):
    def __init__(self):
        super().__init__('web_ui_test_scene')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.mode_pub = self.create_publisher(String, '/control_mux/mode', qos)
        self.localization_pub = self.create_publisher(String, '/localization/status', qos)
        self.velocity_pub = self.create_publisher(Odometry, '/odometry/filtered', 5)
        self.generation = 1
        self.verifying_until = 0
        self.relocalize_count = 0
        self.create_service(Trigger, '/localization/relocalize', self.relocalize)
        self.tf_pub = TransformBroadcaster(self)
        self.initialposes = []
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose, 5)
        self.runs = []
        self.action = ActionServer(self, NavigateToPose, '/navigation/navigate_to_pose', self.execute,
                                   handle_accepted_callback=self.accept,
                                   cancel_callback=lambda _h: CancelResponse.ACCEPT,
                                   callback_group=ReentrantCallbackGroup())
        self.create_timer(.03, self.tick)
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'map'
        self.map.info.width = self.map.info.height = 5
        self.map.info.resolution = 1.
        self.map.info.origin.orientation.w = 1.
        self.map.data = [0]*25
        self.map.data[24] = 100
        self.map_pub.publish(self.map)
        self.mode_pub.publish(String(data='MANUAL'))

    def initialpose(self, msg):
        self.initialposes.append(msg)
        self.generation += 1
        self.verifying_until = time.monotonic()+.3

    def relocalize(self, _request, response):
        self.relocalize_count += 1
        self.generation += 1
        self.verifying_until = time.monotonic()+.3
        response.success = True
        response.message = '위치 찾기를 시작했습니다.'
        return response

    def accept(self, handle):
        self.runs.append(dict(handle=handle, future=Future(), done=False))
        handle.execute()

    async def execute(self, handle):
        run = next(r for r in self.runs if r['handle'] is handle)
        return await run['future']

    def tick(self):
        ready = time.monotonic() >= self.verifying_until
        self.localization_pub.publish(String(data=json.dumps(dict(
            state='READY' if ready else 'VERIFYING', ready=ready, detail='시험 위치 상태', source='auto',
            stamp=time.time(), session='web_ui_test_scene', generation=self.generation))))
        velocity = Odometry()
        velocity.header.stamp = self.get_clock().now().to_msg()
        self.velocity_pub.publish(velocity)
        tf = TransformStamped()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_footprint'
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.transform.translation.x = tf.transform.translation.y = .5
        tf.transform.rotation.w = 1.
        self.tf_pub.sendTransform(tf)
        for run in self.runs:
            if run['handle'].is_cancel_requested and not run['done']:
                run['handle'].canceled()
                run['future'].set_result(NavigateToPose.Result())
                run['done'] = True

    def succeed(self):
        run = self.runs[-1]
        run['handle'].succeed()
        run['future'].set_result(NavigateToPose.Result())
        run['done'] = True


def test_http_touch_goal_manual_takeover_result_cancel_and_initialpose():
    assert os.environ.get('ROS_DOMAIN_ID') == '193'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    shared = SharedState()
    bridge = WebBridge(shared)
    manager = NavigationManager()
    scene = Scene()
    executor = SingleThreadedExecutor()
    for node in (bridge, manager, scene):
        executor.add_node(node)
    server = create_http_server(shared, Path(__file__).resolve().parents[1]/'web', port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    root = f'http://127.0.0.1:{server.server_port}'

    def until(predicate, timeout=8):
        end = time.monotonic()+timeout
        while not predicate():
            assert time.monotonic() < end, shared.snapshot()
            executor.spin_once(timeout_sec=.01)

    def post(endpoint, request_id, **values):
        payload = json.dumps(dict(request_id=request_id, **values)).encode()
        req = Request(root+'/api/'+endpoint, data=payload,
                      headers={'Content-Type':'application/json', 'X-AMR-Request':'1'})
        return json.load(urlopen(req, timeout=2))

    try:
        until(lambda: shared.state['action_ready'] and shared.state['robot'] and 'map' in shared.grids
              and shared.snapshot().get('localization', {}).get('ready') and shared.state['velocity'])
        pose = dict(x=2., y=3., yaw=math.pi/2, map_version=shared.grids['map'].version)
        # Scene starts in MANUAL (LB takeover): goals are refused before any ROS call.
        until(lambda: shared.state['mode'] == 'MANUAL' and shared.state['navigation'] == 'MANUAL')
        with pytest.raises(HTTPError) as refused:
            post('goal', 'ros_manual_goal', **pose)
        assert refused.value.code == 409 and '수동 조작' in refused.value.read().decode()
        scene.mode_pub.publish(String(data='AUTO'))
        until(lambda: shared.state['mode'] == 'AUTO' and shared.state['navigation'] == 'IDLE')
        post('goal', 'ros_stale_goal', **pose)
        shared.state['localization']['received_at'] -= 3
        bridge.process_command(shared.commands.get_nowait())
        assert shared.requests['ros_stale_goal']['status'] == 'FAILED'
        assert not scene.runs
        until(lambda: shared.snapshot()['localization']['ready'])
        first = post('goal', 'ros_goal_001', **pose)
        assert first['status'] == 'QUEUED'
        assert post('goal', 'ros_goal_001', **pose) == first
        until(lambda: (shared.state.get('goal') or {}).get('status') == 'ACCEPTED')
        until(lambda: len(scene.runs) == 1)
        received = scene.runs[0]['handle'].request.pose
        assert received.header.frame_id == 'map'
        assert (received.pose.position.x, received.pose.position.y) == (2., 3.)
        assert received.pose.orientation.z == pytest.approx(math.sin(math.pi/4))
        assert received.pose.orientation.w == pytest.approx(math.cos(math.pi/4))
        until(lambda: shared.state['navigation'] == 'RUNNING')
        scene.succeed()
        until(lambda: shared.state['goal']['status'] == 'SUCCEEDED')
        post('goal', 'ros_goal_002', **pose)
        until(lambda: len(scene.runs) == 2 and shared.state['goal']['status'] == 'ACCEPTED')
        until(lambda: shared.state['pending_request'] is None)
        post('cancel', 'ros_cancel_001')
        until(lambda: shared.state['goal']['status'] == 'CANCELED')
        until(lambda: shared.state['navigation'] == 'CANCELED' and shared.state['pending_request'] is None)
        # LB takeover mid-goal: aborted with the override detail and never resumed.
        post('goal', 'ros_goal_003', **pose)
        until(lambda: len(scene.runs) == 3 and shared.state['goal']['status'] == 'ACCEPTED')
        until(lambda: shared.state['navigation'] == 'RUNNING' and shared.state['pending_request'] is None)
        scene.mode_pub.publish(String(data='MANUAL'))
        until(lambda: shared.state['goal']['status'] == 'FAILED' and shared.state['navigation'] == 'MANUAL')
        assert '수동 조작' in shared.state['goal']['error_message']
        scene.mode_pub.publish(String(data='AUTO'))
        until(lambda: shared.state['navigation'] == 'OVERRIDDEN')
        executor.spin_once(timeout_sec=.3)
        assert len(scene.runs) == 3
        post('initialpose', 'ros_initial_001', **pose)
        until(lambda: len(scene.initialposes) == 1)
        initial = scene.initialposes[0]
        assert initial.header.frame_id == 'map'
        assert initial.header.stamp.sec > 0
        assert initial.pose.covariance[0] == initial.pose.covariance[7] == .25
        assert initial.pose.covariance[35] == pytest.approx(.06853891909122467)
        assert not shared.snapshot()['localization']['ready']
        until(lambda: shared.snapshot()['localization']['ready'])
        result = post('relocalize', 'ros_relocalize_001', map_version=pose['map_version'])
        assert result['status'] == 'QUEUED'
        assert not shared.snapshot()['localization']['ready']
        assert post('relocalize', 'ros_relocalize_001', map_version=pose['map_version']) == result
        until(lambda: shared.state['notice']['request_id'] == 'ros_relocalize_001'
              and shared.state['notice']['status'] == 'STARTED')
        assert scene.relocalize_count == 1
        until(lambda: shared.snapshot()['localization']['ready'])
        assert not shared.state['preview']
    finally:
        server.shutdown(); server.server_close(); thread.join()
        executor.shutdown()
        scene.action.destroy()
        manager.server.destroy()
        for node in (bridge, manager, scene):
            node.destroy_node()
        rclpy.shutdown()


def test_set_floor_service_success_rejection_unavailable_and_executor_readiness():
    from types import SimpleNamespace
    from amr_elevator_interfaces.srv import SetCurrentFloor

    assert os.environ.get('ROS_DOMAIN_ID') == '193'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    shared = SharedState()
    bridge = WebBridge(shared)
    shared.building = SimpleNamespace(floors={'3F': None, '4F': None})
    source = Node('web_ui_set_floor_test')
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(source)
    server = create_http_server(shared, '/nonexistent', port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    calls = []

    def stationary():
        shared.state['velocity'] = dict(linear=0., angular=0., received_at=time.monotonic())

    def post(request_id, floor='3F'):
        stationary()
        req = Request(f'http://127.0.0.1:{server.server_port}/api/set_floor',
                      data=json.dumps(dict(request_id=request_id, floor=floor)).encode(),
                      headers={'Content-Type': 'application/json', 'X-AMR-Request': '1'})
        with urlopen(req, timeout=2) as response:
            assert response.status == 202
            return json.load(response)

    def until(predicate):
        deadline = time.monotonic()+8
        while not predicate():
            assert time.monotonic() < deadline, shared.requests
            stationary()
            executor.spin_once(timeout_sec=.01)

    def set_floor(request, response):
        calls.append(request.floor)
        response.success = request.floor == '3F'
        response.message = 'confirmed' if response.success else 'operator override refused'
        return response

    try:
        assert post('floor_unavailable')['status'] == 'QUEUED'
        until(lambda: shared.requests['floor_unavailable']['status'] == 'FAILED')
        assert shared.state['notice']['detail'] == '층 관리 노드에 연결되지 않았습니다.'
        source.create_service(SetCurrentFloor, 'floor/set_current', set_floor)
        until(lambda: bridge.set_floor_client.service_is_ready())
        assert post('floor_success_01')['status'] == 'QUEUED'
        until(lambda: shared.requests['floor_success_01']['status'] == 'SUCCEEDED')
        assert post('floor_success_01')['status'] == 'SUCCEEDED'
        assert shared.state['pending_request'] is None and calls == ['3F']
        post('floor_rejected_01', '4F')
        until(lambda: shared.requests['floor_rejected_01']['status'] == 'FAILED')
        assert shared.state['notice']['detail'] == 'operator override refused'
        post('floor_changed_01')
        shared.state['navigation'] = 'RUNNING'
        until(lambda: shared.requests['floor_changed_01']['status'] == 'FAILED')
        assert calls == ['3F', '4F']  # Admission was repeated on the executor.
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
        executor.shutdown()
        bridge.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


def test_passive_vehicle_subscriptions_and_restart():
    rclpy.init()
    shared = SharedState()
    bridge = WebBridge(shared)
    source = Node('display_telemetry_test_scene')
    vehicle = VehicleStatusNode()
    status_messages = []
    source.create_subscription(VehicleStatus,'/vehicle/status',status_messages.append,1)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(source)
    executor.add_node(vehicle)
    messages = []
    for name, typ, topic in [('wheel', Odometry, '/wheel/odometry'),
                             ('joints', JointState, '/joint_states'),
                             ('imu', Imu, '/imu/data_raw'),
                             ('scan', LaserScan, '/scan'),
                             ('diagnostics', DiagnosticArray, '/diagnostics'),
                             ('command', Twist, '/cmd_vel')]:
        message = typ()
        if name == 'wheel':
            message.twist.twist.linear.x = .32
        elif name == 'joints':
            message.name = ['right_drive_wheel_joint','left_drive_wheel_joint']
            message.velocity = [math.pi,2*math.pi]
        elif name == 'diagnostics':
            status = DiagnosticStatus(name='amr/can_telemetry', message='test counters')
            status.values = [KeyValue(key=k,value=v) for k,v in
                             [('received','10'),('invalid','7'),('stale_frames','0'),
                              ('wheel_rebases','0'),('clock_resets','0')]]
            message.status = [status]
        elif name == 'command':
            message.linear.x = .3
        messages.append((source.create_publisher(typ,topic,10),message))

    def publish():
        for publisher,message in messages:
            if hasattr(message,'header'):
                message.header.stamp = source.get_clock().now().to_msg()
            publisher.publish(message)

    timer = source.create_timer(.05,publish)
    try:
        end = time.monotonic()+8
        while 'diagnostics' not in shared.snapshot()['telemetry'] or not status_messages:
            assert time.monotonic() < end
            executor.spin_once(timeout_sec=.02)
        telemetry = shared.snapshot()['telemetry']
        assert telemetry['joints']['left_rpm'] == pytest.approx(60.)
        assert telemetry['joints']['right_rpm'] == pytest.approx(30.)
        assert telemetry['wheel']['linear'] == .32
        assert telemetry['command']['linear'] == .3
        assert telemetry['diagnostics']['counters']['invalid'] == '7'
        assert telemetry['imu']['age'] < .2
        assert status_messages[-1].left_motor.node_id == 1
        assert status_messages[-1].right_motor.node_id == 0
        assert status_messages[-1].left_motor.encoder.fresh
        assert status_messages[-1].left_motor.wheel_rpm == pytest.approx(60.)
        assert not status_messages[-1].left_motor.heartbeat.received
        assert not status_messages[-1].battery_soc_available
        assert not status_messages[-1].time_healthy
        assert shared.commands.empty()
        assert not any(p.node_name == bridge.get_name()
                       for p in bridge.get_publishers_info_by_topic('/cmd_vel'))
        assert not any(p.node_name == vehicle.get_name()
                       for p in vehicle.get_publishers_info_by_topic('/cmd_vel'))
        assert not any(s.node_name == bridge.get_name()
                       for s in bridge.get_subscriptions_info_by_topic('/joint_states'))
        source.destroy_timer(timer)
        deadline = time.monotonic()+.9
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
        assert shared.snapshot()['telemetry']['wheel']['age'] >= .8
        assert not status_messages[-1].wheel.fresh
        assert status_messages[-1].wheel.valid
        shared.reset_connection('test restart')
        assert shared.snapshot()['telemetry'] == {'journey':None}
    finally:
        executor.shutdown()
        bridge.destroy_node()
        vehicle.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


def test_vehicle_display_rejects_expired_reordered_and_retired_messages():
    rclpy.init()
    shared = SharedState()
    bridge = WebBridge(shared)
    msg = VehicleStatus()
    msg.publisher_session,msg.sequence = 'first',2
    msg.joints.received = msg.joints.valid = msg.joints.fresh = True
    msg.joints.timeout_sec = .75
    msg.left_motor.wheel_rpm,msg.right_motor.wheel_rpm = 60.,30.
    msg.left_motor.power.received = msg.left_motor.power.valid = msg.left_motor.power.fresh = True
    msg.left_motor.power.timeout_sec = 2.
    msg.left_motor.input_voltage_v,msg.left_motor.bus_current_a = 26.12,.82
    try:
        msg.published_wall_stamp = bridge.get_clock().now().to_msg()
        bridge.on_vehicle_status(msg)
        assert shared.snapshot()['telemetry']['joints']['left_rpm']==60.
        assert shared.snapshot()['telemetry']['motor_left_power']['voltage']==26.12
        assert shared.snapshot()['telemetry']['motor_left_power']['timeout']==2.
        msg.sequence,msg.left_motor.wheel_rpm = 1,20.
        bridge.on_vehicle_status(msg)
        assert shared.snapshot()['telemetry']['joints']['left_rpm']==60.
        msg.sequence = 3
        msg.published_wall_stamp.sec -= 2
        bridge.on_vehicle_status(msg)
        assert shared.snapshot()['telemetry']['joints']['left_rpm']==60.
        msg.published_wall_stamp = bridge.get_clock().now().to_msg()
        msg.publisher_session,msg.sequence = 'second',1
        bridge.on_vehicle_status(msg)
        assert shared.snapshot()['telemetry']['joints']['left_rpm']==20.
        msg.publisher_session,msg.sequence,msg.left_motor.wheel_rpm = 'first',4,90.
        bridge.on_vehicle_status(msg)
        assert shared.snapshot()['telemetry']['joints']['left_rpm']==20.
        assert shared.commands.empty()
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


class FakeEstopBridge(Node):
    """Stands in for cmd_vel_to_can's /estop/status."""
    def __init__(self):
        super().__init__('web_ui_estop_scene')
        self.state = 'ESTOP'
        self.sequence = 0
        self.publisher = self.create_publisher(
            String, '/estop/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = self.create_timer(.05, self.publish)

    def publish(self):
        self.sequence += 1
        self.publisher.publish(String(data=json.dumps(dict(
            session='fake_estop', sequence=self.sequence, stamp=time.time(), epoch=1, state=self.state,
            healthy=self.state == 'READY', latched=self.state in ('UNKNOWN', 'ESTOP'),
            detail='시험 E-stop', reason='stm32_stopped', rx_error='',
            stm32=dict(raw=self.state == 'ESTOP', stopped=self.state == 'ESTOP',
                       drive_enabled=self.state == 'READY', trip_count=1, release_count=1)))))


def test_estop_status_follows_the_bridge_without_approval():
    assert os.environ.get('ROS_DOMAIN_ID') == '193'
    rclpy.init()
    from rclpy.parameter import Parameter
    shared = SharedState()
    bridge = WebBridge(shared, parameter_overrides=[Parameter('require_estop', value=True)])
    fake = FakeEstopBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(fake)
    server = create_http_server(shared, Path(__file__).resolve().parents[1]/'web', port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    root = f'http://127.0.0.1:{server.server_port}'

    def until(predicate, timeout=8):
        end = time.monotonic()+timeout
        while not predicate():
            assert time.monotonic() < end, shared.snapshot()['estop']
            executor.spin_once(timeout_sec=.01)

    try:
        assert shared.require_estop
        until(lambda: shared.snapshot()['estop']['state'] == 'ESTOP')
        state = json.load(urlopen(root+'/api/state', timeout=2))['estop']
        assert 'E-stop이 눌려' in state['error'] and 'approval_required' not in state
        fake.state = 'ARMING'  # Released: the bridge re-arms by itself.
        until(lambda: shared.snapshot()['estop']['state'] == 'ARMING')
        assert '주행 준비' in shared.estop_error()
        fake.state = 'READY'
        until(lambda: shared.snapshot()['estop']['healthy'])
        assert shared.estop_error() is None
        fake.destroy_timer(fake.timer)
        deadline = time.monotonic()+.8
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert 'CAN 브릿지' in shared.estop_error()  # Lost status fails closed.
        assert shared.commands.empty()
    finally:
        server.shutdown(); server.server_close(); thread.join()
        executor.shutdown()
        for node in (bridge, fake):
            node.destroy_node()
        rclpy.shutdown()


def test_operator_stop_sending_resume_and_return_home_over_http():
    from types import SimpleNamespace
    assert os.environ.get('ROS_DOMAIN_ID') == '193'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    shared = SharedState()
    bridge, manager, scene = WebBridge(shared), NavigationManager(), Scene()
    executor = SingleThreadedExecutor()
    for node in (bridge, manager, scene):
        executor.add_node(node)
    server = create_http_server(shared, Path(__file__).resolve().parents[1]/'web', port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    def until(predicate):
        end = time.monotonic()+8
        while not predicate():
            assert time.monotonic() < end, shared.snapshot()
            executor.spin_once(timeout_sec=.01)

    def post(kind, request_id, **values):
        req = Request(f'http://127.0.0.1:{server.server_port}/api/{kind}',
            data=json.dumps(dict(request_id=request_id, **values)).encode(),
            headers={'Content-Type': 'application/json', 'X-AMR-Request': '1'})
        with urlopen(req, timeout=2) as response:
            assert response.status == 202
            return json.load(response)

    try:
        scene.mode_pub.publish(String(data='AUTO'))
        until(lambda: shared.readiness_error('goal') is None and 'map' in shared.grids)
        # A late reader must receive the startup false sample without a new change.
        holds = []
        scene.create_subscription(String, '/operator/hold', lambda m: holds.append(json.loads(m.data)),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        until(lambda: holds)
        assert holds[-1]['held'] is False
        floor = SimpleNamespace(real_enabled=True, rooms={'401': SimpleNamespace(goal=(2., 3., 0.))})
        shared.building = SimpleNamespace(home=SimpleNamespace(floor='4F', room='401'),
            floors={'4F': floor}, floor=lambda _name: floor)
        shared.state['floor'] = dict(floor='4F', zone=1, confidence='CONFIRMED')
        pose = dict(x=2., y=3., yaw=0., map_version=shared.grids['map'].version)
        post('goal', 'hold_sending_goal', **pose)
        bridge.process_command(shared.commands.get_nowait())
        assert shared.state['goal']['status'] == 'SENDING'
        stopped = post('stop', 'operator_stop_001')
        assert stopped['canceled'] == 'hold_sending_goal'
        until(lambda: holds[-1]['held'] and shared.state['goal']['status'] in ('FAILED', 'CANCELED', 'REJECTED'))
        until(lambda: manager.run is None and manager.last_status == 'HELD')
        with pytest.raises(HTTPError) as error:
            post('goal', 'hold_refused_goal', **pose)
        assert error.value.code == 409
        assert post('stop', 'operator_stop_001') == stopped
        post('resume', 'operator_resume_001')
        until(lambda: not holds[-1]['held'] and not manager.held and shared.state['pending_request'] is None)
        count = len(scene.runs)
        assert manager.desired is None
        post('goal', 'before_home_goal', **pose)
        until(lambda: len(scene.runs) > count and shared.state['goal']['status'] == 'ACCEPTED')
        reply = post('return_home', 'operator_home_001')
        assert reply['home'] == dict(floor='4F', room='401')
        until(lambda: shared.state['goal']['request_id'] == 'operator_home_001'
              and shared.state['goal']['status'] == 'ACCEPTED')
        assert shared.state['goal']['origin'] == 'return_home'
        assert shared.state['goal']['kind'] == 'goal'
        assert shared.snapshot()['task'] == '복귀 중 (401호)'
        until(lambda: manager.permitted())
        scene.succeed()
        until(lambda: shared.state['goal']['status'] == 'SUCCEEDED')
        assert shared.snapshot()['task'] == '대기'
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
        executor.shutdown()
        scene.action.destroy()
        manager.server.destroy()
        for node in (bridge, manager, scene):
            node.destroy_node()
        rclpy.shutdown()


def test_patrol_topics_http_ack_and_kiosk_lock():
    """Opt-in topic contract using a status publisher and command recorder only."""
    assert os.environ.get('ROS_DOMAIN_ID') == '193'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    shared = SharedState()
    source = Node('web_ui_patrol_stub')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status_pub = source.create_publisher(String, '/patrol/status', qos)
    status = dict(active=False, phase='IDLE', route=None, routes=['4F_full'], index=0, total=12)
    status_pub.publish(String(data=json.dumps(status)))  # retained for the late bridge subscriber
    bridge = WebBridge(shared)
    manager = NavigationManager()
    scene = Scene()
    executor = SingleThreadedExecutor()
    for node in (bridge, manager, scene, source):
        executor.add_node(node)
    commands = []

    def on_command(message):
        command = json.loads(message.data)
        commands.append(command)
        active = command['command'] == 'start'
        status.update(active=active, phase='NAV' if active else 'ABORTED', route='4F_full',
                      index=1, last_command=dict(request_id=command['request_id'], accepted=True, detail='ack'))
        status_pub.publish(String(data=json.dumps(status)))

    source.create_subscription(String, '/patrol/command', on_command, 10)
    server = create_http_server(shared, '/nonexistent', port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    def until(predicate):
        deadline = time.monotonic()+8
        while not predicate():
            assert time.monotonic() < deadline, shared.snapshot()
            executor.spin_once(timeout_sec=.01)

    def post(kind, request_id, **values):
        req = Request(f'http://127.0.0.1:{server.server_port}/api/{kind}',
                      data=json.dumps(dict(request_id=request_id, **values)).encode(),
                      headers={'Content-Type': 'application/json', 'X-AMR-Request': '1'})
        with urlopen(req, timeout=2) as response:
            assert response.status == 202

    try:
        scene.mode_pub.publish(String(data='AUTO'))
        until(lambda: shared.state['patrol']['routes'] == ['4F_full']
              and shared.readiness_error('goal') is None)
        until(lambda: bridge.patrol_pub.get_subscription_count() > 0)
        post('patrol/start', 'ros_patrol_start', route='4F_full')
        until(lambda: shared.requests['ros_patrol_start']['status'] == 'SUCCEEDED')
        assert commands == [dict(command='start', route='4F_full', request_id='ros_patrol_start')]
        assert shared.snapshot(kiosk=True)['patrol']['active']
        assert '야간 순찰 중' in shared.readiness_error('goal')
        post('patrol/stop', 'ros_patrol_stop')
        until(lambda: shared.requests['ros_patrol_stop']['status'] == 'SUCCEEDED')
        assert commands[-1] == dict(command='stop', request_id='ros_patrol_stop')
        assert not shared.patrol_active() and shared.state['pending_request'] is None
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
        executor.shutdown()
        scene.action.destroy()
        manager.server.destroy()
        for node in (bridge, manager, scene, source):
            node.destroy_node()
        rclpy.shutdown()
