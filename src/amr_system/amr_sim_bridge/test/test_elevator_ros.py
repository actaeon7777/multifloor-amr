"""Opt-in request and arrival-confirm integration, isolated domain 193 only."""
import json
import os
import time
from types import SimpleNamespace

import pytest

if os.environ.get('AMR_RUN_ELEVATOR_TESTS') != '1':
    pytest.skip('Set AMR_RUN_ELEVATOR_TESTS=1 in domain 193', allow_module_level=True)

from amr_elevator_interfaces.msg import ElevatorRequest, FloorArrivalConfirm, FloorState
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from amr_sim_bridge.elevator_node import ElevatorSimNode, ElevatorRelayNode
from amr_stack.floor_config import load_floor_config


@pytest.fixture
def ros():
    assert os.environ['ROS_DOMAIN_ID'] == '193'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rclpy.init()
    executor = SingleThreadedExecutor()
    probe = rclpy.create_node('elevator_test_probe')
    executor.add_node(probe)
    statuses, confirms, raw = [], [], []
    latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    probe.create_subscription(String, '/sim/elevator/status', lambda m: statuses.append(json.loads(m.data)), latched)
    probe.create_subscription(FloorArrivalConfirm, '/floor/arrival_confirm', confirms.append, 10)
    probe.create_subscription(String, '/sim/elevator/request_raw', lambda m: raw.append(json.loads(m.data)), latched)
    request = probe.create_publisher(ElevatorRequest, '/elevator/request', latched)
    state = probe.create_publisher(FloorState, '/floor/state', latched)
    status = probe.create_publisher(String, '/sim/elevator/status', latched)

    def spin(duration, until=None):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
            if until is not None and until():
                return True
        return until is None

    yield SimpleNamespace(executor=executor, probe=probe, statuses=statuses, confirms=confirms,
                          raw=raw, request=request, state=state, status=status, spin=spin)
    nodes = executor.get_nodes()
    executor.shutdown()
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


def candidate(ros, floor, awaiting=True, candidate_id=7):
    message = FloorState(zone=FloorState.ZONE_IN_ELEVATOR, floor=floor,
                         confidence='ASSUMED', awaiting_confirm=awaiting, candidate_id=candidate_id)
    message.header.stamp = ros.probe.get_clock().now().to_msg()
    ros.state.publish(message)


def test_request_opens_car_and_auto_confirms_once_per_candidate(ros):
    node = ElevatorSimNode(building=load_floor_config(), parameter_overrides=[
        Parameter('start_floor', value='4F'), Parameter('press_delay_s', value=.1),
        Parameter('confirm_delay_s', value=.1)])
    ros.executor.add_node(node)
    ros.spin(.5)
    ros.request.publish(ElevatorRequest(kind=ElevatorRequest.KIND_CALL, start_floor='4F',
                                        target_floor='3F', direction=ElevatorRequest.DIR_DOWN, sequence=1))
    assert ros.spin(5., lambda: ros.statuses and ros.statuses[-1]['door'] == 'OPEN')
    candidate(ros, '4F')
    assert ros.spin(2., lambda: ros.confirms)
    assert ros.confirms[-1].actual_floor == ''
    assert ros.confirms[-1].accepted and ros.confirms[-1].source == FloorArrivalConfirm.SOURCE_SIM
    assert ros.confirms[-1].candidate_id == 7
    for _ in range(3):
        candidate(ros, '4F')  # new publication stamp must not generate another answer
        ros.spin(.15)
    assert len(ros.confirms) == 1
    candidate(ros, '4F', False)
    ros.spin(.1)
    candidate(ros, '4F')
    ros.spin(.3)
    assert len(ros.confirms) == 1
    candidate(ros, '4F', candidate_id=8)
    assert ros.spin(2., lambda: len(ros.confirms) == 2)
    assert ros.confirms[-1].candidate_id == 8 and ros.confirms[-1].accepted
    candidate(ros, '3F', candidate_id=9)
    assert ros.spin(2., lambda: len(ros.confirms) == 3)
    assert not ros.confirms[-1].accepted and ros.confirms[-1].candidate_id == 9
    assert ros.confirms[-1].actual_floor == '4F'


def test_auto_confirm_can_be_disabled(ros):
    node = ElevatorSimNode(building=load_floor_config(), parameter_overrides=[Parameter('auto_confirm', value=False)])
    ros.executor.add_node(node)
    ros.spin(.5)
    candidate(ros, '4F')
    ros.spin(1.5)
    assert not ros.confirms


def test_relay_request_json_and_ground_truth_confirmation(ros):
    node = ElevatorRelayNode(parameter_overrides=[Parameter('confirm_delay_s', value=.1)])
    ros.executor.add_node(node)
    ros.spin(.5)
    ros.request.publish(ElevatorRequest(kind=ElevatorRequest.KIND_CAR_FLOOR, target_floor='5F', sequence=2))
    assert ros.spin(2., lambda: ros.raw)
    assert ros.raw[-1] == dict(kind=2, start_floor='', target_floor='5F', direction=0, sequence=2)
    candidate(ros, '5F')
    ros.status.publish(String(data=json.dumps(dict(moving=False, car_floor='5F', door='OPEN'))))
    assert ros.spin(.8, lambda: ros.confirms)
    assert ros.confirms[-1].accepted and ros.confirms[-1].candidate_id == 7


def test_late_adapter_replays_only_latest_request(ros):
    ros.request.publish(ElevatorRequest(kind=ElevatorRequest.KIND_CALL, start_floor='4F',
                                       target_floor='3F', direction=ElevatorRequest.DIR_DOWN, sequence=1))
    ros.request.publish(ElevatorRequest(kind=ElevatorRequest.KIND_CANCEL, sequence=2))
    ros.spin(.2)
    node = ElevatorSimNode(building=load_floor_config(), parameter_overrides=[
        Parameter('start_floor', value='4F'), Parameter('press_delay_s', value=.1)])
    ros.executor.add_node(node)
    assert ros.spin(2., lambda: ros.statuses)
    ros.spin(2.)
    assert all(s['door'] == 'CLOSED' and not s['moving'] for s in ros.statuses)


def test_late_relay_forwards_only_latest_request(ros):
    ros.request.publish(ElevatorRequest(kind=ElevatorRequest.KIND_CAR_FLOOR, target_floor='5F', sequence=1))
    ros.request.publish(ElevatorRequest(kind=ElevatorRequest.KIND_CANCEL, sequence=2))
    ros.spin(.2)
    node = ElevatorRelayNode()
    ros.executor.add_node(node)
    assert ros.spin(2., lambda: ros.raw)
    ros.spin(.3)
    assert ros.raw == [dict(kind=3, start_floor='', target_floor='', direction=0, sequence=2)]
    assert node.raw_pub.qos_profile.depth == 1
