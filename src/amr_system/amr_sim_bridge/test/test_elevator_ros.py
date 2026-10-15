"""Opt-in ROS test: simulated elevator and the CAN relay speak the same topics."""
import json
import os
import time
from types import SimpleNamespace

import pytest

if os.environ.get('AMR_RUN_ELEVATOR_TESTS') != '1':
    pytest.skip('Set AMR_RUN_ELEVATOR_TESTS=1 in domain 193', allow_module_level=True)

from amr_elevator_interfaces.msg import ElevatorBeacon, ElevatorCommand
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from amr_can_bridge import elevator_can_link
from amr_can_bridge.elevator_protocol import (
    ELEVATOR_COMMAND_CAN_ID, ELEVATOR_EVENT_CAN_ID, EVENT_PRESENCE, LOCATION_HALL, ElevatorEvent,
    encode_event,
)
from amr_sim_bridge.elevator_node import ElevatorSimNode
from amr_stack.floor_config import load_floor_config


@pytest.fixture
def ros():
    assert os.environ['ROS_DOMAIN_ID'] == '193'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rclpy.init()
    executor = SingleThreadedExecutor()
    probe = rclpy.create_node('elevator_test_probe')
    executor.add_node(probe)
    beacons, statuses = [], []
    probe.create_subscription(ElevatorBeacon, 'elevator/beacon', beacons.append, 50)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    probe.create_subscription(String, 'sim/elevator/status',
                              lambda m: statuses.append(json.loads(m.data)), latched)
    command_pub = probe.create_publisher(ElevatorCommand, 'elevator/command', 10)
    inject_pub = probe.create_publisher(String, 'sim/elevator/inject', 10)
    truth_pub = probe.create_publisher(Odometry, 'sim/ground_truth', 10)

    def spin(duration, until=None):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
            if until is not None and until():
                return True
        return until is None

    yield SimpleNamespace(executor=executor, beacons=beacons, statuses=statuses, spin=spin,
                          command=command_pub, inject=inject_pub, truth=truth_pub)
    executor.shutdown()
    rclpy.shutdown()


def command(kind, floor, direction, sequence):
    return ElevatorCommand(command=kind, floor_code=floor, direction=direction, sequence=sequence)


def test_simulated_elevator_answers_commands_and_tracks_the_robot(ros):
    building = load_floor_config()
    node = ElevatorSimNode(building=building)
    ros.executor.add_node(node)
    assert ros.spin(3., lambda: any(b.floor_code == 4 for b in ros.beacons))
    first = next(b for b in ros.beacons if b.floor_code == 4)
    assert (first.event, first.location, first.source) == (EVENT_PRESENCE, LOCATION_HALL, 'sim')

    ros.command.publish(command(ElevatorCommand.CMD_HALL_CALL, 4, ElevatorCommand.DIR_DOWN, 21))
    assert ros.spin(2., lambda: any(b.event == ElevatorBeacon.EVENT_BUTTON_ACK for b in ros.beacons))
    ack = next(b for b in ros.beacons if b.event == ElevatorBeacon.EVENT_BUTTON_ACK)
    assert (ack.location, ack.floor_code, ack.ack_sequence) == (LOCATION_HALL, 4, 21)
    assert ros.spin(4., lambda: ros.statuses and ros.statuses[-1]['door'] in ('OPENING', 'OPEN'))
    assert ros.statuses[-1]['car_floor'] == '4F' and ros.statuses[-1]['car_z_m'] == 0.

    # Ground truth inside the 4F car polygon -> robot_in_car; the car height must match too.
    truth = Odometry()
    truth.pose.pose.position.x, truth.pose.pose.position.y = 8.2, 3.31
    ros.truth.publish(truth)
    assert ros.spin(2., lambda: ros.statuses[-1]['robot_in_car'])
    truth.pose.pose.position.z = 3.6  # same xy on 5F: not in the car
    ros.truth.publish(truth)
    assert ros.spin(2., lambda: not ros.statuses[-1]['robot_in_car'])

    ros.inject.publish(String(data=json.dumps({'faults': {'button_fails': True}})))
    ros.spin(.3)
    count = len(ros.beacons)
    ros.command.publish(command(ElevatorCommand.CMD_CAR_FLOOR, 5, 0, 22))
    ros.spin(1.)
    assert all(b.event == EVENT_PRESENCE for b in ros.beacons[count:])
    assert ros.statuses[-1]['faults'] == {'button_fails': True}
    node.destroy_node()


class LoopbackBus:
    """Echo every elevator command as the STM32 would acknowledge it."""
    def __init__(self, **kwargs):
        assert {f['can_id'] for f in kwargs['can_filters']} == {ELEVATOR_EVENT_CAN_ID}
        self.inbox, self.sent = [], []

    def send(self, message, timeout):
        assert message.arbitration_id == ELEVATOR_COMMAND_CAN_ID
        self.sent.append(bytes(message.data))
        sequence = message.data[1]
        self.inbox.append(SimpleNamespace(
            arbitration_id=ELEVATOR_EVENT_CAN_ID, timestamp=time.time(), is_extended_id=False,
            is_remote_frame=False, is_error_frame=False, is_fd=False,
            data=encode_event(ElevatorEvent(ElevatorBeacon.EVENT_BUTTON_ACK, 0,
                                            ElevatorBeacon.LOCATION_CAR, 5, sequence, -40))))

    def recv(self, timeout):
        return self.inbox.pop(0) if self.inbox else None

    def shutdown(self):
        pass


def test_can_link_relays_commands_and_acknowledgements(ros):
    pytest.importorskip('can')
    node = elevator_can_link.ElevatorCanLink(
        bus_factory=LoopbackBus,
        parameter_overrides=[rclpy.parameter.Parameter('dry_run', value=False)])
    ros.executor.add_node(node)
    ros.spin(.3)
    ros.command.publish(command(ElevatorCommand.CMD_CAR_FLOOR, 3, 0, 77))
    assert ros.spin(2., lambda: ros.beacons)
    ack = ros.beacons[0]
    assert (ack.event, ack.location, ack.ack_sequence, ack.source) == \
        (ElevatorBeacon.EVENT_BUTTON_ACK, ElevatorBeacon.LOCATION_CAR, 77, 'can')
    assert node.link.bus.sent[0][1:5] == bytes([77, 2, 3, 0])
    ros.command.publish(command(ElevatorCommand.CMD_HALL_CALL, 3, 0, 78))  # invalid: no direction
    ros.spin(.3)
    assert len(node.link.bus.sent) == 1
    node.close()
    node.destroy_node()
