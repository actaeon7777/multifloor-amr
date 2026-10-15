"""Opt-in ROS test: floor_mission_manager admission and failure handling with fake peers (no Isaac)."""
import json
import os
import time
from types import SimpleNamespace

import pytest

if os.environ.get('AMR_RUN_ELEVATOR_MISSION_TESTS') != '1':
    pytest.skip('Set AMR_RUN_ELEVATOR_MISSION_TESTS=1 in domain 193', allow_module_level=True)

from action_msgs.msg import GoalStatus
from amr_elevator_interfaces.action import NavigateToFloorPose
from amr_elevator_interfaces.msg import ElevatorBeacon, ElevatorCommand, ElevatorDoorState, FloorState
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool

from amr_elevator.mission_node import FloorMissionManager
from amr_stack.floor_config import load_floor_config

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@pytest.fixture
def ros():
    assert os.environ['ROS_DOMAIN_ID'] == '193'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rclpy.init()
    executor = SingleThreadedExecutor()
    peer = rclpy.create_node('mission_test_peer')
    executor.add_node(peer)
    commands, statuses = [], []
    peer.create_subscription(ElevatorCommand, 'elevator/command', commands.append, 20)
    peer.create_subscription(String, 'floor_mission/status', lambda m: statuses.append(json.loads(m.data)), LATCHED)
    pubs = SimpleNamespace(
        floor=peer.create_publisher(FloorState, 'floor/state', LATCHED),
        door=peer.create_publisher(ElevatorDoorState, 'elevator/door', 10),
        beacon=peer.create_publisher(ElevatorBeacon, 'elevator/beacon', 10),
        localization=peer.create_publisher(String, 'localization/status', LATCHED),
        mode=peer.create_publisher(String, 'control_mux/mode', LATCHED))
    navigation_goals, nav_outcomes = [], []

    def navigate(handle):
        navigation_goals.append(handle.request.pose.pose.position)
        if nav_outcomes and nav_outcomes.pop(0) == 'abort':
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result()
    ActionServer(peer, NavigateToPose, 'navigate_to_pose', navigate)
    calls = {}
    for name in ('localization/suspend', 'navigation/maneuver_lease'):
        def service(request, response, name=name):
            calls.setdefault(name, []).append(request.data)
            response.success = True
            return response
        peer.create_service(SetBool, name, service)

    def spin(duration, until=None):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
            if until is not None and until():
                return True
        return until is None

    def floor_state(floor='4F', zone=FloorState.ZONE_OUTSIDE, confidence='BEACON'):
        pubs.floor.publish(FloorState(zone=zone, floor=floor, confidence=confidence))

    pubs.localization.publish(String(data=json.dumps({'state': 'READY', 'ready': True})))
    pubs.mode.publish(String(data='AUTO'))
    yield SimpleNamespace(executor=executor, peer=peer, pubs=pubs, spin=spin, commands=commands,
                          statuses=statuses, navigation_goals=navigation_goals, calls=calls,
                          nav_outcomes=nav_outcomes,
                          floor_state=floor_state)
    executor.shutdown()
    rclpy.shutdown()


def send(ros, floor, room='302'):
    client = ActionClient(ros.peer, NavigateToFloorPose, 'floor_mission/navigate')
    assert client.wait_for_server(timeout_sec=3.)
    building = load_floor_config()
    goal = NavigateToFloorPose.Goal(floor=floor, destination_id=room)
    goal.pose.header.frame_id = 'map'
    target = building.floors[floor].rooms.get(room) if floor in building.floors else None
    goal.pose.pose.position.x, goal.pose.pose.position.y = (target.goal[:2] if target else (1., 1.))
    goal.pose.pose.orientation.w = 1.
    future = client.send_goal_async(goal)
    assert ros.spin(3., future.done)
    return future.result()


def manager(ros, simulation=True, **overrides):
    node = FloorMissionManager(building=load_floor_config(), parameter_overrides=[
        Parameter('simulation', value=simulation), Parameter('ack_timeout', value=.4),
        *[Parameter(k, value=v) for k, v in overrides.items()]])
    ros.executor.add_node(node)
    return node


def test_goal_admission(ros):
    manager(ros, simulation=False)
    ros.floor_state(confidence='ASSUMED')
    ros.spin(.3)
    assert not send(ros, '4F').accepted          # current floor not confirmed
    ros.floor_state()
    ros.spin(.3)
    assert not send(ros, '3F').accepted          # provisional map: real vehicle refuses
    assert not send(ros, '9F').accepted          # unknown floor


def test_dead_button_device_stops_safely_at_the_hall(ros):
    manager(ros)
    ros.floor_state()
    ros.pubs.door.publish(ElevatorDoorState(state=ElevatorDoorState.CLOSED))
    ros.spin(.3)
    handle = send(ros, '3F', '302')
    assert handle.accepted
    result = handle.get_result_async()
    keep_alive = ros.peer.create_timer(.5, lambda: ros.floor_state())
    assert ros.spin(6., result.done)
    keep_alive.cancel()
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.error_code == 'NO_HALL_ACK'
    calls = [c for c in ros.commands if c.command == ElevatorCommand.CMD_HALL_CALL]
    assert len(calls) == 3 and {c.floor_code for c in calls} == {4}
    assert {c.direction for c in calls} == {ElevatorCommand.DIR_DOWN}
    assert len(ros.navigation_goals) == 1  # only the hall leg was driven
    assert ros.statuses[-1]['phase'] == 'FAILED_SAFE'


def test_acknowledged_call_waits_for_the_door_then_prepares_boarding(ros):
    manager(ros)
    ros.floor_state()
    ros.spin(.3)
    handle = send(ros, '5F', '502')
    keep_alive = ros.peer.create_timer(.5, lambda: ros.floor_state())
    assert ros.spin(3., lambda: ros.commands)
    call = ros.commands[0]
    assert (call.command, call.floor_code, call.direction) == (ElevatorCommand.CMD_HALL_CALL, 4, ElevatorCommand.DIR_UP)
    ros.pubs.beacon.publish(ElevatorBeacon(event=ElevatorBeacon.EVENT_BUTTON_ACK,
                                           location=ElevatorBeacon.LOCATION_HALL, floor_code=4,
                                           ack_sequence=call.sequence))
    assert ros.spin(2., lambda: ros.statuses[-1]['phase'] == 'WAIT_DOOR_OPEN')
    ros.pubs.door.publish(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert ros.spin(3., lambda: ros.statuses[-1]['phase'] == 'BOARDING')
    assert ros.calls['localization/suspend'] == [True] and ros.calls['navigation/maneuver_lease'][0] is True
    assert any(c.command == ElevatorCommand.CMD_DOOR_OPEN for c in ros.commands)  # hold the door
    cancel = handle.cancel_goal_async()
    assert ros.spin(2., cancel.done)
    result = handle.get_result_async()
    assert ros.spin(3., result.done)
    keep_alive.cancel()
    assert result.result().status == GoalStatus.STATUS_CANCELED
    assert ros.calls['navigation/maneuver_lease'][-1] is False  # lease released on cancel


def test_lost_floor_state_stops_the_mission(ros):
    manager(ros, floor_state_timeout=1.)
    ros.floor_state()
    ros.spin(.3)
    handle = send(ros, '3F', '302')
    assert handle.accepted
    result = handle.get_result_async()
    assert ros.spin(4., result.done)  # nobody republishes /floor/state
    assert result.result().result.error_code == 'FLOOR_STATE_LOST'


def test_same_floor_goal_waits_for_localization_recovery(ros):
    manager(ros)
    ros.floor_state()
    ros.spin(.3)
    ros.nav_outcomes.append('abort')
    handle = send(ros, '4F', '402')
    keep_alive = ros.peer.create_timer(.5, lambda: ros.floor_state())
    ros.pubs.localization.publish(String(data=json.dumps({'state': 'VERIFYING', 'ready': False})))
    result = handle.get_result_async()
    ros.spin(2.)
    assert not result.done(), (result.result().status, result.result().result.error_code, ros.statuses[-1])
    assert len(ros.navigation_goals) == 1  # paused, retries not used up
    ros.pubs.localization.publish(String(data=json.dumps({'state': 'READY', 'ready': True})))
    assert ros.spin(3., result.done)
    keep_alive.cancel()
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED and len(ros.navigation_goals) == 2


def test_late_ack_of_an_earlier_press_is_accepted(ros):
    manager(ros)
    ros.floor_state()
    ros.pubs.door.publish(ElevatorDoorState(state=ElevatorDoorState.CLOSED))
    ros.spin(.3)
    send(ros, '3F', '302')
    keep_alive = ros.peer.create_timer(.5, lambda: ros.floor_state())
    assert ros.spin(3., lambda: len([c for c in ros.commands if c.command == ElevatorCommand.CMD_HALL_CALL]) >= 2)
    first = ros.commands[0]
    ros.pubs.beacon.publish(ElevatorBeacon(event=ElevatorBeacon.EVENT_BUTTON_ACK,
                                           location=ElevatorBeacon.LOCATION_HALL, floor_code=4,
                                           ack_sequence=first.sequence))  # answers the first press only
    assert ros.spin(2., lambda: ros.statuses[-1]['phase'] == 'WAIT_DOOR_OPEN')
    keep_alive.cancel()
