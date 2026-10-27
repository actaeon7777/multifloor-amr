"""Opt-in ROS test: floor_mission_manager admission and failure handling with fake peers (no Isaac)."""
import dataclasses
import json
import os
import time
from types import SimpleNamespace
from uuid import uuid4

import numpy as np
import pytest

if os.environ.get('AMR_RUN_ELEVATOR_MISSION_TESTS') != '1':
    pytest.skip('Set AMR_RUN_ELEVATOR_MISSION_TESTS=1 in domain 193', allow_module_level=True)

import math

from action_msgs.msg import GoalStatus
from amr_elevator_interfaces.action import NavigateToFloorPose
from amr_elevator_interfaces.msg import (
    ElevatorBeacon, ElevatorCommand, ElevatorCommandStatus, ElevatorDoorState, FloorState,
)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster

from amr_elevator.mission_node import FloorMissionManager
from amr_stack.floor_config import load_floor_config

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@pytest.fixture
def ros():
    assert os.environ['ROS_DOMAIN_ID'] == '193'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    # A previous test's DDS discovery cache must never satisfy readiness or
    # receive this test's action requests / transient-local samples. Also
    # isolate TF, whose default topics are absolute rather than relative.
    namespace = f'/mission_test_{uuid4().hex}'
    rclpy.init(args=['--ros-args', '-r', f'__ns:={namespace}',
                     '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
    executor = SingleThreadedExecutor()
    peer = rclpy.create_node('mission_test_peer')
    executor.add_node(peer)
    commands, statuses, announcements, twists, initial_poses = [], [], [], [], []
    peer.create_subscription(ElevatorCommand, 'elevator/command', commands.append, 20)
    peer.create_subscription(String, 'robot/announcement',
                             lambda m: announcements.append(json.loads(m.data)), 10)
    peer.create_subscription(Twist, 'cmd_vel_maneuver', twists.append, 10)
    peer.create_subscription(PoseWithCovarianceStamped, 'initialpose', initial_poses.append, 10)
    peer.create_subscription(String, 'floor_mission/status', lambda m: statuses.append(json.loads(m.data)), LATCHED)
    pubs = SimpleNamespace(
        floor=peer.create_publisher(FloorState, 'floor/state', LATCHED),
        pose=peer.create_publisher(PoseStamped, 'floor/pose_estimate', 10),
        door=peer.create_publisher(ElevatorDoorState, 'elevator/door', 10),
        beacon=peer.create_publisher(ElevatorBeacon, 'elevator/beacon', 10),
        command_status=peer.create_publisher(ElevatorCommandStatus, 'elevator/command_status', 10),
        localization=peer.create_publisher(String, 'localization/status', LATCHED),
        mode=peer.create_publisher(String, 'control_mux/mode', LATCHED),
        scan=peer.create_publisher(LaserScan, 'scan', qos_profile_sensor_data))
    scan_state = SimpleNamespace(message=None, enabled=True)
    def repeat_scan():
        if scan_state.enabled and scan_state.message is not None:
            pubs.scan.publish(scan_state.message)
    scan_timer = peer.create_timer(.1, repeat_scan)
    navigation_goals, nav_outcomes = [], []

    def navigate(handle):
        navigation_goals.append(handle.request.pose.pose.position)
        if nav_outcomes and nav_outcomes.pop(0) == 'abort':
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result()
    nav_server = ActionServer(peer, NavigateToPose, 'navigate_to_pose', navigate)
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
    def laser_tf(yaw):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'base_footprint', 'laser'
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        StaticTransformBroadcaster(peer).sendTransform(t)

    try:
        yield SimpleNamespace(executor=executor, peer=peer, pubs=pubs, laser_tf=laser_tf, spin=spin, commands=commands,
                              statuses=statuses, announcements=announcements, twists=twists, initial_poses=initial_poses,
                              navigation_goals=navigation_goals, calls=calls,
                              nav_server=nav_server, nav_outcomes=nav_outcomes,
                              floor_state=floor_state, scan_state=scan_state)
    finally:
        scan_state.enabled = False
        scan_timer.cancel()
        nodes = executor.get_nodes()
        # Stop scan, keep-alive and mission timers before shutting down the
        # executor. Tests also create timers directly on peer, so enumerate all.
        for node in nodes:
            for timer in tuple(node.timers):
                timer.cancel()
        executor.shutdown()
        try:
            for node in nodes:
                # Jazzy Node.destroy_node() does not destroy action waitables.
                # Explicitly release the fake Nav2 server, mission server and
                # ALL clients from send(), including rejected/repeated goals.
                # Otherwise their native handles survive until cyclic GC.
                for waitable in tuple(node.waitables):
                    if isinstance(waitable, (ActionClient, ActionServer)):
                        waitable.destroy()
                node.destroy_node()
        finally:
            rclpy.shutdown()


def send(ros, floor, room='302'):
    client = ActionClient(ros.peer, NavigateToFloorPose, 'floor_mission/navigate')
    assert client.wait_for_server(timeout_sec=3.)
    building = load_floor_config()
    # Missions now require a measured pose and scan before admitting boarding.
    # Supply the previously implicit clear hall/car; individual tests replace it.
    pose = building.floor('4F').elevators['E1'].hall_wait_pose
    message = PoseStamped()
    message.header.frame_id = 'map'
    message.pose.position.x, message.pose.position.y = pose[:2]
    message.pose.orientation.z, message.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
    ros.pubs.pose.publish(message)
    ros.laser_tf(0.)
    scan = LaserScan(angle_min=-math.pi, angle_increment=math.pi/180, range_min=.05, range_max=30.)
    scan.header.frame_id = 'laser'
    scan.ranges = [math.inf] * 360
    ros.scan_state.message = scan
    ros.pubs.scan.publish(scan)
    ros.spin(.1)  # static TF and scan may arrive in either order
    ros.scan_state.message = scan
    ros.pubs.scan.publish(scan)
    goal = NavigateToFloorPose.Goal(floor=floor, destination_id=room)
    goal.pose.header.frame_id = 'map'
    target = building.floors[floor].rooms.get(room) if floor in building.floors else None
    goal.pose.pose.position.x, goal.pose.pose.position.y = (target.goal[:2] if target else (1., 1.))
    goal.pose.pose.orientation.w = 1.
    future = client.send_goal_async(goal)
    assert ros.spin(3., future.done)
    return future.result()


def manager(ros, simulation=True, building=None, **overrides):
    node = FloorMissionManager(building=building or load_floor_config(), parameter_overrides=[
        Parameter('simulation', value=simulation), Parameter('ack_timeout', value=.4),
        *[Parameter(k, value=v) for k, v in overrides.items()]])
    ros.executor.add_node(node)
    # Discovery of floor_mission/navigate alone says nothing about Nav2.
    # Establish the fake dependency before any mission can send its hall goal.
    assert ros.spin(3., node.nav_client.server_is_ready), 'fake Nav2 action server not discovered'
    return node


def test_goal_admission(ros):
    building = load_floor_config()
    # Every mapped floor is enabled now; disable one to keep the refusal covered.
    building = dataclasses.replace(building, floors={
        **building.floors, '5F': dataclasses.replace(building.floors['5F'], real_enabled=False)})
    manager(ros, simulation=False, building=building)
    ros.floor_state(confidence='ASSUMED')
    ros.spin(.3)
    assert not send(ros, '4F').accepted          # current floor not confirmed
    ros.floor_state()
    ros.spin(.3)
    assert not send(ros, '5F').accepted          # real_enabled false: real vehicle refuses
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
    assert not any(c.command == ElevatorCommand.CMD_DOOR_OPEN for c in ros.commands)  # button off by default
    cancel = handle.cancel_goal_async()
    assert ros.spin(2., cancel.done)
    result = handle.get_result_async()
    assert ros.spin(3., result.done)
    keep_alive.cancel()
    assert result.result().status == GoalStatus.STATUS_CANCELED
    assert ros.calls['navigation/maneuver_lease'][-1] is False  # lease released on cancel


def test_manual_takeover_aborts_boarding_and_blocks_new_missions(ros):
    manager(ros)
    ros.floor_state()
    ros.spin(.3)
    handle = send(ros, '5F', '502')
    keep_alive = ros.peer.create_timer(.5, lambda: ros.floor_state())
    assert ros.spin(3., lambda: ros.commands)
    ros.pubs.beacon.publish(ElevatorBeacon(event=ElevatorBeacon.EVENT_BUTTON_ACK,
                                           location=ElevatorBeacon.LOCATION_HALL, floor_code=4,
                                           ack_sequence=ros.commands[0].sequence))
    ros.pubs.door.publish(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert ros.spin(4., lambda: ros.statuses[-1]['phase'] == 'BOARDING')
    ros.pubs.mode.publish(String(data='MANUAL'))
    result = handle.get_result_async()
    assert ros.spin(3., result.done)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.error_code == 'MANUAL_OVERRIDE'
    assert ros.calls['navigation/maneuver_lease'][-1] is False
    assert ros.spin(1., lambda: ros.calls['localization/suspend'][-1] is False)  # outside: resumed
    assert ros.statuses[-1]['phase'] == 'FAILED_SAFE'
    assert not send(ros, '4F', '402').accepted
    ros.pubs.mode.publish(String(data='AUTO'))
    ros.spin(.3)
    keep_alive.cancel()
    goals = len(ros.navigation_goals)
    assert send(ros, '4F', '402').accepted  # a new goal, not the aborted mission
    assert ros.spin(3., lambda: len(ros.navigation_goals) > goals)


@pytest.mark.parametrize('button', [False, True])
def test_estop_aborts_boarding_and_blocks_missions_until_ready(ros, button):
    node = manager(ros, require_estop=True, use_door_open_button=button)
    publisher = ros.peer.create_publisher(String, 'estop/status', LATCHED)
    estop = {'state': 'ARMING', 'epoch': 0, 'sequence': 0}

    def publish():
        estop['sequence'] += 1
        state = estop['state']
        publisher.publish(String(data=json.dumps(dict(
            session='mission_estop_test', sequence=estop['sequence'], epoch=estop['epoch'],
            stamp=time.time(), state=state, healthy=state == 'READY',
            latched=state in ('UNKNOWN', 'ESTOP')))))
    ros.peer.create_timer(.05, publish)
    ros.floor_state()
    assert ros.spin(1., lambda: node.estop.status.get('state') == 'ARMING')
    assert not send(ros, '5F', '502').accepted  # Missions wait for READY.
    estop['state'] = 'READY'
    assert ros.spin(1., node.estop_ready)
    handle = send(ros, '5F', '502')
    assert handle.accepted
    keep_alive = ros.peer.create_timer(.5, lambda: ros.floor_state())
    assert ros.spin(3., lambda: ros.commands)
    ros.pubs.beacon.publish(ElevatorBeacon(event=ElevatorBeacon.EVENT_BUTTON_ACK,
                                           location=ElevatorBeacon.LOCATION_HALL, floor_code=4,
                                           ack_sequence=ros.commands[0].sequence))
    ros.pubs.door.publish(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert ros.spin(4., lambda: ros.statuses[-1]['phase'] == 'BOARDING')
    if button:
        door = [c for c in ros.commands if c.command == ElevatorCommand.CMD_DOOR_OPEN]
        assert [c.repeat for c in door] == [6]  # One command holds the door while boarding.
        # Unacknowledged: pressed again with a new sequence and the same repeat count.
        assert ros.spin(2., lambda: sum(c.command == ElevatorCommand.CMD_DOOR_OPEN for c in ros.commands) == 2)
        door = [c for c in ros.commands if c.command == ElevatorCommand.CMD_DOOR_OPEN]
        assert door[1].repeat == 6 and door[1].sequence != door[0].sequence
    # The robot has now reached the doorway, restoring the repeat-15 premise.
    geometry = node.geometry('4F')
    doorway = PoseStamped()
    doorway.header.frame_id = 'map'
    doorway.pose.position.x = sum(p[0] for p in geometry.car_door) / 2
    doorway.pose.position.y = sum(p[1] for p in geometry.car_door) / 2
    doorway.pose.orientation.z, doorway.pose.orientation.w = math.sin(math.pi/2), math.cos(math.pi/2)
    ros.pubs.pose.publish(doorway)
    assert ros.spin(1., node.in_doorway)
    sent = len(ros.commands)
    estop.update(state='ESTOP', epoch=1)
    result = handle.get_result_async()
    assert ros.spin(3., result.done)
    keep_alive.cancel()
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.error_code == 'ESTOP'
    if button:
        # Stopped at the measured doorway pose: the car beacon holds the door for ~15 s.
        assert ros.spin(1., lambda: any(c.command == ElevatorCommand.CMD_DOOR_OPEN and c.repeat == 15
                                        for c in ros.commands[sent:]))
    else:
        ros.spin(1.)
        assert not any(c.command == ElevatorCommand.CMD_DOOR_OPEN for c in ros.commands)
    assert ros.calls['navigation/maneuver_lease'][-1] is False
    assert ros.statuses[-1]['phase'] == 'FAILED_SAFE'
    estop['state'] = 'ARMING'  # Released: the bridge re-arms by itself.
    ros.spin(.3)
    assert not send(ros, '4F', '402').accepted


def test_lost_floor_state_stops_the_mission(ros):
    # A long ack_timeout keeps the unanswered hall call from failing first.
    manager(ros, floor_state_timeout=1., ack_timeout=10.)
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


def test_rejected_hall_call_fails_at_once(ros):
    node = manager(ros, ack_timeout=3.)
    ros.floor_state()
    ros.spin(.3)
    handle = send(ros, '5F', '502')
    keep_alive = ros.peer.create_timer(.5, lambda: ros.floor_state())
    assert ros.spin(3., lambda: ros.commands)
    call = ros.commands[0]
    # RF_TIMEOUT only waits for the normal retry; DEVICE_REJECTED can never succeed.
    ros.pubs.command_status.publish(ElevatorCommandStatus(
        cmd_seq=call.sequence, status=ElevatorCommandStatus.STATUS_RF_TIMEOUT,
        device_result=ElevatorCommandStatus.RESULT_NONE, target=0x44, attempts=2))
    ros.spin(.3)
    assert node.mission.phase == 'CALL_ELEVATOR'
    ros.pubs.command_status.publish(ElevatorCommandStatus(
        cmd_seq=call.sequence, status=ElevatorCommandStatus.STATUS_DEVICE_REJECTED,
        device_result=ElevatorCommandStatus.RESULT_UNSUPPORTED, target=0x44, attempts=1))
    result = handle.get_result_async()
    assert ros.spin(1.5, result.done)  # Well before ack_timeout x ack_attempts.
    keep_alive.cancel()
    assert result.result().result.error_code == 'HALL_REJECTED'
    assert 'UNSUPPORTED' in result.result().result.message
    assert sum(c.command == ElevatorCommand.CMD_HALL_CALL for c in ros.commands) == 1


def test_slow_exit_rearms_the_door_open_repeats(ros):
    from rclpy.task import Future
    from amr_elevator.mission_node import Mission, PendingAck

    node = manager(ros, use_door_open_button=True, door_open_rearm_s=1., door_open_rearm_max=1)
    building = load_floor_config()
    geometry = building.elevator_geometry('3F', 'E1')
    # finished keeps the node's own tick away; only the re-arm rule runs here.
    node.mission = Mission(None, Future(), '3F', (0., 0., 0.), '', '4F', phase='ALIGHTING',
                           finished=True)
    node.pose = geometry.car_pose  # Still in the car: the door is needed.
    node.door_pending = PendingAck(9, ElevatorBeacon.LOCATION_CAR, (ElevatorCommand.CMD_DOOR_OPEN, 0, 0),
                                   node.now() - 2., 1, sequences={9}, repeat=10, acked=True,
                                   acked_at=node.now() - .5)
    node.rearm_door_open(geometry)
    assert not ros.spin(.2, lambda: ros.commands)  # Repeats still running.
    node.door_pending.acked_at = node.now() - 1.5
    node.rearm_door_open(geometry)
    assert ros.spin(1., lambda: ros.commands)
    assert (ros.commands[0].command, ros.commands[0].repeat) == (ElevatorCommand.CMD_DOOR_OPEN, 10)
    node.door_pending.acked, node.door_pending.acked_at = True, node.now() - 5.
    node.rearm_door_open(geometry)  # door_open_rearm_max reached.
    ros.spin(.3)
    assert len(ros.commands) == 1
    node.mission.counters['_door_rearms'] = 0
    exit_pose = building.floor('3F').elevators['E1'].hall_exit_pose
    node.pose = exit_pose  # Clear of the threshold: nothing to hold.
    node.rearm_door_open(geometry)
    ros.spin(.3)
    assert len(ros.commands) == 1
    node.mission = None


def test_clearance_uses_the_laser_frame_from_tf(ros):
    node = manager(ros)
    n = 360
    scan = LaserScan(angle_min=-math.pi, angle_increment=2 * math.pi / n, range_min=.05, range_max=30.)
    scan.header.frame_id = 'laser'
    scan.ranges = [math.inf] * n
    scan.ranges[0] = 1.0                 # laser bearing -pi: behind the laser, in front of the turned S2's robot
    ros.scan_state.message = scan
    ros.pubs.scan.publish(scan)
    ros.spin(.3)
    assert node.scan is not None and node.laser_pose is None
    assert node.clearance(False) == 0.   # frame unknown: hold rather than drive blind
    ros.laser_tf(math.pi)
    ros.scan_state.message = scan
    ros.pubs.scan.publish(scan)
    assert ros.spin(2., lambda: node.laser_pose is not None)
    assert abs(abs(node.laser_pose[2]) - math.pi) < 1e-6
    assert abs(node.clearance(False) - .68) < 1e-6
    assert node.clearance(True) == math.inf


def wait_at_open_door(ros, **overrides):
    """Reach WAIT_DOOR_OPEN with fake ACK, live floor state and valid sensors."""
    node = manager(ros, **overrides)
    ros.floor_state()
    ros.spin(.3)
    handle = send(ros, '3F', '302')
    assert handle.accepted
    ros.peer.create_timer(.3, ros.floor_state)
    # Keep the original three-second budget, but distinguish an unexecuted
    # Nav2 goal from a missing result / subsequent hall call when it fails.
    deadline = time.monotonic() + 3.
    def navigation_state():
        return (f'phase={node.mission.phase}, nav={node.nav}, '
                f'fake_goals={len(ros.navigation_goals)}, '
                f'nav_ready={node.nav_client.server_is_ready()}, localization={node.localization}')
    assert ros.spin(max(0., deadline - time.monotonic()), lambda: ros.navigation_goals), navigation_state()
    assert ros.spin(max(0., deadline - time.monotonic()), lambda: ros.commands), navigation_state()
    call = ros.commands[0]
    ros.pubs.beacon.publish(ElevatorBeacon(event=ElevatorBeacon.EVENT_BUTTON_ACK,
                                          location=ElevatorBeacon.LOCATION_HALL, floor_code=4,
                                          ack_sequence=call.sequence))
    assert ros.spin(2., lambda: node.mission.phase == 'WAIT_DOOR_OPEN')
    assert ros.spin(1., lambda: node.pose is not None and node.laser_pose is not None)
    return node


def publish_point(ros, xy=None):
    """One synthetic return in the identity-mounted laser (or a clear scan)."""
    angle = math.atan2(xy[1], xy[0]) if xy is not None else 0.
    scan = LaserScan(angle_min=angle, angle_increment=.01, range_min=.05, range_max=30.)
    scan.header.frame_id = 'laser'
    scan.ranges = [math.hypot(*xy)] if xy is not None else [math.inf]
    ros.scan_state.message = scan
    ros.pubs.scan.publish(scan)


def car_point_in_base(node):
    x, y, yaw = node.pose
    cx, cy = node.geometry('4F').car_pose[:2]
    return ((cx-x)*math.cos(yaw)+(cy-y)*math.sin(yaw),
            -(cx-x)*math.sin(yaw)+(cy-y)*math.cos(yaw))


def test_occupied_car_announces_then_clear_interval_allows_boarding(ros):
    node = wait_at_open_door(ros, clear_confirm_s=.15, announce_period=.1)
    publish_point(ros, car_point_in_base(node))
    assert ros.spin(1., lambda: len(node._scan_points) == 1)
    ros.pubs.door.publish(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert ros.spin(2., lambda: any(a['key'] == 'board_space' for a in ros.announcements))
    assert node.mission.phase == 'WAIT_DOOR_OPEN' and node.maneuver is None
    assert ros.twists and all(t.linear.x == 0. and t.angular.z == 0. for t in ros.twists)
    ros.spin(.25)
    assert node.mission.phase == 'WAIT_DOOR_OPEN'
    publish_point(ros)
    assert ros.spin(1., lambda: len(node._scan_points) == 0)
    assert node.mission.phase == 'WAIT_DOOR_OPEN'  # confirmation time has not passed
    assert ros.spin(2., lambda: node.mission.phase == 'BOARDING')
    assert node.maneuver is not None
    assert ros.spin(1., lambda: any(a['key'] == 'board_start' for a in ros.announcements))
    assert {'call', 'waiting'} <= {a['key'] for a in ros.announcements}
    for announcement in ros.announcements:
        assert set(announcement) == {'key', 'text', 'phase', 'floor', 'stamp'}


def test_person_leaving_through_the_hall_approach_delays_boarding(ros):
    node = wait_at_open_door(ros, clear_confirm_s=.15, announce_period=.1)
    hall_door = np.mean(node.geometry('4F').hall_door, axis=0)
    x, y, yaw = node.pose
    # Between the bumper and the hall door: outside the car, still on the path.
    walker = ((hall_door[0]-x)*math.cos(yaw)+(hall_door[1]-y)*math.sin(yaw) - .2, 0.)
    assert .348 < walker[0]
    publish_point(ros, walker)
    assert ros.spin(1., lambda: len(node._scan_points) == 1)
    ros.pubs.door.publish(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert ros.spin(2., lambda: any(a['key'] == 'board_space' for a in ros.announcements))
    ros.spin(.3)
    assert node.mission.phase == 'WAIT_DOOR_OPEN' and node.maneuver is None
    publish_point(ros)  # the person has passed the robot
    assert ros.spin(2., lambda: node.mission.phase == 'BOARDING')


def test_door_closes_while_space_occupied_relocalizes_and_recalls(ros):
    node = wait_at_open_door(ros)
    publish_point(ros, car_point_in_base(node))
    assert ros.spin(1., lambda: len(node._scan_points) == 1)
    ros.pubs.door.publish(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert ros.spin(2., lambda: any(a['key'] == 'board_space' for a in ros.announcements))
    ros.pubs.door.publish(ElevatorDoorState(state=ElevatorDoorState.CLOSING))
    assert ros.spin(3., lambda: node.mission.phase == 'RELOCALIZE')
    assert node.mission.counters['boarding_attempts'] == 1
    assert node.maneuver is None
    assert ros.spin(1., lambda: ros.calls['navigation/maneuver_lease'][-1] is False)
    assert ros.spin(1., lambda: node.mission.counters.get('_relocalize') == 'seed')
    ros.pubs.localization.publish(String(data=json.dumps({'state': 'WAITING_INITIAL_POSE', 'ready': False})))
    assert ros.spin(1., lambda: node.mission.counters.get('_relocalize') == 'verify')
    ros.pubs.localization.publish(String(data=json.dumps({'state': 'READY', 'ready': True})))
    assert ros.spin(1., lambda: node.mission.phase == 'CALL_ELEVATOR')
    assert any(a['key'] == 'board_cancel' for a in ros.announcements)
    assert not any(a['key'] == 'board_start' for a in ros.announcements)


def test_continuously_blocked_boarding_backs_out(ros):
    node = wait_at_open_door(ros, board_block_timeout=.2, clear_confirm_s=.05, announce_period=.1,
                             obstacle_announce_delay=0.)
    ros.pubs.door.publish(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert ros.spin(2., lambda: node.mission.phase == 'BOARDING')
    publish_point(ros, (.4, 0.))
    assert ros.spin(1., lambda: node.maneuver is not None and node.maneuver.blocked)
    assert ros.twists[-1].linear.x == 0.
    assert ros.spin(1., lambda: node.mission.phase == 'BACKING_OUT')
    assert node.mission.counters['boarding_aborts'] == 1
    assert ros.spin(1., lambda: {'obstacle', 'board_cancel'} <= {a['key'] for a in ros.announcements})


def test_blocked_in_car_turn_holds_zero_then_cancels_boarding(ros):
    from amr_elevator.maneuver import LineManeuver, TURN

    node = wait_at_open_door(ros, turn_block_timeout=.5)
    geometry = node.geometry('4F')
    node.pose = (*geometry.car_pose[:2], geometry.car_pose[2] + math.pi)
    node.set_phase('IN_CAR_ALIGN')
    node.maneuver = LineManeuver(geometry.car_pose, final_yaw=geometry.car_pose[2])
    node.maneuver.phase = TURN
    publish_point(ros, (0., .45))  # outside inflated starting body, inside 180-degree sweep
    ros.pubs.door.publish(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert ros.spin(1., lambda: node.maneuver.blocked)
    ros.twists.clear()
    ros.spin(.2)
    assert node.mission.phase == 'IN_CAR_ALIGN'
    assert ros.twists and all(t.linear.x == 0. and t.angular.z == 0. for t in ros.twists)
    assert any(a['key'] == 'board_space' for a in ros.announcements)
    assert ros.spin(1., lambda: node.mission.phase == 'BACKING_OUT')


def test_return_retry_reuses_loaded_map_and_clears_arrival_timers(ros):
    from amr_elevator.maneuver import LineManeuver, DONE

    node = wait_at_open_door(ros)
    node.pose = node.geometry('3F').car_pose
    node.mission.counters.update(_loaded_map='3F', _last_transit=-100., _assumed_since=-100., _assumed_hold=True)
    node.set_phase('RETURN_TO_CAR')
    node.maneuver = LineManeuver(node.pose)
    node.maneuver.phase = DONE
    node.mission.counters['_return_wait_since'] = node.now() - 16.
    ros.pubs.door.publish(ElevatorDoorState(state=ElevatorDoorState.CLOSED))
    assert ros.spin(1., lambda: node.mission.phase == 'SELECT_FLOOR')
    assert ros.spin(1., lambda: any(c.command == ElevatorCommand.CMD_CAR_FLOOR and c.floor_code == 3
                                   for c in ros.commands))
    press = next(c for c in reversed(ros.commands) if c.command == ElevatorCommand.CMD_CAR_FLOOR)
    ros.pubs.beacon.publish(ElevatorBeacon(event=ElevatorBeacon.EVENT_BUTTON_ACK,
                                          location=ElevatorBeacon.LOCATION_CAR, floor_code=3,
                                          ack_sequence=press.sequence))
    assert ros.spin(1., lambda: node.mission.phase == 'RIDING')
    assert all(k not in node.mission.counters for k in ('_last_transit', '_assumed_since', '_assumed_hold'))
    node.on_floor_state(FloorState(floor='3F', zone=FloorState.ZONE_IN_ELEVATOR, confidence='BEACON'))
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    node.phase_riding()
    assert node.mission.phase == 'SWITCH_MAP'
    node.phase_switch_map()  # no map server in fixture: cached map must suffice
    assert node.mission.phase == 'ALIGHTING' and node.maneuver is None


def test_depth_uses_cached_static_tf_and_expires(ros):
    import numpy as np
    from sensor_msgs.msg import PointCloud2, PointField

    node = manager(ros, depth_max_age=.1)
    node.mission = SimpleNamespace(phase='WAIT_DOOR_OPEN', finished=True)
    cloud = PointCloud2(height=1, width=1, point_step=16, row_step=20,
                        fields=[PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                for i, name in enumerate(('x', 'y', 'z'))])
    cloud.header.frame_id = 'camera_depth_optical_frame'
    cloud.data = np.array([0., 0., .45, 99., 99.], dtype='<f4').tobytes()
    node.on_depth(cloud)
    assert node.depth_transform is None  # no camera TF yet
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_footprint', cloud.header.frame_id
    # optical z -> base x, optical x -> -base y, optical y -> -base z
    transform.transform.translation.z = .3  # return is above the floor
    transform.transform.rotation.x = -.5
    transform.transform.rotation.y = .5
    transform.transform.rotation.z = -.5
    transform.transform.rotation.w = .5
    broadcaster = StaticTransformBroadcaster(ros.peer)
    broadcaster.sendTransform(transform)
    ros.laser_tf(0.)
    publish_point(ros)
    ros.spin(.2)
    publish_point(ros)
    assert ros.spin(1., lambda: node.laser_pose is not None)  # drops /tf, keeps /tf_static
    ros.spin(.1)
    node.on_depth(cloud)
    assert node.depth_transform is not None
    assert node.clearance(False) == pytest.approx(.13)
    assert node.obstacle_points().shape == (1, 2)
    ros.spin(.15)
    assert node.clearance(False) == math.inf
    assert node.obstacle_points().shape == (0, 2)


def test_occupied_exit_returns_to_car_and_shares_attempt_limit(ros):
    node = wait_at_open_door(ros, alight_block_timeout=.2, max_alighting_attempts=1)
    node.pose = node.geometry('3F').car_pose
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    publish_point(ros, (.6, 0.))
    assert ros.spin(1., lambda: len(node._scan_points) == 1)
    node.start_alighting()
    ros.twists.clear()
    assert ros.spin(1., lambda: node.mission.phase == 'RETURN_TO_CAR')
    assert node.mission.counters['alighting_attempts'] == 1
    assert ros.twists and all(t.linear.x == 0. and t.angular.z == 0. for t in ros.twists)
    assert ros.spin(1., lambda: 'alight_cancel' in {a['key'] for a in ros.announcements})
    assert not any(a['key'] == 'alight_start' for a in ros.announcements)
    # Same counter is consumed by a subsequent door-closing cancellation.
    node.start_alighting()
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.CLOSED))
    node.phase_alighting()
    assert node.mission.phase == 'FAILED_SAFE'
    assert 'ALIGHTING_FAILED' in node.mission.detail


def test_boarding_timeout_retreats_and_turn_timeout_cannot_retreat_at_other_floor(ros):
    from amr_elevator.maneuver import LineManeuver, TURN

    node = wait_at_open_door(ros, boarding_timeout=.3, turn_block_timeout=.2, clear_confirm_s=.05)
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert ros.spin(2., lambda: node.mission.phase == 'BOARDING')
    # Pose does not advance in this fake-peer fixture: the old BOARDING_TIMEOUT
    # failure is now a retreat, even without a persistent obstacle.
    assert ros.spin(1., lambda: node.mission.phase == 'BACKING_OUT')
    geometry = node.geometry('3F')
    node.pose = (*geometry.car_pose[:2], geometry.car_pose[2] + math.pi)
    node.set_phase('IN_CAR_ALIGN')
    node.maneuver = LineManeuver(geometry.car_pose, final_yaw=geometry.car_pose[2])
    node.maneuver.phase = TURN
    publish_point(ros, (0., .45))
    assert ros.spin(1., lambda: node.maneuver.blocked)
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.CLOSED))
    node.mission.counters['_blocked_since'] = node.now() - .25
    node.phase_in_car_align()
    assert node.mission.phase == 'IN_CAR_ALIGN'
    node.mission.counters['_blocked_since'] = node.now() - .45
    node.phase_in_car_align()
    assert node.mission.phase == 'FAILED_SAFE' and 'ALIGN_BLOCKED' in node.mission.detail


def occupied_wait(ros, **overrides):
    node = wait_at_open_door(ros, **overrides)
    publish_point(ros, car_point_in_base(node))
    assert ros.spin(1., lambda: len(node._scan_points) == 1)
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert ros.spin(2., lambda: node.mission.counters.get('_prepared') is True)
    assert node.mission.counters['_suspended'] is True
    return node


@pytest.mark.parametrize('reason', ['attempt_limit', 'wait_timeout'])
def test_hall_failure_resumes_suspended_localization_and_seeds_pose(ros, reason):
    node = occupied_wait(ros)
    if reason == 'attempt_limit':
        node.mission.counters['boarding_attempts'] = node.p['max_boarding_attempts'] - 1
        node.cancel_wait_boarding('attempt limit')
        code = 'BOARDING_FAILED'
    else:
        node.mission.counters['recalls'] = 1
        node.mission.phase_since = node.now() - node.p['door_wait_timeout'] - 1.
        node.phase_wait_door_open()
        code = 'DOOR_NOT_OPENED'
    assert node.mission.finished and code in node.mission.detail
    assert ros.spin(1., lambda: ros.calls['localization/suspend'][-1] is False)
    assert ros.spin(1., lambda: bool(ros.initial_poses))
    seed = ros.initial_poses[-1].pose.pose.position
    assert (seed.x, seed.y) == pytest.approx(node.pose[:2])
    assert not node.mission.counters.get('_suspended')
    assert ros.spin(1., lambda: 'robot_error' in {a['key'] for a in ros.announcements})


def test_failure_completes_even_if_announcement_transport_raises(ros, monkeypatch):
    node = occupied_wait(ros)
    def broken_announcement(key):
        assert key == 'robot_error'
        assert node.mission.finished and node.mission.phase == 'FAILED_SAFE'
        assert node.lease_renewed == -math.inf
        raise RuntimeError('synthetic announcement publication failure')
    monkeypatch.setattr(node, 'announce', broken_announcement)
    node.fail('TEST_FAILURE')
    assert node.mission.completion.done()
    assert ros.spin(1., lambda: bool(ros.initial_poses))
    assert ros.calls['localization/suspend'][-1] is False


def test_failure_inside_car_keeps_localization_suspended(ros):
    node = occupied_wait(ros)
    node.on_floor_state(FloorState(floor='4F', zone=FloorState.ZONE_IN_ELEVATOR, confidence='BEACON'))
    node.fail('TEST_FAILURE')
    ros.spin(.3)
    assert ros.calls['localization/suspend'] == [True]
    assert not ros.initial_poses


def test_door_flicker_holds_and_restarts_grace_and_space_confirmation(ros):
    node = occupied_wait(ros, door_cancel_grace=.5, clear_confirm_s=.3)
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.CLOSING))
    publish_point(ros)
    assert ros.spin(1., lambda: len(node._scan_points) == 0)
    ros.spin(.15)
    assert node.mission.phase == 'WAIT_DOOR_OPEN' and node.maneuver is None
    assert '_clear_since' not in node.mission.counters
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    node.phase_wait_door_open()
    assert '_door_not_open_since' not in node.mission.counters
    assert node.mission.phase == 'WAIT_DOOR_OPEN'  # clear-confirm timer has restarted
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.CLOSING))
    ros.spin(.35)
    assert node.mission.phase == 'WAIT_DOOR_OPEN'
    assert not node.mission.counters.get('boarding_attempts')
    assert all(t.linear.x == 0. and t.angular.z == 0. for t in ros.twists)
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert ros.spin(1., lambda: node.mission.phase == 'BOARDING')
    assert not any(a['key'] == 'board_cancel' for a in ros.announcements)


def freeze_phase_timer(node):
    """Keep fake-peer transport active while stepping phase methods explicitly."""
    for timer in node.timers:
        timer.cancel()


def test_deep_boarding_cannot_back_out_through_closed_door(ros, monkeypatch):
    from amr_elevator.maneuver import LineManeuver

    node = wait_at_open_door(ros, use_door_open_button=True)
    freeze_phase_timer(node)
    geometry = node.geometry('4F')
    node.pose = geometry.car_pose
    node.on_floor_state(FloorState(floor='4F', zone=FloorState.ZONE_IN_ELEVATOR, confidence='BEACON'))
    node.set_phase('BOARDING')
    node.maneuver = LineManeuver(geometry.car_pose, final_yaw=geometry.car_pose[2])
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.CLOSED))
    twists = []
    monkeypatch.setattr(node, 'publish_twist', lambda linear=0., angular=0.: twists.append((linear, angular)))
    assert not node.back_out('boarding timeout')
    assert twists == [(0., 0.)] and node.mission.phase == 'IN_CAR_ALIGN'
    assert not node.mission.counters.get('boarding_aborts')
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert node.back_out('turn timeout')
    assert node.mission.phase == 'BACKING_OUT' and node.maneuver.reverse
    assert ros.spin(1., lambda: any(c.command == ElevatorCommand.CMD_DOOR_OPEN and c.repeat == 6
                                   for c in ros.commands))


@pytest.mark.parametrize('front_in_car', [False, True])
def test_door_closing_during_retreat_stops_and_bounds_or_aligns(ros, front_in_car, monkeypatch):
    node = wait_at_open_door(ros)
    freeze_phase_timer(node)
    geometry = node.geometry('4F')
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert node.back_out('test retreat')
    # Use the hall door plane or car door plane to straddle the corresponding edge.
    door = geometry.car_door if front_in_car else geometry.hall_door
    node.pose = (sum(p[0] for p in door)/2, sum(p[1] for p in door)/2,
                 node.stop_geometry('4F').car_inward_yaw)
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.CLOSING))
    twists = []
    monkeypatch.setattr(node, 'publish_twist', lambda linear=0., angular=0.: twists.append((linear, angular)))
    node.phase_backing_out()
    assert twists and all(t == (0., 0.) for t in twists)
    if front_in_car:
        assert node.mission.phase == 'IN_CAR_ALIGN'
    else:
        assert node.mission.phase == 'BACKING_OUT'
        node.mission.phase_since = node.now() - node.p['boarding_timeout'] - 1.
        node.phase_backing_out()
        assert node.mission.finished and 'BACK_OUT_TIMEOUT' in node.mission.detail


@pytest.mark.parametrize('total_timeout', [False, True])
def test_return_to_car_blocked_or_phase_timeout_fails_and_holds_door(ros, total_timeout):
    from amr_elevator.maneuver import LineManeuver, DRIVE

    node = wait_at_open_door(ros, use_door_open_button=True, alight_block_timeout=.2,
                             obstacle_announce_delay=0.)
    freeze_phase_timer(node)
    geometry = node.geometry('3F')
    node.set_phase('RETURN_TO_CAR')
    node.pose = (sum(p[0] for p in geometry.car_door)/2,
                 sum(p[1] for p in geometry.car_door)/2, geometry.car_pose[2])
    node.maneuver = LineManeuver(geometry.car_pose, reverse=True)
    node.maneuver.phase = DRIVE
    publish_point(ros, (-.4, 0.))
    assert ros.spin(1., lambda: len(node._scan_points) == 1)
    node.phase_return_to_car()
    assert node.maneuver.blocked
    assert ros.spin(1., lambda: 'obstacle' in {a['key'] for a in ros.announcements})
    if total_timeout:
        node.mission.phase_since = node.now() - node.p['boarding_timeout'] - 1.
    else:
        node.mission.counters['_blocked_since'] = node.now() - .3
    node.phase_return_to_car()
    assert node.mission.finished and 'RETURN_BLOCKED' in node.mission.detail
    assert ros.spin(1., lambda: any(c.command == ElevatorCommand.CMD_DOOR_OPEN and c.repeat == 15
                                   for c in ros.commands))


@pytest.mark.parametrize('open_on_start_floor', [False, True])
def test_alignment_phase_bound_survives_block_flicker(ros, open_on_start_floor):
    from amr_elevator.maneuver import LineManeuver, TURN

    node = wait_at_open_door(ros, turn_block_timeout=.2)
    freeze_phase_timer(node)
    geometry = node.geometry('4F')
    node.pose = (*geometry.car_pose[:2], geometry.car_pose[2] + math.pi)
    node.set_phase('IN_CAR_ALIGN')
    node.maneuver = LineManeuver(geometry.car_pose, final_yaw=geometry.car_pose[2])
    node.maneuver.phase = TURN
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN if open_on_start_floor else ElevatorDoorState.CLOSED))
    # Both continuous timers are fresh after flicker, but phase-wide time expired.
    node.mission.counters.update(_blocked_since=node.now(), _align_active_since=node.now())
    node.mission.phase_since = node.now() - (2 * node.p['turn_block_timeout'] + 20.) - .1
    node.phase_in_car_align()
    if open_on_start_floor:
        assert node.mission.phase == 'BACKING_OUT'
    else:
        assert node.mission.finished and 'ALIGN_TIMEOUT' in node.mission.detail


def test_canceled_exit_repress_uses_ack_retries_and_does_not_repeat_board_announcements(ros):
    from amr_elevator.maneuver import LineManeuver, DONE

    node = wait_at_open_door(ros)
    node.set_phase('SELECT_FLOOR')  # original boarding announcements, once
    assert ros.spin(1., lambda: 'alight_floor' in {a['key'] for a in ros.announcements})
    node.set_phase('RETURN_TO_CAR')
    node.pose = node.geometry('3F').car_pose
    node.maneuver = LineManeuver(node.pose)
    node.maneuver.phase = DONE
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.CLOSED))
    node.mission.counters['_return_wait_since'] = node.now() - 16.
    previous = node.sequence
    assert ros.spin(1., lambda: node.mission.phase == 'SELECT_FLOOR')
    assert ros.spin(3., lambda: node.mission.finished)
    assert 'NO_CAR_ACK' in node.mission.detail
    def retries():
        return [c for c in ros.commands if c.command == ElevatorCommand.CMD_CAR_FLOOR and c.sequence > previous]
    assert ros.spin(1., lambda: len(retries()) == node.p['ack_attempts'])
    presses = retries()
    assert len(presses) == node.p['ack_attempts']
    assert len({c.sequence for c in presses}) == len(presses)
    assert sum(a['key'] == 'board_done' for a in ros.announcements) == 1
    assert sum(a['key'] == 'alight_floor' for a in ros.announcements) == 1


def test_clear_exit_has_no_obstacle_announcement_on_entry_or_confirmation(ros):
    node = wait_at_open_door(ros, clear_confirm_s=.15)
    node.pose = node.geometry('3F').car_pose
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    node.start_alighting()
    assert ros.spin(1., lambda: 'alight_start' in {a['key'] for a in ros.announcements})
    assert not any(a['key'] == 'obstacle' for a in ros.announcements)


def publish_test_pose(ros, pose):
    message = PoseStamped()
    message.header.frame_id = 'map'
    message.pose.position.x, message.pose.position.y = pose[:2]
    message.pose.orientation.z, message.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
    ros.pubs.pose.publish(message)


def test_closing_door_before_threshold_retreats_relocalizes_and_recalls(ros):
    node = wait_at_open_door(ros, clear_confirm_s=.05)
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert ros.spin(2., lambda: node.mission.phase == 'BOARDING')
    stop = node.stop_geometry('4F')
    x, y, yaw = stop.hall_wait_pose
    near_door = (x + .1 * math.cos(yaw), y + .1 * math.sin(yaw), yaw)
    publish_test_pose(ros, near_door)
    assert ros.spin(1., lambda: math.hypot(node.pose[0]-near_door[0], node.pose[1]-near_door[1]) < .001)
    assert node.depth('4F', node.geometry('4F').hall_door) + .32 < 0.
    ros.pubs.door.publish(ElevatorDoorState(state=ElevatorDoorState.CLOSING))
    assert ros.spin(1., lambda: node.mission.phase == 'BACKING_OUT')
    assert ros.spin(1., lambda: any(t.linear.x < 0. for t in ros.twists))
    publish_test_pose(ros, stop.hall_wait_pose)
    assert ros.spin(1., lambda: node.mission.phase == 'RELOCALIZE')
    assert ros.spin(1., lambda: node.mission.counters.get('_relocalize') == 'seed')
    ros.pubs.localization.publish(String(data=json.dumps({'state': 'WAITING_INITIAL_POSE', 'ready': False})))
    assert ros.spin(1., lambda: node.mission.counters.get('_relocalize') == 'verify')
    ros.pubs.localization.publish(String(data=json.dumps({'state': 'READY', 'ready': True})))
    assert ros.spin(1., lambda: node.mission.phase == 'CALL_ELEVATOR')
    assert ros.spin(1., lambda: sum(c.command == ElevatorCommand.CMD_HALL_CALL for c in ros.commands) == 2)
    assert node.mission.counters['boarding_aborts'] == node.mission.counters['boarding_attempts'] == 1


def test_return_arrival_allows_two_door_represses_then_floor_selection_beyond_drive_timeout(ros):
    from amr_elevator.maneuver import LineManeuver, DONE

    node = wait_at_open_door(ros, use_door_open_button=True)
    freeze_phase_timer(node)
    node.pose = node.geometry('3F').car_pose
    node.set_phase('RETURN_TO_CAR')
    node.maneuver = LineManeuver(node.pose, reverse=True)
    node.maneuver.phase = DONE
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.CLOSED))
    for attempt in (1, 2):
        node.mission.phase_since = node.now() - 40. * attempt
        node.mission.counters['_return_wait_since'] = node.now() - 16.
        node.phase_return_to_car()
        assert node.mission.phase == 'RETURN_TO_CAR' and not node.mission.finished
        assert node.mission.counters['return_open_press'] == attempt
        assert ros.spin(1., lambda: sum(c.command == ElevatorCommand.CMD_DOOR_OPEN and c.repeat == 10
                                       for c in ros.commands) == attempt)
    node.mission.counters['_return_wait_since'] = node.now() - 16.
    node.phase_return_to_car()
    assert node.mission.phase == 'SELECT_FLOOR'
    node.phase_select_floor()
    assert ros.spin(1., lambda: any(c.command == ElevatorCommand.CMD_CAR_FLOOR and c.floor_code == 3
                                   for c in ros.commands))


def test_flickering_exit_space_has_total_wait_bound_and_cancels_without_driving(ros, monkeypatch):
    from unittest.mock import Mock

    node = wait_at_open_door(ros, boarding_timeout=.1, alight_block_timeout=.6, clear_confirm_s=.4)
    freeze_phase_timer(node)
    node.pose = node.geometry('3F').car_pose
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    node.start_alighting()
    drive = Mock(side_effect=AssertionError('no drive before exit clearance or while already returned'))
    monkeypatch.setattr(node, 'drive', drive)
    for age, point in ((.2, (.6, 0.)), (.3, None), (.4, (.6, 0.)), (.5, None)):
        publish_point(ros, point)
        assert ros.spin(1., lambda: len(node._scan_points) == (0 if point is None else 1))
        node.mission.phase_since = node.now() - age
        node.phase_alighting()
        assert node.mission.phase == 'ALIGHTING' and not node.mission.finished
        assert not node.mission.counters.get('_exit_started')
    node.mission.phase_since = node.now() - .7
    node.phase_alighting()
    assert node.mission.phase == 'RETURN_TO_CAR' and node.maneuver is None
    assert node.mission.counters['_return_arrived'] and node.mission.counters['alighting_attempts'] == 1
    assert ros.spin(1., lambda: 'alight_cancel' in {a['key'] for a in ros.announcements})
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.CLOSED))
    node.mission.counters['_return_wait_since'] = node.now() - 16.
    node.phase_return_to_car()
    assert node.mission.phase == 'SELECT_FLOOR'
    drive.assert_not_called()
    assert ros.twists and all(t.linear.x == 0. and t.angular.z == 0. for t in ros.twists)


def test_alighting_drive_bound_starts_when_the_exit_drive_starts(ros):
    node = wait_at_open_door(ros, boarding_timeout=.5, clear_confirm_s=.05)
    freeze_phase_timer(node)
    node.pose = node.geometry('3F').car_pose
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    node.start_alighting()
    # A long wait for a clear exit must not eat the drive's own time bound.
    node.mission.phase_since = node.now() - 10.
    node.mission.counters.update(_exit_started=True, _exit_started_at=node.now())
    node.check_alighting_timeout()
    assert node.mission.phase == 'ALIGHTING'
    node.mission.counters['_exit_started_at'] = node.now() - 1.
    node.check_alighting_timeout()
    assert node.mission.phase == 'RETURN_TO_CAR'


@pytest.mark.parametrize('already_in_car', [True, False])
def test_exit_cancel_holds_the_door_only_from_the_doorway(ros, already_in_car):
    node = wait_at_open_door(ros, use_door_open_button=True)
    freeze_phase_timer(node)
    node.pose = node.geometry('3F').car_pose
    node.start_alighting()
    ros.commands.clear()
    node.return_to_car('test cancel', already_in_car=already_in_car)
    assert node.mission.phase == 'RETURN_TO_CAR'
    held = ros.spin(.5, lambda: any(c.command == ElevatorCommand.CMD_DOOR_OPEN for c in ros.commands))
    # From the car centre the door must close so SELECT_FLOOR can press the target again.
    assert held is (not already_in_car)


def test_stale_scan_holds_rotation_and_space_until_new_receive(ros):
    from amr_elevator.clearance import boarding_region
    from amr_elevator.maneuver import LineManeuver, TURN

    node = wait_at_open_door(ros, clear_confirm_s=.05)
    freeze_phase_timer(node)
    geometry = node.geometry('4F')
    node.pose = (*geometry.car_pose[:2], geometry.car_pose[2] + math.pi)
    node.set_phase('IN_CAR_ALIGN')
    node.maneuver = LineManeuver(geometry.car_pose, final_yaw=geometry.car_pose[2])
    node.maneuver.phase = TURN
    ros.scan_state.enabled = False
    ros.spin(.65)
    assert not node.scan_fresh() and node.clearance(False) == node.clearance(True) == 0.
    assert not node.space_clear(boarding_region(geometry, .1))
    ros.twists.clear()
    node.phase_in_car_align()
    assert node.maneuver.blocked
    assert ros.spin(1., lambda: bool(ros.twists))
    assert all(t.linear.x == 0. and t.angular.z == 0. for t in ros.twists)
    publish_point(ros)
    assert ros.spin(1., node.scan_fresh)
    node.phase_in_car_align()
    assert not node.maneuver.blocked
    assert ros.spin(1., lambda: any(t.angular.z != 0. for t in ros.twists))


def test_late_suspend_reply_after_prepare_failure_resumes_again_and_seeds(ros, monkeypatch):
    from rclpy.task import Future

    node = wait_at_open_door(ros)
    delayed = Future()
    real_call = node.suspend_client.call_async
    def delay_suspend(request):
        return delayed if request.data else real_call(request)
    monkeypatch.setattr(node.suspend_client, 'call_async', delay_suspend)
    node.mission.phase_since = node.now()
    node.on_door(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert ros.spin(1., lambda: node.mission.counters.get('_prepared') == 'pending')
    node.mission.phase_since = node.now() - node.p['door_wait_timeout'] - 1.
    assert ros.spin(1., lambda: node.mission.finished)
    assert 'PREPARE_FAILED' in node.mission.detail
    assert not delayed.done()
    assert ros.spin(1., lambda: ros.calls.get('localization/suspend', []).count(False) == 1)
    assert ros.spin(1., lambda: len(ros.initial_poses) == 1)
    delayed.set_result(SetBool.Response(success=True))
    assert ros.spin(1., lambda: ros.calls['localization/suspend'].count(False) == 2)
    assert ros.spin(1., lambda: len(ros.initial_poses) == 2)
    assert node.mission.finished and not node.mission.counters.get('_suspended')


def test_operator_hold_cancels_boarding_and_rejects_new_missions(ros):
    node = manager(ros)
    publisher = ros.peer.create_publisher(String, '/operator/hold', LATCHED)
    assert ros.spin(3., lambda: publisher.get_subscription_count() > 0)
    ros.floor_state()
    ros.spin(.3)
    handle = send(ros, '5F', '502')
    assert handle.accepted
    keep_alive = ros.peer.create_timer(.5, lambda: ros.floor_state())
    assert ros.spin(3., lambda: ros.commands)
    ros.pubs.beacon.publish(ElevatorBeacon(event=ElevatorBeacon.EVENT_BUTTON_ACK,
        location=ElevatorBeacon.LOCATION_HALL, floor_code=4, ack_sequence=ros.commands[0].sequence))
    ros.pubs.door.publish(ElevatorDoorState(state=ElevatorDoorState.OPEN))
    assert ros.spin(4., lambda: ros.statuses[-1]['phase'] == 'BOARDING')
    publisher.publish(String(data=json.dumps(dict(held=True, reason='central', stamp=time.time(), sequence=1))))
    result = handle.get_result_async()
    assert ros.spin(3., result.done)
    # Topic cancellation has no action CANCELING transition, so the transport aborts;
    # mission cleanup, door protection and terminal phase follow action cancel.
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.error_code == 'CANCELED'
    assert node.held and ros.statuses[-1]['phase'] == 'CANCELED'
    assert ros.calls['navigation/maneuver_lease'][-1] is False
    assert ros.spin(1., lambda: ros.calls['localization/suspend'][-1] is False)
    assert not send(ros, '4F', '402').accepted
    publisher.publish(String(data=json.dumps(dict(held=False, reason='central', stamp=time.time(), sequence=2))))
    assert ros.spin(1., lambda: not node.held)
    assert node.mission.finished
    assert send(ros, '4F', '402').accepted
    keep_alive.cancel()
