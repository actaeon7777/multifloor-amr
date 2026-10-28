"""Opt-in ROS test: elevator_monitor through a full 4F -> 3F ride with synthetic sensors."""
import functools
import json
import math
import os
import time
from types import SimpleNamespace

import pytest

if os.environ.get('AMR_RUN_ELEVATOR_MONITOR_TESTS') != '1':
    pytest.skip('Set AMR_RUN_ELEVATOR_MONITOR_TESTS=1 in domain 195', allow_module_level=True)

from amr_elevator_interfaces.msg import (
    ElevatorBeacon, ElevatorCommand, ElevatorDoorState, FloorEvent, FloorState,
)
from amr_elevator_interfaces.srv import SetCurrentFloor
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from amr_elevator.geometry import ray_segment_distance
from amr_elevator.monitor_node import ElevatorMonitor
from amr_stack.floor_config import load_floor_config

COUNT = 720


@functools.lru_cache(maxsize=None)
def world(hall_open, car_open):
    """4F elevator walls as segments from floors.yaml (hall door plane x=9.975, car to the west).

    The hall wall face sits ``hall_recess_m`` east of the door plane, the car's
    inner front wall ``threshold_depth_m`` west of it, with side walls in between.
    """
    g = load_floor_config().elevator_geometry('4F', 'E1')
    hx = g.hall_entry[0][0]                                  # hall wall face
    dx = g.hall_door[0][0]                                   # hall door plane (map wall)
    cx = g.car_door[0][0]                                    # car front wall
    bx = min(p[0] for p in g.car_polygon)                    # car back wall
    door_lo, door_hi = sorted(p[1] for p in g.hall_door)     # door span
    car_lo, car_hi = min(p[1] for p in g.car_polygon), max(p[1] for p in g.car_polygon)  # car sides
    segments = [
        ((hx, door_hi), (hx, 8.)), ((hx, door_lo), (hx, -2.)),         # hall wall face
        ((cx, car_hi), (cx, door_hi)), ((cx, door_lo), (cx, car_lo)),  # car front wall
        ((bx, car_lo), (bx, car_hi)),                                  # car back wall
        ((bx, car_lo), (cx, car_lo)), ((bx, car_hi), (cx, car_hi)),    # car sides
        ((cx, door_lo), (hx, door_lo)), ((cx, door_hi), (hx, door_hi)),  # recess + threshold sides
    ]
    if not hall_open:
        segments.append(((dx, door_lo), (dx, door_hi)))
    if not car_open:
        segments.append(((cx, door_lo), (cx, door_hi)))
    return segments


def scan(pose, doors_open):
    ranges = []
    for i in range(COUNT):
        angle = pose[2] - math.pi + i * 2 * math.pi / COUNT
        hits = [d for a, b in world(doors_open, doors_open)
                if (d := ray_segment_distance(pose[:2], angle, a, b)) is not None]
        ranges.append(min(hits) if hits else 8.)
    message = LaserScan(angle_min=-math.pi, angle_increment=2 * math.pi / COUNT,
                        range_min=.05, range_max=30., ranges=ranges)
    message.header.frame_id = 'laser'
    return message


@pytest.fixture
def ros():
    assert os.environ['ROS_DOMAIN_ID'] == '195'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rclpy.init()
    executor = SingleThreadedExecutor()
    probe = rclpy.create_node('monitor_test_probe')
    executor.add_node(probe)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    states, events, doors = [], [], []
    probe.create_subscription(FloorState, 'floor/state', states.append, latched)
    probe.create_subscription(FloorEvent, 'floor/events', events.append, 20)
    probe.create_subscription(ElevatorDoorState, 'elevator/door', doors.append, 20)
    pubs = SimpleNamespace(
        scan=probe.create_publisher(LaserScan, 'scan', qos_profile_sensor_data),
        odom=probe.create_publisher(Odometry, 'odometry/filtered', 10),
        status=probe.create_publisher(String, 'localization/status', latched),
        beacon=probe.create_publisher(ElevatorBeacon, 'elevator/beacon', 10),
        command=probe.create_publisher(ElevatorCommand, 'elevator/command', 10))
    static = StaticTransformBroadcaster(probe)
    transforms = []
    for parent, child in (('map', 'odom'), ('base_footprint', 'laser')):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.rotation.w = 1.
        transforms.append(t)
    static.sendTransform(transforms)
    set_floor = probe.create_client(SetCurrentFloor, 'floor/set_current')

    def spin(duration, until=None):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
            if until is not None and until():
                return True
        return until is None

    yield SimpleNamespace(executor=executor, probe=probe, pubs=pubs, spin=spin, states=states,
                          events=events, doors=doors, set_floor=set_floor)
    executor.shutdown()
    rclpy.shutdown()


def test_full_ride_and_operator_override(ros):
    node = ElevatorMonitor(building=load_floor_config())
    ros.executor.add_node(node)
    seq = {'n': 0}

    def place(pose, doors_open, ready):
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x, odom.pose.pose.position.y = pose[0], pose[1]
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        ros.pubs.odom.publish(odom)
        ros.pubs.status.publish(String(data=json.dumps(
            {'state': 'READY' if ready else 'SUSPENDED', 'ready': ready})))
        for _ in range(3):
            ros.pubs.scan.publish(scan(pose, doors_open))
            ros.spin(.12)

    def beacon(code, event=ElevatorBeacon.EVENT_PRESENCE, location=ElevatorBeacon.LOCATION_HALL, ack=0):
        seq['n'] += 1
        message = ElevatorBeacon(event=event, location=location, floor_code=code,
                                 sequence=seq['n'] & 0xff, ack_sequence=ack, rssi_dbm=-55, source='sim')
        message.header.stamp = node.get_clock().now().to_msg()
        ros.pubs.beacon.publish(message)
        ros.spin(.2)

    def names():
        return [e.event for e in ros.events]

    hall = (10.575, 3.31, math.pi)
    place(hall, False, True)
    beacon(4)
    assert ros.spin(2., lambda: ros.states and ros.states[-1].confidence == 'BEACON')
    assert ros.doors[-1].state == ElevatorDoorState.CLOSED

    place(hall, True, True)
    assert ros.doors[-1].state == ElevatorDoorState.OPEN
    car = load_floor_config().elevator_geometry('4F', 'E1').car_pose[:2] + (math.pi,)
    place(car, True, False)  # localization suspended inside the car
    assert ros.spin(2., lambda: ros.states[-1].zone == FloorState.ZONE_IN_ELEVATOR)

    ros.pubs.command.publish(ElevatorCommand(command=ElevatorCommand.CMD_CAR_FLOOR, floor_code=3, sequence=40))
    ros.spin(.2)
    beacon(0, ElevatorBeacon.EVENT_BUTTON_ACK, ElevatorBeacon.LOCATION_CAR, ack=40)
    for _ in range(5):
        place(car, False, False)
    assert ros.spin(3., lambda: ros.states[-1].confidence == 'TRANSIT')
    assert ros.states[-1].transit_to == '3F'
    beacon(4)  # still heard while travelling: ignored
    assert ros.states[-1].confidence == 'TRANSIT'

    place(car, True, False)
    beacon(3)
    assert ros.spin(2., lambda: ros.states[-1].floor == '3F' and ros.states[-1].confidence == 'BEACON')
    exit_pose = (10.775, 3.31, 0.)
    place((9.6, 3.31, 0.), True, False)
    place(exit_pose, True, False)
    assert ros.spin(2., lambda: ros.states[-1].zone == FloorState.ZONE_OUTSIDE)
    place(exit_pose, True, True)
    assert ros.spin(2., lambda: ros.states[-1].confidence == 'CONFIRMED')
    assert names()[:6] == ['FLOOR_BEACON', 'ENTERED_ELEVATOR', 'DEPARTED', 'ARRIVED',
                           'EXITED_ELEVATOR', 'FLOOR_CONFIRMED']

    # A warning event after info events must not crash the node (rclpy call-site severity).
    beacon(5)
    assert ros.spin(2., lambda: ros.states[-1].floor_conflict)
    assert 'FLOOR_CONFLICT' in names()
    assert ros.set_floor.wait_for_service(timeout_sec=2.)
    future = ros.set_floor.call_async(SetCurrentFloor.Request(floor='5F'))
    assert ros.spin(2., future.done)
    assert future.result().success
    assert ros.spin(2., lambda: ros.states[-1].floor == '5F')
    future = ros.set_floor.call_async(SetCurrentFloor.Request(floor='9F'))
    assert ros.spin(2., future.done)
    assert not future.result().success
    node.destroy_node()
