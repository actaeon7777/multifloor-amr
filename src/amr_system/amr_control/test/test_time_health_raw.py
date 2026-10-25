"""CDR stamp and freshness equivalence without creating ROS nodes."""
import inspect
import struct
from unittest.mock import Mock, call

import pytest
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage

from amr_control import time_health
from amr_control.time_health import TimeHealth


@pytest.mark.parametrize('typ', [LaserScan, Odometry, Imu])
@pytest.mark.parametrize('sec,nanosec', [(1, 0), (17, 999999999), (1729000000, 123456789)])
def test_raw_header_stamp_matches_deserialized_time(typ, sec, nanosec):
    message = typ()
    message.header.stamp.sec, message.header.stamp.nanosec = sec, nanosec
    message.header.frame_id = 'sensor_frame'
    if typ is LaserScan:
        message.ranges = [1.25] * 1080
        message.intensities = [2.] * 1080
    data = serialize_message(message)
    parsed_sec, parsed_nanosec = TimeHealth.header_stamp(data)
    restored = deserialize_message(data, typ)
    assert parsed_sec * 10**9 + parsed_nanosec == Time.from_msg(restored.header.stamp).nanoseconds


@pytest.mark.parametrize('little', [True, False])
def test_header_stamp_endianness_and_truncated_input(little):
    data = (b'\x00\x01\x00\x00' if little else b'\x00\x00\x00\x00')
    data += struct.pack('<iI' if little else '>iI', 7, 999999999)
    assert TimeHealth.header_stamp(memoryview(data)) == (7, 999999999)
    for length in range(12):
        with pytest.raises(ValueError):
            TimeHealth.header_stamp(data[:length])


def make_health():
    node = object.__new__(TimeHealth)
    node.future, node.max_age, node.max_ages = .1, .75, {'depth': 1.}
    node.samples = {}
    node.now = lambda: 100.
    return node


@pytest.mark.parametrize('kind,typ', [('scan', LaserScan), ('wheel', Odometry), ('imu', Imu)])
def test_raw_observer_preserves_freshness_and_ordering(kind, typ, monkeypatch):
    raw, original = make_health(), make_health()
    callback = raw.raw_observer(kind)
    assert len(inspect.signature(callback).parameters) == 1
    for index, stamp in enumerate((0., 98., 100.2, 99.5, 99.4, 99.5, 99.9, 100.05)):
        monkeypatch.setattr(time_health.time, 'monotonic', lambda: float(index))
        message = typ()
        message.header.stamp = Time(nanoseconds=round(stamp*1e9)).to_msg()
        callback(serialize_message(message))
        original.observe(kind, message)
        assert raw.samples == original.samples
    before = dict(raw.samples)
    callback(b'\x00\x01')
    assert raw.samples == before


def test_tf_still_checks_all_transforms_and_filtered_odom_keeps_twist():
    node = make_health()
    node.observe = Mock(return_value=True)
    ignored = TransformStamped()
    ignored.header.frame_id, ignored.child_frame_id = 'map', 'odom'
    matches = []
    for stamp in (99, 100):
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'odom', 'base_footprint'
        transform.header.stamp.sec = stamp
        matches.append(transform)
    node.on_tf(TFMessage(transforms=[ignored, *matches]))
    assert node.observe.call_args_list == [call('odom_tf', transform) for transform in matches]
    message = Odometry()
    message.twist.twist.linear.x, message.twist.twist.angular.z = .3, -.2
    node.on_odom(message)
    assert node.speed == (.3, -.2)
    node.observe.return_value = False
    message.twist.twist.linear.x = 10.
    node.on_odom(message)
    assert node.speed == (.3, -.2)


def test_subscription_modes_and_tick_period(monkeypatch):
    # Exercise constructor wiring with all Node entry points replaced; no DDS context.
    monkeypatch.setattr(time_health.Node, '__init__', lambda *a, **k: None)
    monkeypatch.setattr(TimeHealth, 'declare_parameter', lambda self, key, value: Mock(value=value))
    monkeypatch.setattr(TimeHealth, 'get_parameter', lambda *a: Mock(value=False))
    for method in ('create_publisher', 'create_service', 'create_timer', 'create_subscription'):
        monkeypatch.setattr(TimeHealth, method, Mock())
    node = TimeHealth()
    calls = {call.args[1]: call for call in node.create_subscription.call_args_list}
    for topic in ('/scan', '/wheel/odometry', '/imu/data_raw'):
        assert calls[topic].kwargs == {'raw': True}
        assert len(inspect.signature(calls[topic].args[2]).parameters) == 1
        assert calls[topic].args[3] is time_health.qos_profile_sensor_data
    assert calls['/tf'].kwargs == {} and calls['/tf'].args[3] == 100
    assert calls['/odometry/filtered'].kwargs == {}
    assert node.create_timer.call_args.args[0] == .05
