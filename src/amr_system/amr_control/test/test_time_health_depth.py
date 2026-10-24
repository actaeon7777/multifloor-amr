"""time_health depth watch: raw PointCloud2 stamps gate AUTO only when enabled."""
import math
import struct
import time

import pytest
import rclpy
from rclpy.parameter import Parameter

from amr_control.time_health import TimeHealth


@pytest.fixture
def ros():
    context = rclpy.Context()
    rclpy.init(context=context)
    nodes = []

    def make(**overrides):
        node = TimeHealth(context=context, parameter_overrides=[Parameter(k, value=v) for k, v in overrides.items()])
        nodes.append(node)
        return node
    yield make
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown(context=context)


def cdr_header_message(sec, nanosec, little=True):
    """Serialized bytes of a message that starts with std_msgs/Header (CDR, 4-byte encapsulation)."""
    encapsulation = b'\x00\x01\x00\x00' if little else b'\x00\x00\x00\x00'
    return encapsulation + struct.pack('<iI' if little else '>iI', sec, nanosec) + b'\x00' * 32


def feed_basics(node, now):
    for kind in ('scan', 'wheel', 'imu', 'odom', 'odom_tf'):
        node.observe_stamp(kind, now)


def test_header_stamp_reads_both_byte_orders():
    assert TimeHealth.header_stamp(cdr_header_message(1700000000, 250_000_000)) == (1700000000, 250_000_000)
    assert TimeHealth.header_stamp(cdr_header_message(7, 9, little=False)) == (7, 9)
    with pytest.raises(ValueError):
        TimeHealth.header_stamp(b'\x00\x01\x00\x00\x01')


def test_depth_is_ignored_unless_watched(ros):
    node = ros()
    now = node.now()
    feed_basics(node, now)
    assert node.data_error(now, time.monotonic()) == ''
    assert 'depth' not in node.watched
    # Without the watch there is no depth subscription; a stale sample would not matter anyway.
    node.samples['depth'] = (now - 60, time.monotonic() - 60)
    assert node.data_error(now, time.monotonic()) == ''


def test_watched_depth_must_be_fresh(ros):
    node = ros(watch_depth=True, depth_max_age=1.)
    now = node.now()
    feed_basics(node, now)
    steady = time.monotonic()
    assert node.data_error(now, steady) == 'depth: missing or stale'
    sec, frac = divmod(now - .4, 1)
    node.on_depth_raw(cdr_header_message(int(sec), int(frac * 1e9)))
    assert 'depth' in node.samples
    assert node.data_error(now, steady) == ''
    # The depth allowance is longer than the 0.75 s default: 0.9 s old is still fine.
    node.samples['depth'] = (now - .9, steady - .9)
    assert node.data_error(now, steady) == ''
    node.samples['depth'] = (now - 1.2, steady - 1.2)
    assert node.data_error(now, steady) == 'depth: missing or stale'
    # Garbage bytes never count as a sample.
    node.samples.pop('depth')
    node.on_depth_raw(b'\x00\x01')
    assert 'depth' not in node.samples


def test_depth_threshold_must_be_positive(ros):
    with pytest.raises(ValueError):
        ros(watch_depth=True, depth_max_age=0.)
    assert math.isfinite(ros(depth_max_age=2.).depth_max_age)
