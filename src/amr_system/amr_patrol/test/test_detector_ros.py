"""Opt-in synthetic map/TF/camera integration; no navigation or hardware nodes."""
import json
import os
from pathlib import Path
import time
from uuid import uuid4

import pytest

if os.environ.get('AMR_RUN_PATROL_TESTS') != '1':
    pytest.skip('Opt in with AMR_RUN_PATROL_TESTS=1, isolated domain 198', allow_module_level=True)
assert os.environ.get('ROS_DOMAIN_ID') == '198'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'

from geometry_msgs.msg import TransformStamped
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
import yaml

from amr_patrol.detector_node import MotionDetectorNode
from amr_stack.floor_config import load_floor_config


class Harness:
    def __init__(self, tmp_path):
        path = tmp_path / 'patrol.yaml'
        path.write_text(yaml.safe_dump(dict(schema_version=1, routes={
            'test': dict(floor='4F', end='stay', waypoints=[dict(x=1, y=1)])})))
        building_path = Path(__file__).resolve().parents[2] / 'amr_stack/config/floors.yaml'
        self.node = MotionDetectorNode(building=load_floor_config(building_path), parameter_overrides=[
            Parameter('config', value=str(path)),
            Parameter('spool_dir', value=str(tmp_path / 'queue')),
            Parameter('image_dir', value=str(tmp_path / 'images')),
            Parameter('detect_period', value=.1),
            Parameter('warmup_s', value=0.),  # Pure tests exercise the default five-second warm-up.
        ])
        self.probe = Node('motion_detector_probe')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.probe)
        self.baseline_subscriptions = len(list(self.node.subscriptions))
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.probe.create_publisher(String, '/patrol/status', retained)
        self.map = self.probe.create_publisher(OccupancyGrid, '/map', retained)
        self.costmap = self.probe.create_publisher(OccupancyGrid, '/global_costmap/costmap', retained)
        self.updates = self.probe.create_publisher(
            OccupancyGridUpdate, '/global_costmap/costmap_updates', 10)
        self.camera = self.probe.create_publisher(CompressedImage, '/camera/color/image_raw/compressed', 1)
        self.event = self.probe.create_publisher(String, '/robot/event', 10)
        self.events, self.statuses = [], []
        self.probe.create_subscription(String, '/robot/event',
                                       lambda m: self.events.append(json.loads(m.data)), 10)
        self.probe.create_subscription(String, '/patrol/detector_status',
                                       lambda m: self.statuses.append(json.loads(m.data)), retained)
        self.tf = StaticTransformBroadcaster(self.probe)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'base_footprint'
        transform.header.stamp = self.probe.get_clock().now().to_msg()
        transform.transform.translation.x = transform.transform.translation.y = 1.
        transform.transform.rotation.w = 1.
        self.tf.sendTransform(transform)
        self.until(lambda: self.status.get_subscription_count() > 0
                   and self.event.get_subscription_count() >= 2 and bool(self.statuses))

    def until(self, predicate, timeout=6.):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, (self.events, self.statuses[-1:])
            self.executor.spin_once(timeout_sec=.01)

    def spin(self, seconds=.3):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.01)

    def activate(self, active=True, phase='NAV', floor='4F'):
        self.status.publish(String(data=json.dumps(dict(active=active, phase=phase, floor=floor))))
        expected = active and phase in ('NAV', 'DWELL', 'RETURN')
        self.until(lambda: self.node.active == expected and self.node.floor == floor)
        if expected:
            self.until(lambda: all(pub.get_subscription_count() > 0
                                   for pub in (self.map, self.costmap, self.camera, self.updates)))

    def grid(self, x=None):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.probe.get_clock().now().to_msg()
        grid.info.width = grid.info.height = 200
        grid.info.resolution = .05
        grid.info.origin.orientation.w = 1.
        data = np.zeros((200, 200), dtype=np.int8)
        if x is not None:
            data[60:70, x:x + 10] = 100
        grid.data = data.ravel().tolist()
        return grid

    def first_frame(self):
        self.map.publish(self.grid())
        self.costmap.publish(self.grid(60))
        self.until(lambda: self.node.processed_version == 1)
        assert not self.node.coverage.any() and not self.node.detector.tracks
        self.patch(60)
        self.until(lambda: self.node.processed_version == 2 and len(self.node.detector.tracks) == 1)
        self.spin(.4)  # A later 0.5 m displacement must respect the 1.5 m/s speed bound.

    def patch(self, x):
        message = OccupancyGridUpdate()
        message.header.frame_id = 'map'
        message.width = message.height = 200
        message.data = self.grid(x).data
        self.updates.publish(message)

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.probe.destroy_node()


@pytest.fixture
def harness(tmp_path):
    rclpy.init()
    instance = None
    try:
        instance = Harness(tmp_path)
        yield instance
    finally:
        if instance is not None:
            instance.close()
        if rclpy.ok():
            rclpy.shutdown()


def test_motion_snapshot_single_spool_and_subscription_lifecycle(harness):
    h = harness
    assert not h.node.active and not h.node.sensor_subscriptions
    assert h.node.tf is None and h.node.tf_listener is None
    assert not any(s.topic_name in ('/tf', '/tf_static') for s in h.node.subscriptions)
    h.activate()
    assert len(list(h.node.subscriptions)) == h.baseline_subscriptions + 6
    assert h.node.tf is not None and h.node.tf_listener is not None
    h.first_frame()
    # Synthetic JPEG bytes: the adapter stores compressed bytes without decoding.
    jpeg = b'\xff\xd8synthetic-camera-frame\xff\xd9'
    h.camera.publish(CompressedImage(format='jpeg', data=jpeg))
    h.until(lambda: h.node.camera_fresh())
    h.patch(70)
    h.until(lambda: len(h.events) == 1 and len(list(h.node.spool.spool_dir.glob('*.json'))) == 1)
    event, = h.events
    assert event['kind'] == 'security.motion' and event['severity'] == 'alert'
    assert event['floor'] == '4F'
    assert event['pose'] == pytest.approx([3.75, 3.25, 0.])
    assert event['extra']['displacement_m'] == pytest.approx(.5)
    assert event['extra']['robot_pose'] == pytest.approx([1., 1., 0.])
    image = Path(event['image_path'])
    assert image.is_absolute() and image.read_bytes() == jpeg
    assert image.name.endswith(f'-{event["id"]}.jpg')
    spooled, = h.node.spool.spool_dir.glob('*.json')
    assert json.loads(spooled.read_text()) == event
    h.spin(.4)
    assert len(h.events) == 1
    assert len(list(h.node.spool.spool_dir.glob('*.json'))) == 1
    # Timer callbacks alone must not count as additional observed frames.
    assert next(iter(h.node.detector.tracks.values())).frames == 2
    h.until(lambda: any(s['detections'] == 1 and s['camera_fresh'] for s in h.statuses))
    h.activate(False, 'DONE')
    assert len(list(h.node.subscriptions)) == h.baseline_subscriptions
    assert not h.node.sensor_subscriptions
    assert h.node.tf is None and h.node.tf_listener is None
    assert not any(s.topic_name in ('/tf', '/tf_static') for s in h.node.subscriptions)
    assert h.node.costmap is None and h.node.detector.static is None and h.node.camera is None
    h.until(lambda: not h.statuses[-1]['active'])
    # Patrol lifecycle events still spool after sensors were unsubscribed.
    patrol = dict(id=uuid4().hex, kind='patrol.finished', severity='info', text='완료',
                  floor='4F', room='', pose=None, image_path=None, stamp=12.)
    h.event.publish(String(data=json.dumps(patrol)))
    h.until(lambda: len(list(h.node.spool.spool_dir.glob('*.json'))) == 2)
    saved = [json.loads(p.read_text()) for p in h.node.spool.spool_dir.glob('*.json')]
    saved_patrol, = [p for p in saved if p['id'] == patrol['id']]
    assert saved_patrol.pop('wall_time') > 0
    assert saved_patrol.pop('extra') is None
    assert saved_patrol == patrol


def test_patch_no_camera_phase_gating_and_floor_refresh(harness):
    h = harness
    h.activate(True, 'IDLE')
    assert len(list(h.node.subscriptions)) == h.baseline_subscriptions
    h.activate(True, 'DWELL')
    h.first_frame()
    update = OccupancyGridUpdate()
    update.header.frame_id = 'map'
    update.x, update.y, update.width, update.height = 60, 60, 20, 10
    patch = np.zeros((10, 20), dtype=np.int8)
    patch[:, 10:] = 100
    update.data = patch.ravel().tolist()
    h.updates.publish(update)
    h.until(lambda: len(h.events) == 1 and bool(list(h.node.spool.spool_dir.glob('*.json'))))
    assert h.events[0]['image_path'] is None
    old_subscriptions = tuple(h.node.sensor_subscriptions)
    h.activate(True, 'RETURN', '3F')
    assert h.node.sensor_subscriptions and all(new is not old for new in h.node.sensor_subscriptions
                                              for old in old_subscriptions)
    h.until(lambda: h.node.detector.static is not None)  # Transient-local map redelivery.
    h.until(lambda: h.node.coverage is not None)
    assert not h.node.coverage.any() and not h.node.detector.tracks
    h.activate(False, 'ABORTED', '3F')
    assert len(list(h.node.subscriptions)) == h.baseline_subscriptions
    assert h.node.tf_listener is None and h.node.tf is None
