"""Callback tests using a plain harness, never a ROS node/context or real executor."""
from concurrent.futures import Future
from io import BytesIO
from types import SimpleNamespace as NS

import numpy as np
from PIL import Image
import pytest

pytest.importorskip('rclpy')
from amr_floor_vision.node import FloorVision
from amr_floor_vision.recognizer import Gallery


class Harness:
    on_door = FloorVision.on_door
    on_image = FloorVision.on_image
    on_state = FloorVision.on_state
    on_confirm = FloorVision.on_confirm
    start_inference = FloorVision.start_inference
    infer = FloorVision.infer
    tick = FloorVision.tick

    def __init__(self, root):
        self.gallery = Gallery()
        self.gallery.add(np.eye(2), ['3F', '4F'], ['a', 'b'])
        self.dataset_dir, self.model_id, self.known_floors = root, 'fake', {'3F', '4F'}
        self.target_floor, self.frame_count, self.collect_s = '4F', 3, 1.2
        self.min_score, self.min_margin = .85, .01
        self.state = NS(zone=2, confidence='TRANSIT', transit_to='4F')
        self.door, self.session, self.capture = 1, None, None
        self.jobs, self.sent, self.logs, self.embedded = [], [], [], []
        self.publisher = NS(publish=self.sent.append)
        self.embedder = NS(embed=self.embed)

    def get_logger(self):
        return NS(info=self.logs.append, warning=self.logs.append, error=self.logs.append)

    def embed(self, images):
        self.embedded.extend(images)
        return np.array([[1., 0.]] * len(images), dtype=np.float32)

    def submit(self, fn, *args):
        future = Future()
        future.set_result(fn(*args))
        self.jobs.append(future)
        return future


def stamp(ns):
    from builtin_interfaces.msg import Time
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def door(state, ns=1_000_000_000):
    return NS(state=state, header=NS(stamp=stamp(ns)))


def frame(ns):
    image = BytesIO()
    Image.new('RGB', (8, 8)).save(image, 'JPEG')
    return NS(header=NS(stamp=stamp(ns)), data=image.getvalue())


@pytest.fixture
def harness(tmp_path, monkeypatch):
    clock = [0.]
    monkeypatch.setattr('amr_floor_vision.node.time.monotonic', lambda: clock[0])
    return Harness(tmp_path), clock


def test_one_guess_per_open_and_only_new_stamped_frames(harness):
    node, _ = harness
    node.on_door(door(3))
    node.on_image(frame(1_000_000_000))  # exactly at OPEN is not after OPEN
    for ns in (1_100_000_000, 1_100_000_000, 1_200_000_000, 1_300_000_000):
        node.on_image(frame(ns))
    node.tick()
    node.on_door(door(3))
    node.on_image(frame(1_400_000_000))
    node.tick()
    assert len(node.sent) == 1 and len(node.embedded) == 3
    assert node.sent[0].floor == '3F' and node.sent[0].frames == 3
    assert node.sent[0].header.stamp == stamp(1_300_000_000)


@pytest.mark.parametrize('zone,confidence', [(1, 'TRANSIT'), (2, 'VERIFIED'), (2, 'ASSUMED')])
def test_no_hall_or_nontransit_capture(harness, zone, confidence):
    node, _ = harness
    node.state.zone, node.state.confidence = zone, confidence
    node.on_door(door(3))
    node.on_image(frame(1_100_000_000))
    node.tick()
    assert node.capture is None and not node.embedded and not node.sent


def test_deadline_partial_capture_and_empty_unknown(harness):
    node, clock = harness
    node.on_door(door(3))
    node.on_image(frame(1_100_000_000))
    clock[0] = 1.2
    node.on_image(frame(2_200_000_000))  # arrives outside the collection window
    node.tick()
    assert node.sent[-1].frames == 1
    node.on_door(door(1))
    node.on_door(door(3, 3_000_000_000))
    clock[0] = 2.4
    node.tick()
    assert node.sent[-1].frames == 0 and node.sent[-1].floor == ''


def test_closed_door_drops_publication_but_keeps_bound_sample(harness):
    node, clock = harness
    node.on_door(door(3))
    node.on_image(frame(1_100_000_000))
    node.on_state(NS(zone=2, awaiting_confirm=True, candidate_id=7, floor='4F', candidate_source='TARGET'))
    node.on_door(door(1))
    node.tick()
    assert not node.sent
    node.on_confirm(NS(candidate_id=7, floor='4F', accepted=False, actual_floor='3F'))
    assert node.session.record is None   # labelled only by the monitor's verdict
    node.on_state(NS(zone=2, awaiting_confirm=False, candidate_id=7, floor='3F', confidence='VERIFIED'))
    assert node.session.record['label'] == '3F'
    assert len(node.gallery.labels) == 3
