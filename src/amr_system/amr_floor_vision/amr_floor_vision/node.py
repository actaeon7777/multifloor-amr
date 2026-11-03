"""Run with the floorvision interpreter; JPEG callbacks never decode or infer."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from io import BytesIO
import json
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from amr_elevator_interfaces.msg import ElevatorDoorState, FloorArrivalConfirm, FloorState, FloorVisionGuess

from .embedder import DEFAULT_MODEL, DinoEmbedder
from .session import ArrivalSample, build_gallery


def workspace():
    if os.environ.get('AMR_WS'):
        return Path(os.environ['AMR_WS']).expanduser()
    for start in (Path(__file__).resolve(), Path.cwd()):
        for parent in (start, *start.parents):
            if (parent / 'src').is_dir():
                return parent
    raise RuntimeError('Workspace not found; set AMR_WS')


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class FloorVision(Node):
    def __init__(self):
        super().__init__('floor_vision')
        ws = workspace()
        param = lambda name, default: self.declare_parameter(name, default).value
        self.model_id = param('model_id', DEFAULT_MODEL)
        threads = int(param('threads', 2))
        self.frame_count = int(param('frames', 3))
        self.collect_s = float(param('collect_s', 1.2))
        self.min_score = float(param('min_score', .85))
        self.min_margin = float(param('min_margin', .01))
        self.dataset_dir = Path(param('dataset_dir', str(ws / 'datasets/floor_vision'))).expanduser()
        directories = param('gallery_dirs', [str(self.dataset_dir / 'arrival'),
                                             str(ws / 'datasets/floor_vision_20261102/arrival')])
        if threads < 1 or self.frame_count < 1 or not 0 < self.collect_s < float('inf'):
            raise ValueError('threads, frames and collect_s must be positive')
        from amr_stack.floor_config import load_floor_config
        building = load_floor_config()
        self.known_floors = set(building.floors) | set(building.pass_floors)
        # Startup errors propagate to main, which logs and exits nonzero.
        self.embedder = DinoEmbedder(self.model_id, threads)
        self.gallery = build_gallery([Path(p).expanduser() for p in directories],
                                     self.dataset_dir / 'gallery_cache.npz', self.embedder,
                                     self.model_id, self.known_floors)
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='floor-vision')
        self.jobs = []
        self.state = None
        self.door = ElevatorDoorState.UNKNOWN
        self.target_floor = ''
        self.session = None
        self.capture = None
        self.publisher = self.create_publisher(FloorVisionGuess, '/floor/vision_guess', 10)
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed',
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(ElevatorDoorState, '/elevator/door', self.on_door, 10)
        self.create_subscription(FloorState, '/floor/state', self.on_state, latched)
        self.create_subscription(FloorArrivalConfirm, '/floor/arrival_confirm', self.on_confirm, 10)
        self.create_subscription(String, '/floor_mission/status', self.on_mission, latched)
        self.create_timer(.1, self.tick)  # light on the Jetson's Python CPU budget
        self.get_logger().info(f'Gallery ready: {len(self.gallery.labels)} references')

    def submit(self, function, *args):
        future = self.pool.submit(function, *args)
        self.jobs.append(future)
        return future

    def on_mission(self, message):
        try:
            status = json.loads(message.data)
            self.target_floor = str(status.get('goal_floor') or '')
        except (ValueError, AttributeError):
            pass

    def on_state(self, message):
        self.state = message
        if self.session and message.zone == FloorState.ZONE_IN_ELEVATOR and message.awaiting_confirm:
            self.submit(self.session.bind, message.candidate_id, message.floor, message.candidate_source)
        elif self.session and not message.awaiting_confirm and message.candidate_id:
            if getattr(message, 'candidate_source', '') == 'VISION_AUTO' and message.zone == FloorState.ZONE_IN_ELEVATOR:
                # Decided by this node's own confident guess: no question was asked.
                self.submit(self.session.bind, message.candidate_id, message.floor, 'VISION_AUTO')
            # Label from the monitor's verdict, not from whichever answer arrived first.
            verified = message.floor if message.confidence == 'VERIFIED' else ''
            self.submit(self.session.resolve, message.candidate_id, verified)

    def on_door(self, message):
        opened = message.state == ElevatorDoorState.OPEN and self.door != ElevatorDoorState.OPEN
        self.door = message.state
        if not opened or self.state is None:
            return
        # A new opening invalidates publication from any earlier inference.
        self.capture = None
        self.session = None
        if self.state.zone != FloorState.ZONE_IN_ELEVATOR or self.state.confidence != 'TRANSIT':
            return
        self.session = ArrivalSample(self.gallery, self.dataset_dir, self.model_id,
                                     self.known_floors, self.target_floor or self.state.transit_to)
        self.capture = dict(session=self.session, edge=stamp_ns(message.header.stamp),
                            started=time.monotonic(), frames=deque(maxlen=self.frame_count),
                            future=None, published=False)

    def on_image(self, message):
        c = self.capture
        if c is None or c['future'] is not None or self.door != ElevatorDoorState.OPEN:
            return
        if time.monotonic() - c['started'] >= self.collect_s:
            self.start_inference(c)
            return
        stamp = stamp_ns(message.header.stamp)
        if stamp <= c['edge'] or (c['frames'] and stamp <= c['frames'][-1][0]):
            return
        c['frames'].append((stamp, message.header.stamp, bytes(message.data)))
        if len(c['frames']) >= self.frame_count:
            self.start_inference(c)

    def start_inference(self, capture):
        capture['future'] = self.submit(self.infer, capture['session'], list(capture['frames']))

    def infer(self, sample, frames):
        if sample is not self.session:
            return None  # a newer opening started: do not spend the single worker on this one
        from PIL import Image
        images, raw, stamps = [], [], []
        for _, stamp, data in frames:
            try:
                with Image.open(BytesIO(data)) as image:
                    images.append(image.convert('RGB'))
                raw.append(data)
                stamps.append(stamp)
            except (OSError, ValueError) as error:
                self.get_logger().warning(f'Invalid camera JPEG: {error}')
        embeddings = self.embedder.embed(images) if images else np.empty((0, 0), dtype=np.float32)
        guess = sample.complete(raw, embeddings, self.min_score, self.min_margin)
        return guess, sample.scores, stamps

    def on_confirm(self, message):
        if self.session:
            self.submit(self.session.confirm, message.candidate_id, message.floor,
                        message.accepted, message.actual_floor)

    def tick(self):
        for job in self.jobs[:]:
            if job.done():
                self.jobs.remove(job)
                if job.exception() is not None:
                    self.get_logger().error(f'Floor vision worker failed: {job.exception()}')
        c = self.capture
        if c is None or c['published']:
            return
        if c['future'] is None and (time.monotonic() - c['started'] >= self.collect_s
                                    or self.door != ElevatorDoorState.OPEN):
            self.start_inference(c)
        future = c['future']
        if future is None or not future.done():
            return
        c['published'] = True
        if future.exception() is not None or future.result() is None:
            return
        guess, scores, stamps = future.result()
        latency = time.monotonic() - c['started']
        self.get_logger().info(f'floor={guess["floor"] or "unknown"} score={guess["score"]:.4f} '
                               f'margin={guess["margin"]:.4f} per_floor={scores} latency={latency:.3f}s')
        if self.door != ElevatorDoorState.OPEN:
            return
        message = FloorVisionGuess(**guess, frames=len(stamps), model=self.model_id,
                                   reference_floors=sorted(set(self.gallery.labels)))
        if stamps:
            message.header.stamp = stamps[-1]
        self.publisher.publish(message)

    def destroy_node(self):
        self.pool.shutdown(wait=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FloorVision()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        rclpy.logging.get_logger('floor_vision').error(f'Floor vision failed: {error}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
