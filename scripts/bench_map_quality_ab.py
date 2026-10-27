#!/usr/bin/env python3
"""Old vs new MapQuality.score and tracking ray-pose interpolation on real bag scans.

Usage: bench_map_quality_ab.py BAG --baseline /path/to/older/checkout
Loads /map, /scan and /amcl_pose from the bag, scores every 4th scan at its AMCL
pose with both implementations, checks the results are identical and times them.
"""
import dataclasses
import importlib.util
import argparse
import math
import time
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WS = Path(__file__).resolve().parent.parent
parser = argparse.ArgumentParser()
parser.add_argument('bag')
parser.add_argument('--baseline', type=Path, required=True, help='checkout of the older commit')
options = parser.parse_args()


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


old = load('lq_old', options.baseline / 'src/amr_system/amr_stack/amr_stack/localization_quality.py')
new = load('lq_new', WS / 'src/amr_system/amr_stack/amr_stack/localization_quality.py')

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=options.bag, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
reader.set_filter(rosbag2_py.StorageFilter(topics=['/map', '/scan', '/amcl_pose']))
types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
grid = None
scans, pose = [], None
while reader.has_next():
    topic, data, _ = reader.read_next()
    msg = deserialize_message(data, types[topic])
    if topic == '/map':
        grid = msg
    elif topic == '/amcl_pose':
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w))
    elif pose is not None:
        scans.append((msg, pose))
scans = scans[::4]
info = grid.info
q = info.origin.orientation
origin = (info.origin.position.x, info.origin.position.y, 2 * math.atan2(q.z, q.w))
raw = np.asarray(grid.data, dtype=np.int16)
maps = {name: m.MapQuality(info.width, info.height, info.resolution, *origin, raw)
        for name, m in (('old', old), ('new', new))}

LASER_YAW = math.pi  # S2 mounted turned 180 deg, no planar offset (URDF laser_joint)
args = []
for scan, (x, y, yaw) in scans:
    laser = (x, y, yaw + LASER_YAW)
    common = dict(angle_min=scan.angle_min, angle_increment=scan.angle_increment,
                  range_min=scan.range_min, range_max=scan.range_max, laser_pose=laser,
                  max_beams=360, max_range=30.0)
    idx, _, _ = new.MapQuality.sampled_rays(scan.ranges, scan.angle_min, scan.angle_increment,
                                            scan.range_min, scan.range_max, 360, 30.0)
    # Small per-ray motion, as tracking_ray_poses produces while driving.
    t = np.linspace(0, 1, len(idx))[:, None]
    rays = np.asarray(laser) + t * np.array([.03, .01, .02])
    args.append((list(scan.ranges), np.asarray(scan.ranges), common, rays))

mismatch = 0
timing = {}
for name in ('old', 'new'):
    m = maps[name]
    started = time.perf_counter()
    results = []
    for listed, array, common, rays in args:
        ranges = listed if name == 'old' else array
        results.append(m.score(ranges=ranges, ray_poses=rays, **common))
    timing[name] = (time.perf_counter() - started) / len(args) * 1000
    timing[name + '_results'] = results
for a, b in zip(timing['old_results'], timing['new_results']):
    mismatch += dataclasses.astuple(a) != dataclasses.astuple(b)

# tracking_ray_poses interpolation: per-ray loop vs arrays (360 rays, 10 ms knots over 0.1 s).
knot_times = np.append(np.arange(0, .1, .01), .1)
knots = np.cumsum(np.random.default_rng(0).normal(0, .005, (len(knot_times), 3)), axis=0)
unwrapped = np.unwrap(knots[:, 2])
ray_times = np.linspace(0, .1, 360)
correction = (1.0, 2.0, .3)


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c*b[0] - s*b[1], a[1] + s*b[0] + c*b[1], math.atan2(math.sin(a[2]+b[2]), math.cos(a[2]+b[2])))


def loop():
    return [compose(correction, (float(np.interp(t, knot_times, knots[:, 0])),
                                 float(np.interp(t, knot_times, knots[:, 1])),
                                 float(np.interp(t, knot_times, unwrapped)))) for t in ray_times]


def vector():
    x = np.interp(ray_times, knot_times, knots[:, 0])
    y = np.interp(ray_times, knot_times, knots[:, 1])
    h = correction[2] + np.interp(ray_times, knot_times, unwrapped)
    c, s = math.cos(correction[2]), math.sin(correction[2])
    return np.column_stack((correction[0] + c*x - s*y, correction[1] + s*x + c*y, np.arctan2(np.sin(h), np.cos(h))))


assert np.allclose(np.asarray(loop()), vector(), atol=1e-12)
reps = 200
t0 = time.perf_counter(); [loop() for _ in range(reps)]; t_loop = (time.perf_counter() - t0) / reps * 1000
t0 = time.perf_counter(); [vector() for _ in range(reps)]; t_vec = (time.perf_counter() - t0) / reps * 1000

print(f'scans scored: {len(args)}  map {info.width}x{info.height}')
print(f'MapQuality.score  old {timing["old"]:.2f} ms  new {timing["new"]:.2f} ms  '
      f'(x{timing["old"]/timing["new"]:.1f})  results differing: {mismatch}')
print(f'ray pose interp   old {t_loop:.2f} ms  new {t_vec:.3f} ms  (x{t_loop/t_vec:.0f})')
