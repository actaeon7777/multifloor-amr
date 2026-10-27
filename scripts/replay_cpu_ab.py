#!/usr/bin/env python3
"""Per-node CPU of two code versions on the same recorded real-vehicle bag.

Usage (stack stopped; one variant at a time, never while driving):
    git worktree add --detach /path/to/baseline <commit>
    (cd /path/to/baseline && colcon build --packages-select amr_depth_filter \
        --cmake-args -DCMAKE_BUILD_TYPE=Release)
    source scripts/env.sh
    python3 scripts/replay_cpu_ab.py baseline BAG OUT.json --baseline /path/to/baseline
    python3 scripts/replay_cpu_ab.py current  BAG OUT.json

Runs scan_body_filter, time_health, elevator_monitor, floor_mission_manager,
web_ui (+ a 5 Hz browser-like poller) and depth_persistence_filter on the
isolated domain 151 (localhost only) while the bag plays at 1x, and records
each process's average CPU over a 150 s window (1 core = 100 %).
Nodes that need CAN (can_telemetry, cmd_vel_to_can) or a live Nav2/AMCL
(navigation_startup) are not covered; see bench_map_quality_ab.py.
First used 2026-10-25 (bags/2026-10-25/023737_depth_box, round-1 optimisation).
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from urllib.request import urlopen

import psutil

WS = Path(__file__).resolve().parent.parent
parser = argparse.ArgumentParser()
parser.add_argument('variant', choices=['baseline', 'current'])
parser.add_argument('bag')
parser.add_argument('out', type=Path)
parser.add_argument('--baseline', type=Path, help='checkout of the older commit (required for baseline)')
options = parser.parse_args()
variant, bag, out = options.variant, options.bag, options.out
if variant == 'baseline' and not options.baseline:
    parser.error('--baseline is required for the baseline variant')
OLD = options.baseline
root = OLD if variant == 'baseline' else WS
PKGS = ['amr_stack', 'amr_control', 'amr_elevator', 'amr_web_ui', 'amr_can_bridge', 'amr_remote_bridge']

env = dict(os.environ, ROS_DOMAIN_ID='151', ROS_LOCALHOST_ONLY='1', PYTHONDONTWRITEBYTECODE='1')
if variant == 'baseline':
    env['PYTHONPATH'] = ':'.join(str(OLD / 'src/amr_system' / p) for p in PKGS) + ':' + env.get('PYTHONPATH', '')
cfg = WS / 'src/amr_system/amr_stack/config'
port = 18471 if variant == 'baseline' else 18472
sim = ['-p', 'use_sim_time:=true']


def py(module, *ros):
    return [sys.executable, '-c', f'import sys; from {module} import main; sys.exit(main())', *ros]


nodes = {
    'scan_body_filter': py('amr_stack.scan_body_filter', '--ros-args', '--params-file',
                           str(cfg / 'scan_body_filter.yaml'), *sim),
    'time_health': py('amr_control.time_health', '--ros-args', *sim),
    'elevator_monitor': py('amr_elevator.monitor_node', '--ros-args', '-r', '__node:=elevator_monitor',
                           '-p', f'floors_config:={cfg / "floors.yaml"}', '-p', 'initial_floor:=4F', *sim),
    'floor_mission': py('amr_elevator.mission_node', '--ros-args', '-r', '__node:=floor_mission_manager',
                        '-p', f'floors_config:={cfg / "floors.yaml"}', '-p', 'require_estop:=false', *sim),
    'web_ui': py('amr_web_ui.server', '--host', '127.0.0.1', '--port', str(port), '--ros-args',
                 '-p', 'require_time_health:=true', '-p', 'require_estop:=false', *sim),
    'depth_filter': [str(root / 'install/amr_depth_filter/lib/amr_depth_filter/depth_persistence_filter'),
                     '--ros-args', '-p', 'min_height:=0.06', '-p', 'max_height:=1.0', '-p', 'max_range:=3.0',
                     '-p', 'frames:=3', '-p', 'cell_size:=0.05', '-p', 'neighbor_cells:=1', *sim],
}
TOPICS = ['/scan_raw', '/tf', '/tf_static', '/wheel/odometry', '/imu/data_raw', '/odometry/filtered',
          '/amcl_pose', '/map', '/camera/depth/points', '/local_costmap/costmap',
          '/local_costmap/costmap_updates', '/global_costmap/costmap', '/global_costmap/costmap_updates',
          '/local_costmap/published_footprint', '/plan', '/local_plan', '/localization/status',
          '/control_mux/mode', '/navigation/status', '/clock']

logs = out.parent / f'{out.stem}_logs'
logs.mkdir(parents=True, exist_ok=True)
procs = {}
for name, cmd in nodes.items():
    procs[name] = subprocess.Popen(cmd, env=env, cwd=root, stdout=(logs / f'{name}.log').open('w'),
                                   stderr=subprocess.STDOUT, start_new_session=True)
# A subscriber on the filtered cloud, as the costmaps would be (the new filter skips work without one).
procs['_sink'] = subprocess.Popen(['ros2', 'topic', 'hz', '/camera/depth/points_filtered'], env=env,
                                  stdout=(logs / 'sink.log').open('w'), stderr=subprocess.STDOUT,
                                  start_new_session=True)
time.sleep(6)
play = subprocess.Popen(['ros2', 'bag', 'play', bag, '--clock', '100', '--topics', *TOPICS[:-1]],
                        env=env, stdout=(logs / 'play.log').open('w'), stderr=subprocess.STDOUT,
                        start_new_session=True)

stop = threading.Event()
polls = {'state': 0, 'png': 0, 'state_bytes': 0, 'errors': 0}


def poller():
    base = f'http://127.0.0.1:{port}'
    versions = {}
    while not stop.is_set():
        try:
            body = urlopen(base + '/api/state', timeout=2).read()
            polls['state'] += 1
            polls['state_bytes'] += len(body)
            layers = json.loads(body).get('layers', {})
            for layer in ('map', 'local_costmap'):
                info = layers.get(layer) or {}
                if info.get('version') is not None and info.get('version') != versions.get(layer):
                    urlopen(f'{base}/api/layers/{layer}.png', timeout=5).read()
                    versions[layer] = info['version']
                    polls['png'] += 1
        except Exception:
            polls['errors'] += 1
        stop.wait(.2)


threading.Thread(target=poller, daemon=True).start()
time.sleep(15)  # warm-up: map, TF, persistence history
tracked = {name: psutil.Process(p.pid) for name, p in procs.items() if not name.startswith('_')}
start = {name: sum(p.cpu_times()[:2]) for name, p in tracked.items()}
t0 = time.monotonic()
window = 150
while time.monotonic() - t0 < window and play.poll() is None:
    time.sleep(1)
elapsed = time.monotonic() - t0
result = {}
for name, p in tracked.items():
    try:
        result[name] = round(100 * (sum(p.cpu_times()[:2]) - start[name]) / elapsed, 1)
    except psutil.Error:
        result[name] = None
result['_alive'] = {n: procs[n].poll() is None for n in procs}
stop.set()
for p in [play, *procs.values()]:
    try:
        os.killpg(p.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
time.sleep(3)
for p in [play, *procs.values()]:
    try:
        os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
result['_seconds'] = round(elapsed, 1)
result['_polls'] = polls
out.write_text(json.dumps(result, indent=1))
print(json.dumps(result, indent=1))
