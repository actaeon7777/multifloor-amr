#!/usr/bin/env python3
"""Reproducible GUI/ROS/web performance trial, exclusively on sim domain 107."""
import argparse
from collections import defaultdict
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from urllib.request import urlopen

import numpy as np
import psutil
import rclpy
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from run_sim_trial import ROOT, Trial


def distribution(values):
    return {key: float(value) for key, value in zip(
        ('count', 'mean', 'p95', 'p99', 'max'),
        (len(values), np.mean(values), *np.percentile(values, [95, 99]), max(values)))} if values else {}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--headless', choices=['true', 'false'], default='false')
    parser.add_argument('--lidar-backend', choices=['cpp', 'python'], default='cpp')
    parser.add_argument('--render-loading', choices=['nonblocking', 'synchronous'], default='nonblocking')
    parser.add_argument('--initialization', choices=['manual', 'automatic'], default='manual')
    parser.add_argument('--goal-timeout', type=float, default=100.,
                        help='Per-goal wall-clock deadline, including recovery maneuvers')
    parser.add_argument('--navigation-config', type=Path, default=ROOT/'src/amr_system/amr_stack/config/navigation.yaml')
    parser.add_argument('--goals', type=json.loads, default=[[14.5, 3.31, 0.], [14.5, 5.5, math.pi/2], [11.5, 5.5, math.pi]])
    args = parser.parse_args()
    if not math.isfinite(args.goal_timeout) or args.goal_timeout <= 0:
        parser.error('--goal-timeout must be positive and finite')
    assert os.environ.get('ROS_DOMAIN_ID') == '107', 'Simulation domain 107 only'
    output = args.output.resolve();output.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=[])
    probe = rclpy.create_node('performance_preflight')
    for _ in range(10): rclpy.spin_once(probe, timeout_sec=.1)
    assert not any(name in {'amr_sim_bridge', 'controller_server', 'control_mux'} for name, _ in probe.get_node_names_and_namespaces()), 'Stop existing domain 107 stack first'
    probe.destroy_node()
    env = os.environ.copy();env['AMR_SIM_PROFILE'] = str(output/'sim_profile.json')
    env['AMR_LIDAR_BACKEND'] = args.lidar_backend
    env['AMR_RENDER_LOADING'] = args.render_loading
    logs = [(output/f'{name}.log').open('w') for name in ('launch', 'web')]
    stack = subprocess.Popen([str(ROOT/'scripts/experiment.sh'), 'sim', f'headless:={args.headless}', 'start_joy:=false', 'world:=floor4', f'navigation_config:={args.navigation_config}'], env=env, stdout=logs[0], stderr=subprocess.STDOUT, start_new_session=True)
    web = subprocess.Popen(['ros2', 'launch', 'amr_web_ui', 'web_ui.launch.py', 'use_sim_time:=true', 'port:=8081'], env=env, stdout=logs[1], stderr=subprocess.STDOUT, start_new_session=True)
    node = Trial(json.loads((ROOT/'sim/config/floor4.json').read_text())['spawn'])
    records = defaultdict(list);cpu = defaultdict(list);stop = threading.Event();measuring = threading.Event()
    def monitor():
        previous = {}
        previous_wall = time.monotonic()
        while not stop.wait(.2):
            started = time.monotonic()
            try:
                state = json.load(urlopen('http://127.0.0.1:8081/api/state', timeout=1))
                if measuring.is_set():
                    records['web_response_s'].append(time.monotonic()-started)
                    records['web_scan_points'].append(len((state.get('scan') or {}).get('points', [])))
            except OSError: pass
            now = time.monotonic()
            try: children = psutil.Process(stack.pid).children(recursive=True)+psutil.Process(web.pid).children(recursive=True)
            except psutil.Error: continue
            for proc in children:
                try:
                    argv = proc.cmdline()
                    name = Path(argv[1]).name if 'python' in Path(argv[0]).name and len(argv)>1 else Path(argv[0]).name
                    times = proc.cpu_times();current = times.user+times.system
                    if proc.pid in previous and measuring.is_set():cpu[name].append(100*(current-previous[proc.pid])/(now-previous_wall))
                    previous[proc.pid] = current
                except (psutil.Error, IndexError):pass
            previous_wall = now
    thread = threading.Thread(target=monitor, daemon=True);thread.start()
    clock = [0.];last_clock_wall = [None];last_scan_wall = [None]
    def record_arrival(name, previous):
        now = time.monotonic()
        if previous[0] is not None and measuring.is_set():
            records[name].append(now-previous[0])
        previous[0] = now
    def on_clock(msg):
        clock[0] = msg.clock.sec+msg.clock.nanosec/1e9
        record_arrival('clock_wall_gap_s', last_clock_wall)
    def on_scan(msg):record_arrival('scan_wall_gap_s', last_scan_wall)
    def on_tf(msg):
        if measuring.is_set():
            for transform in msg.transforms:
                if transform.child_frame_id == 'base_footprint':
                    records['tf_age_s'].append(clock[0]-(transform.header.stamp.sec+transform.header.stamp.nanosec/1e9))
    def on_log(msg):
        if measuring.is_set() and ('missed its desired rate' in msg.msg or 'Extrapolation Error' in msg.msg or 'Passing new path' in msg.msg):
            records['events'].append({'wall':time.monotonic(), 'node':msg.name, 'message':msg.msg})
    subscriptions = [node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data),
                     node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data),
                     node.create_subscription(TFMessage, '/tf', on_tf, 100), node.create_subscription(Log, '/rosout', on_log, 100)]
    report = {'headless':args.headless, 'lidar_backend_requested':args.lidar_backend,
              'initialization':args.initialization,
              'goal_timeout_s':args.goal_timeout,
              'render_loading_requested':args.render_loading,
              'goals':args.goals, 'passed':False}
    try:
        node.initialize(automatic=args.initialization == 'automatic')
        expected_backend = 'physx_scene_query_cpp_v1' if args.lidar_backend == 'cpp' else 'physx_scene_query_python'
        node.until(lambda: node.sim_status.get('lidar_backend') == expected_backend, 5)
        expected_render = args.render_loading if args.headless == 'false' else 'headless'
        node.until(lambda: node.sim_status.get('render_loading') == expected_render, 5)
        client = node.create_client(GetParameters, '/ekf_filter_node/get_parameters')
        node.until(client.service_is_ready, 5)
        requested = ['use_sim_time', 'predict_to_current_time', 'frequency']
        future = client.call_async(GetParameters.Request(names=requested))
        node.until(future.done, 5)
        report['ekf_parameters'] = {name: value.bool_value if value.type == 1 else value.double_value
                                    for name, value in zip(requested, future.result().values)}
        node.destroy_client(client)
        node.samples.clear();node.raw_command_times.clear();measuring.set()
        started = time.monotonic();sim_start = clock[0]
        report['goal_statuses'] = []
        report['within_goal_command_gaps_s'] = []
        for goal in args.goals:
            command_start = len(node.raw_command_times)
            status = node.goal(*goal, timeout=args.goal_timeout)
            report['within_goal_command_gaps_s'].extend(
                np.diff(node.raw_command_times[command_start:]).tolist())
            report['goal_statuses'].append(status)
            print('PERF_GOAL', goal, status, flush=True)
            if status != 4:break
        report.update(wall_s=time.monotonic()-started, sim_s=clock[0]-sim_start,
                      contacts=node.sim_status.get('obstacle_contact_events'), recoveries=node.recoveries)
        report['rtf'] = report['sim_s']/report['wall_s']
        report['passed'] = len(report['goal_statuses'])==len(args.goals) and all(s==4 for s in report['goal_statuses']) and report['contacts']==0
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        measuring.clear();stop.set();thread.join(timeout=3)
        gaps = np.diff(node.raw_command_times).tolist()
        report['command_gap_s'] = distribution(gaps)
        report['command_gaps_over_75ms'] = sum(v>.075 for v in gaps)
        active_gaps = report.pop('within_goal_command_gaps_s', [])
        report['within_goal_command_gap_s'] = distribution(active_gaps)
        report['within_goal_command_gaps_over_75ms'] = sum(v>.075 for v in active_gaps)
        report['events'] = records.pop('events', [])
        report['measurements'] = {k:distribution(v) for k,v in records.items()}
        report['cpu_percent_one_core'] = {k:distribution(v) for k,v in cpu.items()}
        report['simulation'] = node.sim_status
        (output/'samples.json').write_text(json.dumps(node.samples)+'\n')
        (output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
        for process in (stack, web):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        for log in logs:log.close()
    print(json.dumps({k:v for k,v in report.items() if k not in ('events','cpu_percent_one_core')}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
