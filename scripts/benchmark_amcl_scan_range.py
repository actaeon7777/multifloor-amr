#!/usr/bin/env python3
"""Replay recorded sensors into real AMCL, isolated on localhost domain 195.

Only map, static TF, odom->base_footprint TF, scan and initialpose are published.
Recorded AMCL poses/map->odom are never replay inputs. Ground truth is used only
for timestamp-interpolated offline evaluation. No motion outputs are created.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage


def stamp(s):
    return s.sec + s.nanosec / 1e9


def pose_row(message):
    p = message.pose.pose
    q = p.orientation
    yaw = math.atan2(2 * (q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return [stamp(message.header.stamp), p.position.x, p.position.y, yaw]


def load_bag(path):
    connection = sqlite3.connect(f'file:{path}?mode=ro', uri=True)
    types = {i: (n, get_message(t)) for i, n, t in connection.execute('select id,name,type from topics')}
    wanted = {'/map', '/tf_static', '/tf', '/clock', '/scan', '/initialpose',
              '/parameter_events', '/rosout', '/sim/ground_truth', '/amcl_pose'}
    events, truth, original = [], [], []
    for timestamp, topic, raw in connection.execute('select timestamp,topic_id,data from messages order by timestamp,id'):
        name, typ = types[topic]
        if name not in wanted:
            continue
        message = deserialize_message(raw, typ)
        if name == '/sim/ground_truth':
            truth.append(pose_row(message))
        elif name == '/amcl_pose':
            original.append(pose_row(message))
        elif name == '/tf':
            transforms = [t for t in message.transforms
                          if t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint']
            if transforms:
                events.append((timestamp/1e9, '/tf', TFMessage(transforms=transforms)))
        elif name == '/parameter_events':
            if message.node == '/amcl':
                for p in [*message.new_parameters, *message.changed_parameters]:
                    if p.name == 'max_beams':
                        events.append((timestamp/1e9, 'beams', p.value.integer_value))
        elif name == '/rosout':
            if message.name == 'amcl' and 'Requesting no-motion update' in message.msg:
                events.append((timestamp/1e9, 'nomotion', None))
        else:
            if name == '/tf_static':
                assert all(t.header.frame_id != 'map' and t.child_frame_id != 'odom'
                           for t in message.transforms)
            events.append((timestamp/1e9, name, message))
    connection.close()
    return events, np.asarray(truth), original


def evaluate(rows, truth):
    values = []
    unwrapped = np.unwrap(truth[:, 3])
    for t, x, y, yaw in rows:
        if not truth[0, 0] <= t <= truth[-1, 0]:
            continue
        actual = [np.interp(t, truth[:, 0], truth[:, i]) for i in (1, 2)]
        actual_yaw = np.interp(t, truth[:, 0], unwrapped)
        angle = abs(math.atan2(math.sin(yaw-actual_yaw), math.cos(yaw-actual_yaw)))
        values.append(dict(stamp=t, position_error_m=math.hypot(x-actual[0], y-actual[1]),
                           yaw_error_deg=math.degrees(angle), estimate=[x, y, yaw],
                           truth=[*actual, actual_yaw]))
    summaries = {}
    for name, minimum in [('all', 0), ('tracking', 4), ('late_tracking', 20)]:
        group = [v for v in values if v['stamp'] >= minimum]
        summaries[name] = {'samples': len(group)}
        for key in ('position_error_m', 'yaw_error_deg'):
            numbers = [v[key] for v in group]
            if numbers:
                summaries[name].update({f'{key}_{stat}': float(value) for stat, value in (
                    ('mean', np.mean(numbers)), ('p95', np.percentile(numbers, 95)), ('max', max(numbers)))})
    return dict(summary=summaries, samples=values)


def trial(events, laser_range, config, directory):
    rclpy.init()
    node = Node('offline_amcl_range_replay')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publishers = {}
    for _, name, msg in events:
        if name.startswith('/') and name not in publishers:
            publishers[name] = node.create_publisher(type(msg), name, latched if name in ['/map', '/tf_static'] else 100)
    outputs, parameter_results = [], []
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda msg: outputs.append(pose_row(msg)), latched)
    lifecycle = node.create_client(ChangeState, '/amcl/change_state')
    parameters = node.create_client(SetParametersAtomically, '/amcl/set_parameters_atomically')
    nomotion = node.create_client(Empty, '/request_nomotion_update')
    process = None
    logfile = directory / f'range_{laser_range:g}m.amcl.log'

    def until(predicate, timeout=10):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > end:
                raise TimeoutError('AMCL setup/service deadline expired')
            rclpy.spin_once(node, timeout_sec=.01)

    try:
        with logfile.open('w') as log:
            process = subprocess.Popen([
                '/opt/ros/jazzy/lib/nav2_amcl/amcl', '--ros-args', '--params-file', str(config),
                '-p', 'use_sim_time:=true', '-p', 'max_beams:=360',
                '-p', f'laser_max_range:={float(laser_range)}',
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            for _, name, message in events:
                if name in ['/map', '/tf_static']:
                    publishers[name].publish(message)
            first_clock = next(message for _, name, message in events if name == '/clock')
            publishers['/clock'].publish(first_clock)
            until(lifecycle.service_is_ready)
            for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                request = ChangeState.Request()
                request.transition.id = transition
                future = lifecycle.call_async(request)
                until(future.done)
                assert future.result().success
            until(lambda: publishers['/initialpose'].get_subscription_count() > 0
                  and publishers['/scan'].get_subscription_count() > 0)
            origin, started = events[0][0], time.monotonic()
            counts = {}
            for timestamp, name, message in events:
                deadline = started + timestamp - origin
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=min(.003, deadline-time.monotonic()))
                counts[name] = counts.get(name, 0)+1
                if name == 'beams':
                    p = Parameter(name='max_beams', value=ParameterValue(
                        type=ParameterType.PARAMETER_INTEGER, integer_value=message))
                    future = parameters.call_async(SetParametersAtomically.Request(parameters=[p]))
                    until(future.done, 2)
                    parameter_results.append(dict(stamp=timestamp, max_beams=message,
                                                  successful=future.result().result.successful))
                    assert future.result().result.successful
                elif name == 'nomotion':
                    nomotion.call_async(Empty.Request())
                else:
                    publishers[name].publish(message)
                rclpy.spin_once(node, timeout_sec=0)
            end = time.monotonic()+.5
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.01)
            assert outputs, logfile.read_text()[-3000:]
            return dict(laser_max_range=laser_range, outputs=outputs, input_counts=counts,
                        beam_updates=parameter_results, log=str(logfile),
                        elapsed_wall_sec=time.monotonic()-started)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '195' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Requires isolated ROS_DOMAIN_ID=195 ROS_LOCALHOST_ONLY=1')
    database = args.trial / 'bag/bag_0.db3'
    config = args.trial / 'params/amcl.yaml'
    args.output.mkdir(parents=True, exist_ok=True)
    events, truth, original = load_bag(database)
    result = dict(status='running', bag=str(database), config=str(config), domain=195,
                  config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),
                  input_policy='Recorded map/scan/initialpose/clock/static TF and only odom->base_footprint dynamic TF. No recorded map->odom or AMCL pose input.',
                  replay_rate=1.0, original_recorded=evaluate(original, truth), trials=[])
    report = args.output / 'comparison.json'
    try:
        for laser_range in (5.0, 30.0):
            sample = trial(events, laser_range, config, args.output)
            sample.update(evaluate(sample.pop('outputs'), truth))
            result['trials'].append(sample)
            report.write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps(dict(range=laser_range, summary=sample['summary'])), flush=True)
        result['baseline_exactly_reproduces_recorded_pose_samples'] = (
            result['trials'][0]['samples'] == result['original_recorded']['samples'])
        result['limitations'] = [
            'One recorded trajectory and its recorded initial-pose sequence; not a multi-location robustness claim.',
            'Open-loop sensor replay compares estimator accuracy, not changed closed-loop driving behavior.',
            'The localization quality gate is not run here; lower estimator error alone does not prove gate passage.',
        ]
        result['status'] = 'passed'
    finally:
        report.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
