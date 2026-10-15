#!/usr/bin/env python3
"""Compare manual-pose readiness with real AMCL in isolated ROS domain 194.

Uses the startup integration harness's synthetic map, 3240-ray scans and odometry.
Navigation lifecycle responses are mocked: the result excludes real Nav2 startup
time. No hardware, simulator, controllers or velocity-command publishers run.
"""

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml


ROOT = Path(__file__).resolve().parents[1]
STACK = ROOT / 'src/amr_system/amr_stack'


def load_module(name, path):
    specification = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(specification)
    sys.modules[name] = module
    specification.loader.exec_module(module)
    return module


def fingerprint(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--module', type=Path, default=STACK / 'amr_stack/navigation_startup.py')
    parser.add_argument('--config', type=Path, default=STACK / 'config/localization_startup.yaml')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--label', required=True)
    parser.add_argument('--timeout', type=float, default=60.0)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '194' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Requires ROS_DOMAIN_ID=194 and ROS_LOCALHOST_ONLY=1')
    os.environ['AMR_RUN_STARTUP_TESTS'] = '1'
    sys.path.insert(0, str(STACK))

    implementation = load_module('amr_stack.manual_latency_implementation', args.module)
    harness_module = load_module(
        'manual_latency_ros_harness', STACK / 'test/test_navigation_startup_ros.py',
    )
    harness_module.NavigationStartup = implementation.NavigationStartup
    harness_module.navigation_startup = implementation
    config = yaml.safe_load(args.config.read_text())['navigation_startup']['ros__parameters']
    config['automatic_localization'] = False
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log_path = args.output.with_suffix('.amcl.log')
    trace, poses = [], []
    seed_at = None
    process = None
    harness = None
    result = {
        'label': args.label,
        'status': 'running',
        'implementation': str(args.module.resolve()),
        'implementation_sha256': fingerprint(args.module),
        'startup_config': str(args.config.resolve()),
        'startup_config_sha256': fingerprint(args.config),
        'startup_parameters': config,
        'scope': 'Actual AMCL; synthetic map/scan/odom; mocked navigation lifecycle; no motion output',
        'domain': 194,
        'scan_count': 3240,
        'seed_covariance': {'x': .25, 'y': .25, 'yaw': .06853891909122467},
        'amcl_log': str(log_path.resolve()),
    }

    def elapsed():
        return None if seed_at is None else round(time.monotonic() - seed_at, 6)

    def status_callback(message):
        status = json.loads(message.data)
        current = (status['state'], status['detail'])
        if not trace or current != (trace[-1]['state'], trace[-1]['detail']):
            trace.append({'elapsed_sec': elapsed(), **status})

    def pose_callback(message):
        poses.append({
            'elapsed_sec': elapsed(),
            'stamp_ns': implementation.stamp_ns(message.header.stamp),
            'x': message.pose.pose.position.x,
            'y': message.pose.pose.position.y,
            'yaw': implementation.yaw(message.pose.pose.orientation),
            'variance_x': message.pose.covariance[0],
            'variance_y': message.pose.covariance[7],
            'variance_yaw': message.pose.covariance[35],
        })

    try:
        harness = harness_module.Harness(
            real_amcl=True, map_tf=False, scan_count=3240, startup_parameters=config,
        )
        harness.probe.create_subscription(
            harness_module.String, '/localization/status', status_callback, harness.backend.latched,
        )
        harness.probe.create_subscription(
            harness_module.PoseWithCovarianceStamped, '/amcl_pose', pose_callback,
            harness.backend.latched,
        )
        amcl_config = STACK / 'config/localization.yaml'
        result['amcl_config_sha256'] = fingerprint(amcl_config)
        with log_path.open('w') as log:
            process = subprocess.Popen([
                '/opt/ros/jazzy/lib/nav2_amcl/amcl', '--ros-args', '--params-file', str(amcl_config),
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            client = harness.probe.create_client(harness_module.ChangeState, '/amcl/change_state')
            harness.until(client.service_is_ready)
            for transition in (
                harness_module.Transition.TRANSITION_CONFIGURE,
                harness_module.Transition.TRANSITION_ACTIVATE,
            ):
                request = harness_module.ChangeState.Request()
                request.transition.id = transition
                future = client.call_async(request)
                harness.until(future.done)
                assert future.result().success, log_path.read_text()
            harness.until(lambda: harness.supervisor.lifecycle_active('amcl'))
            harness.until(lambda: harness.initial_pub.get_subscription_count() >= 2)
            harness.until(lambda: harness.supervisor.state == 'WAITING_INITIAL_POSE')
            result['initial_state'] = harness.supervisor.state
            seed = harness.backend.estimated_pose()
            seed.pose.covariance[0] = seed.pose.covariance[7] = .25
            seed.pose.covariance[35] = .06853891909122467
            seed_at = time.monotonic()
            harness.initial_pub.publish(seed)
            harness.until(lambda: harness.supervisor.source == 'manual')
            harness.until(lambda: harness.supervisor.state == 'READY', timeout=args.timeout)
            result['ready_elapsed_sec'] = elapsed()
            harness.spin(.1)
            assert process.poll() is None, log_path.read_text()
            result.update(
                status='passed',
                final_state=harness.supervisor.state,
                final_metrics=harness.supervisor.metrics,
                nav_start_commands=harness.backend.start_commands,
                effective_parameters=harness.supervisor.cfg,
            )
    except BaseException as exc:
        result.update(status='failed', error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        if harness is not None:
            harness.close()
        result.update(trace=trace, amcl_pose_updates=poses)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({
            key: result.get(key) for key in ('label', 'status', 'ready_elapsed_sec', 'error')
        }, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
