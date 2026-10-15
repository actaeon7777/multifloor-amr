#!/usr/bin/env python3
"""Keep one Web UI alive across two Isaac runs, with no hardware IO (domain 107)."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from urllib.request import urlopen

import rclpy
from run_sim_trial import ROOT, Trial


def stop(process):
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGINT)  # ros2 launch forwards it once to its children.
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '107', 'Simulation domain 107 only'
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Trial(json.loads((ROOT/'sim/config/floor4.json').read_text())['spawn'])
    node.spin(1.)
    assert not any(name in {'amr_sim_bridge', 'controller_server', 'control_mux', 'amr_web_ui'}
                   for name, _ in node.get_node_names_and_namespaces()), 'An existing stack/UI must remain untouched'
    logs = []
    stack = web = None
    report = dict(passed=False, runs=[])
    def launch(command, name):
        stream = (args.output/name).open('w'); logs.append(stream)
        return subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    def state():
        try:
            return json.load(urlopen('http://127.0.0.1:8081/api/state', timeout=1))
        except OSError:
            return {}
    def wait_state(predicate, timeout=20):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            value = state()
            if predicate(value): return value
            node.spin(.1)
        raise AssertionError(state())
    try:
        web = launch(['ros2', 'launch', 'amr_web_ui', 'web_ui.launch.py',
                      'use_sim_time:=true', 'port:=8081', 'host:=127.0.0.1'], 'web.log')
        report['web_pid'] = web.pid
        previous_session = None
        for run in range(2):
            node.localization_status = {}
            node.truth = node.odom = node.scan = None
            stack = launch([str(ROOT/'scripts/experiment.sh'), 'sim', 'headless:=true',
                            'start_joy:=false'], f'stack_{run}.log')
            node.initialize(automatic=True)
            ready = wait_state(lambda s: (s.get('localization') or {}).get('ready')
                and (s.get('localization') or {}).get('session') != previous_session
                and s.get('robot') and not s.get('tf_error'))
            assert abs(ready['robot']['x']-11.5) < .3, ready['robot']
            previous_session = ready['localization']['session']
            result = node.goal(14.5, 3.31, 0., timeout=60)
            assert result == 4, result
            arrived = wait_state(lambda s: s.get('robot') and abs(s['robot']['x']-14.5) < .3)
            report['runs'].append(dict(session=previous_session, goal_status=result,
                connection_epoch=ready['connection_epoch'], start_robot=ready['robot'], end_robot=arrived['robot']))
            stop(stack); stack = None
            disconnected = wait_state(lambda s: s and s.get('tf_error')
                and not (s.get('localization') or {}).get('ready'), timeout=5)
            report['runs'][-1]['disconnected_tf_error'] = disconnected['tf_error']
            assert web.poll() is None
        assert report['runs'][1]['connection_epoch'] > report['runs'][0]['connection_epoch']
        report['same_web_process_survived'] = web.poll() is None
        report['passed'] = True
    except Exception as error:
        report['error'] = repr(error)
        raise
    finally:
        stop(stack); stop(web)
        for stream in logs: stream.close()
        text = (args.output/'web.log').read_text(errors='replace') if (args.output/'web.log').exists() else ''
        report['tf_old_data_warnings'] = text.count('TF_OLD_DATA')
        (args.output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        node.destroy_node(); rclpy.shutdown()
        print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
