#!/usr/bin/env python3
"""Read trial parameters through one DDS participant with a shared deadline."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.parameter import parameter_value_to_python
import yaml


COMMON_NODES = (
    'controller_server', 'planner_server', 'global_costmap/global_costmap',
    'local_costmap/local_costmap', 'bt_navigator', 'behavior_server',
    'navigation_manager', 'navigation_startup', 'tim_global_localizer',
    'control_mux', 'ekf_filter_node', 'amcl',
)


def trial_nodes(mode):
    return COMMON_NODES + (('cmd_vel_to_can',) if mode == 'real' else ())


def nested_parameters(values):
    """Use the same dotted-name nesting as ros2 param dump."""
    result = {}
    for name, value in sorted(values.items()):
        branch = result
        *prefixes, leaf = name.split('.')
        for prefix in prefixes:
            branch = branch.setdefault(prefix, {})
        branch[leaf] = value
    return result


def capture(nodes, output, timeout=8.0):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    reader = rclpy.create_node(
        '_trial_parameter_snapshot', start_parameter_services=False,
        enable_rosout=False,
    )
    states = {}
    try:
        for name in nodes:
            name = '/' + name.strip('/')
            states[name] = {
                'list_client': reader.create_client(ListParameters, f'{name}/list_parameters'),
                'get_client': reader.create_client(GetParameters, f'{name}/get_parameters'),
                'stage': 'discovery', 'values': {}, 'unset': [],
                'queue': deque(), 'requests': [],
            }

        def finish(name, state, status, error=None):
            state['stage'] = 'done'
            state['result'] = {
                'status': status, 'elapsed_s': round(time.monotonic() - started, 3),
                'parameter_count': len(state['values']),
                'unset_parameters': sorted(state['unset']),
            }
            filename = name.strip('/').replace('/', '__')
            if error:
                state['result']['error'] = error
                (output / f'{filename}.failed.txt').write_text(error + '\n')
                print(f'파라미터 저장 건너뜀: {name} ({error})', flush=True)
            else:
                document = {name: {'ros__parameters': nested_parameters(state['values'])}}
                (output / f'{filename}.yaml').write_text(
                    yaml.safe_dump(document, allow_unicode=True, sort_keys=True))

        while any(state['stage'] != 'done' for state in states.values()):
            if time.monotonic() - started >= timeout:
                for name, state in states.items():
                    if state['stage'] != 'done':
                        reason = ('미실행 또는 파라미터 서비스 미발견'
                                  if state['stage'] == 'discovery' else '파라미터 응답 시간 초과')
                        finish(name, state, 'failed', reason)
                break
            for name, state in states.items():
                try:
                    if state['stage'] == 'discovery':
                        if (state['list_client'].service_is_ready()
                                and state['get_client'].service_is_ready()):
                            state['list_future'] = state['list_client'].call_async(ListParameters.Request())
                            state['stage'] = 'list'
                    if state['stage'] == 'list' and state['list_future'].done():
                        names = sorted(state['list_future'].result().result.names)
                        if names:
                            state['queue'].append(names)
                        state['stage'] = 'get'
                    if state['stage'] == 'get':
                        remaining = []
                        for names, future in state['requests']:
                            if not future.done():
                                remaining.append((names, future))
                                continue
                            values = future.result().values
                            if len(values) == len(names):
                                for parameter_name, value in zip(names, values):
                                    if value.type == ParameterType.PARAMETER_NOT_SET:
                                        state['unset'].append(parameter_name)
                                    else:
                                        state['values'][parameter_name] = parameter_value_to_python(value)
                            elif not values:
                                # robot_localization has declared but unset sensor slots.
                                # An unset slot can make the entire bulk get return [].
                                # Split only failing batches; retain all initialized values.
                                if len(names) == 1:
                                    state['unset'].append(names[0])
                                else:
                                    middle = len(names) // 2
                                    state['queue'].extend((names[:middle], names[middle:]))
                            else:
                                raise RuntimeError('parameter name/value count mismatch')
                        state['requests'] = remaining
                        while state['queue'] and len(state['requests']) < 4:
                            names = state['queue'].popleft()
                            future = state['get_client'].call_async(GetParameters.Request(names=names))
                            state['requests'].append((names, future))
                        if not state['requests'] and not state['queue']:
                            finish(name, state, 'saved')
                except Exception as error:
                    finish(name, state, 'failed', str(error))
            if any(state['stage'] != 'done' for state in states.values()):
                rclpy.spin_once(reader, timeout_sec=.02)

        results = {name: state['result'] for name, state in states.items()}
        elapsed = time.monotonic() - started
        (output / 'collection.log').write_text(''.join(
            f'{name.strip("/")}: {result["status"]} ({result["elapsed_s"]:.3f}s)\n'
            for name, result in results.items()))
        summary = {'elapsed_s': round(elapsed, 3), 'timeout_s': timeout, 'nodes': results}
        (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
        saved = sum(result['status'] == 'saved' for result in results.values())
        print(f'파라미터 저장: {saved}/{len(results)}개 노드, {elapsed:.2f}초', flush=True)
        return summary
    finally:
        reader.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['sim', 'real'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=8.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    rclpy.init(args=[])
    try:
        capture(trial_nodes(args.mode), args.output, args.timeout)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
