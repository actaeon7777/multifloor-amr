"""Opt-in localhost check for TIM startup-only parameter semantics.

Run with the workspace sourced:
  ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 python3 <this file>
No robot topics, transforms, localization requests, or motion are published.
"""
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import DescribeParameters, GetParameters, SetParameters


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '198', 'Use isolated domain 198'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Use localhost-only DDS'
    prefix = Path(get_package_prefix('amr_tim_localization'))
    config = Path(get_package_share_directory('amr_tim_localization')) / 'config/tim_localization.yaml'
    names = ['max_beams', 'max_range', 'max_timeout_sec', 'max_map_cells', 'threads',
             'random_seed', 'map_sampling_ratio', 'sampling_duration_sec', 'anchor_spacing',
             'min_laser_clearance', 'angle_search_step', 'batch_size', 'candidates_per_batch',
             'min_score', 'max_score', 'sigma_hit', 'max_tolerance_dist',
             'max_correspondence_distance', 'icp_iterations', 'nms_distance', 'nms_angle',
             'service_name', 'fine_enabled', 'fine_max_candidates', 'fine_max_beams',
             'fine_map_sampling_ratio', 'fine_icp_iterations', 'fine_correspondence_distance',
             'fine_max_translation', 'fine_max_yaw', 'fine_min_overlap',
             'fine_min_geometry_ratio', 'fine_budget_sec', 'fine_min_score_gain']
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([str(prefix / 'lib/amr_tim_localization/tim_global_localizer'),
                                    '--ros-args', '--params-file', str(config),
                                    '-p', 'max_range:=28.0'], stdout=log, stderr=log,
                                   start_new_session=True)
        node = None
        try:
            rclpy.init()
            node = rclpy.create_node('tim_readonly_parameters_test')
            get_client = node.create_client(GetParameters, '/tim_global_localizer/get_parameters')
            describe_client = node.create_client(DescribeParameters, '/tim_global_localizer/describe_parameters')
            set_client = node.create_client(SetParameters, '/tim_global_localizer/set_parameters')

            def call(client, request):
                assert client.wait_for_service(timeout_sec=10.), 'Parameter service unavailable'
                future = client.call_async(request)
                deadline = time.monotonic() + 10.
                while not future.done() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.05)
                assert future.done(), 'Parameter request timed out'
                return future.result()

            descriptors = call(describe_client, DescribeParameters.Request(names=names)).descriptors
            assert len(descriptors) == len(names) and all(d.read_only for d in descriptors)
            before = call(get_client, GetParameters.Request(names=names)).values
            assert before[names.index('max_range')].double_value == 28., 'Startup override was ignored'
            changes = []
            for name, old in zip(names, before):
                value = copy.deepcopy(old)
                if value.type == ParameterType.PARAMETER_BOOL:
                    value.bool_value = not value.bool_value
                elif value.type == ParameterType.PARAMETER_INTEGER:
                    value.integer_value += 1
                elif value.type == ParameterType.PARAMETER_DOUBLE:
                    value.double_value += .1
                elif value.type == ParameterType.PARAMETER_STRING:
                    value.string_value += '_changed'
                else:
                    raise AssertionError(f'Unexpected TIM parameter type: {name}')
                changes.append(Parameter(name=name, value=value))
            results = call(set_client, SetParameters.Request(parameters=changes)).results
            assert len(results) == len(names) and all(not r.successful for r in results), results
            assert all('read-only' in r.reason.lower() or 'read only' in r.reason.lower() for r in results)
            after = call(get_client, GetParameters.Request(names=names)).values
            assert before == after, 'Rejected write changed a declared parameter value'
            sim_descriptor = call(describe_client, DescribeParameters.Request(names=['use_sim_time'])).descriptors[0]
            assert not sim_descriptor.read_only, 'Standard ROS time behavior was changed'
            for desired in (True, False):
                result = call(set_client, SetParameters.Request(parameters=[Parameter(
                    name='use_sim_time', value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                                             bool_value=desired))])).results[0]
                assert result.successful, result.reason
                assert call(get_client, GetParameters.Request(names=['use_sim_time'])).values[0].bool_value == desired
            print(json.dumps({'read_only_parameters': len(names), 'rejected_writes': len(results),
                              'reported_values_unchanged': True, 'startup_override_honored': True,
                              'use_sim_time_runtime_changes': 'accepted'}, indent=2))
        except BaseException:
            log.seek(0)
            print(log.read())
            raise
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5.)


if __name__ == '__main__':
    main()
