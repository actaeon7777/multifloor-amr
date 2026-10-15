"""Integration checks for bounded, read-only ROS parameter snapshots."""
from pathlib import Path
import sys
import threading
import time

import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, ListParameters

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from capture_parameters import capture, trial_nodes


@pytest.fixture
def parameter_servers():
    rclpy.init(args=[], domain_id=109)
    executor = SingleThreadedExecutor()
    nodes = []
    for full_name in trial_nodes('sim'):
        namespace, _, name = ('/' + full_name).rpartition('/')
        node = rclpy.create_node(name, namespace=namespace or '/', enable_rosout=False)
        node.declare_parameter('enabled', True)
        node.declare_parameter('controller.max_velocity', .4)
        node.declare_parameter('controller.critics', ['PathAlign', 'GoalAlign'])
        node.declare_parameter('footprint_padding', 0.0)
        if name == 'ekf_filter_node':
            # A real rclpy server returns [] for an entire get containing this slot.
            node.declare_parameter('unused_imu', Parameter.Type.DOUBLE)
        executor.add_node(node)
        nodes.append(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield nodes, executor
    finally:
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def test_all_simulation_nodes_keep_types_nested_names_and_unset_slots(parameter_servers, tmp_path):
    summary = capture(trial_nodes('sim'), tmp_path, timeout=5)
    assert all(result['status'] == 'saved' for result in summary['nodes'].values())
    assert '/cmd_vel_to_can' not in summary['nodes']
    for name in trial_nodes('sim'):
        document = yaml.safe_load((tmp_path / (name.replace('/', '__') + '.yaml')).read_text())
        params = document['/' + name]['ros__parameters']
        assert params['enabled'] is True
        assert params['controller']['max_velocity'] == .4
        assert params['controller']['critics'] == ['PathAlign', 'GoalAlign']
        assert params['footprint_padding'] == 0.0
        assert 'unused_imu' not in params
    assert summary['nodes']['/ekf_filter_node']['unset_parameters'] == ['unused_imu']


def test_missing_nodes_share_deadline_and_successful_snapshots_survive(parameter_servers, tmp_path):
    started = time.monotonic()
    summary = capture(('controller_server', 'absent_one', 'absent_two'), tmp_path, timeout=1.5)
    assert time.monotonic() - started < 3
    assert summary['nodes']['/controller_server']['status'] == 'saved'
    assert summary['nodes']['/absent_one']['status'] == 'failed'
    assert summary['nodes']['/absent_two']['status'] == 'failed'
    assert (tmp_path / 'absent_one.failed.txt').is_file()
    assert not (tmp_path / 'absent_one.yaml').exists()
    assert 'cmd_vel_to_can' in trial_nodes('real')


def test_unresponsive_parameter_services_do_not_block_capture(parameter_servers, tmp_path):
    # Advertise services without spinning the owner, so requests never complete.
    node = rclpy.create_node('unresponsive', start_parameter_services=False, enable_rosout=False)
    node.create_service(ListParameters, '/unresponsive/list_parameters', lambda req, res: res)
    node.create_service(GetParameters, '/unresponsive/get_parameters', lambda req, res: res)
    try:
        started = time.monotonic()
        summary = capture(('controller_server', 'unresponsive'), tmp_path, timeout=1.5)
        assert time.monotonic() - started < 3
        assert summary['nodes']['/controller_server']['status'] == 'saved'
        assert summary['nodes']['/unresponsive']['status'] == 'failed'
        assert '응답 시간 초과' in summary['nodes']['/unresponsive']['error']
    finally:
        node.destroy_node()
