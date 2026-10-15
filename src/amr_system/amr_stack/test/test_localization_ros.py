"""Load the selected real map and activate AMCL in an isolated domain, without IO."""
import os
import signal
import subprocess
import time

import pytest

if os.environ.get('AMR_RUN_LOCALIZATION_TESTS') != '1':
    pytest.skip('Opt in with AMR_RUN_LOCALIZATION_TESTS=1, domain 189 localhost only', allow_module_level=True)

import rclpy
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import GetParameters


def test_saved_map_and_amcl_lifecycle(tmp_path):
    assert os.environ['ROS_DOMAIN_ID'] == '189'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rclpy.init()
    node = rclpy.create_node('localization_test_probe')
    log = (tmp_path/'launch.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'amr_stack', 'localization.launch.py',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def spin_until(predicate, timeout=15):
        deadline = time.monotonic()+timeout
        while not predicate():
            assert process.poll() is None, (tmp_path/'launch.log').read_text()
            assert time.monotonic()<deadline, (tmp_path/'launch.log').read_text()
            rclpy.spin_once(node, timeout_sec=.02)

    def request(client, req):
        spin_until(client.service_is_ready)
        future = client.call_async(req)
        spin_until(future.done)
        return future.result()

    try:
        for name in ['map_server','amcl']:
            client = node.create_client(GetState, '/'+name+'/get_state')
            deadline = time.monotonic()+15
            while True:
                if request(client, GetState.Request()).current_state.id == State.PRIMARY_STATE_ACTIVE:
                    break
                assert time.monotonic()<deadline
                rclpy.spin_once(node, timeout_sec=.1)
        # Subscribe AFTER activation: stored map must reach late transient-local clients.
        maps = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, '/map', maps.append, qos)
        spin_until(lambda: bool(maps))
        m = maps[-1]
        assert m.header.frame_id == 'map'
        assert (m.info.width,m.info.height) == (1014,462)
        assert m.info.resolution == pytest.approx(.05)
        assert m.info.origin.position.x == pytest.approx(-3.8)
        assert m.info.origin.position.y == pytest.approx(-1.54)
        assert len(m.data) == 1014*462
        assert {0,100} <= set(m.data) <= {-1,0,100}
        assert node.count_publishers('/map') == 1
        client = node.create_client(GetParameters, '/amcl/get_parameters')
        req = GetParameters.Request()
        req.names = ['base_frame_id','odom_frame_id','global_frame_id','scan_topic','set_initial_pose','always_reset_initial_pose','laser_max_range']
        values = request(client,req).values
        assert [v.string_value for v in values[:4]] == ['base_footprint','odom','map','/scan']
        assert not values[4].bool_value and values[5].bool_value
        assert values[6].double_value == pytest.approx(30.)
        names = [name for name,_ in node.get_node_names_and_namespaces()]
        assert {'map_server','amcl','lifecycle_manager_localization'} <= set(names)
        assert not {'ekf_filter_node','cmd_vel_to_can','control_mux','can_telemetry',
                    'sllidar_node','controller_server','planner_server','slam_toolbox'} & set(names)
    finally:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()
