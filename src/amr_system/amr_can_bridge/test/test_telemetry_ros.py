"""Opt-in real ROS pub/sub with injected receive-only transport; no hardware IO."""
import os
import struct
import time
from types import SimpleNamespace

import pytest

if os.environ.get('AMR_RUN_TELEMETRY_TESTS') != '1':
    pytest.skip('Set AMR_RUN_TELEMETRY_TESTS=1 in domain 184', allow_module_level=True)

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from amr_can_bridge import can_telemetry


class ReceiveOnlyBus:
    def __init__(self, **kwargs):
        self.closed = False
        assert kwargs['receive_own_messages'] is False
        assert {f['can_id'] for f in kwargs['can_filters']} == {0x009,0x029,0x301,0x302,0x017,0x037,0x001,0x021}

    def recv(self, timeout):
        assert 0. <= timeout <= .1
        time.sleep(timeout)  # the receive thread blocks here like SocketCAN
        return None

    def send(self, *args, **kwargs):
        pytest.fail('Telemetry must never send CAN frames')

    def shutdown(self):
        self.closed = True


def test_ros_measurements_and_stale_diagnostics(monkeypatch):
    assert os.environ['ROS_DOMAIN_ID'] == '184'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    rclpy.init()
    node = can_telemetry.CanTelemetry(bus_factory=ReceiveOnlyBus)
    probe = rclpy.create_node('telemetry_test_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)
    odom, imu, joints, diagnostics = [], [], [], []
    probe.create_subscription(Odometry, 'wheel/odometry', odom.append, 10)
    probe.create_subscription(Imu, 'imu/data_raw', imu.append, qos_profile_sensor_data)
    probe.create_subscription(JointState, 'joint_states', joints.append, 10)
    probe.create_subscription(DiagnosticArray, 'diagnostics', diagnostics.append, 10)
    now = [time.time()]
    monkeypatch.setattr(can_telemetry, 'time', lambda: now[0])
    monkeypatch.setattr(can_telemetry, 'monotonic', lambda: now[0])

    def spin(duration=.1):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.005)

    def feed(cid, payload, stamp=None, **flags):
        msg = SimpleNamespace(arbitration_id=cid, data=payload,
                              timestamp=now[0] if stamp is None else stamp,
                              is_extended_id=False, is_remote_frame=False,
                              is_error_frame=False, is_fd=False)
        for k,v in flags.items():
            setattr(msg,k,v)
        node.process_message(msg,now[0])

    bus = node.bus
    try:
        spin(.4)
        feed(0x009, struct.pack('<ff', 20., 0.))
        feed(0x029, struct.pack('<ff', -30., 0.))
        now[0] += .1
        feed(0x009, struct.pack('<ff', 20.1, 1.))
        feed(0x029, struct.pack('<ff', -30.1, -1.))
        feed(0x301, struct.pack('<hhhH', 0, 0, 16384, 7))
        feed(0x302, struct.pack('<hhhH', 0, 0, 1000, 7))
        node.publish_health()
        spin()
        assert len(odom) == 2 and len(joints) == 2 and len(imu) == 1
        assert odom[-1].header.frame_id == 'odom'
        assert odom[-1].child_frame_id == 'base_footprint'
        assert odom[-1].pose.pose.position.x == pytest.approx(.1*3.141592653589793*.172, abs=1e-6)
        assert odom[-1].twist.twist.linear.x == pytest.approx(3.141592653589793*.172)
        assert odom[-1].pose.covariance[0] > odom[0].pose.covariance[0]
        assert joints[-1].name == ['left_drive_wheel_joint','right_drive_wheel_joint']
        assert imu[-1].header.frame_id == 'imu_link'
        assert imu[-1].orientation_covariance[0] == -1.
        assert imu[-1].linear_acceleration.z == pytest.approx(16384*.000061*9.80665)
        assert imu[-1].angular_velocity.z == pytest.approx(4.375*3.141592653589793/180)
        assert diagnostics[-1].status[0].level == DiagnosticStatus.OK
        assert probe.count_publishers('/tf') == 0
        assert probe.count_publishers('/tf_static') == 0
        # Old queued data, RTR requests and corrupt floats cannot create odometry.
        now[0] += 1.
        feed(0x009, bytes(8), stamp=now[0]-1.)
        feed(0x029, bytes(8), is_remote_frame=True)
        feed(0x009, struct.pack('<ff', float('nan'), 0.))
        feed(0x301, bytes(7))
        node.publish_health()
        spin()
        assert len(odom) == 2 and len(imu) == 1 and len(joints) == 2
        assert node.stale == 1 and node.invalid == 2
        assert diagnostics[-1].status[0].level == DiagnosticStatus.WARN
        assert 'wheel missing/stale' in diagnostics[-1].status[0].message
        assert 'imu missing/stale' in diagnostics[-1].status[0].message
        feed(0x009, struct.pack('<ff', 0., 0.))
        feed(0x029, struct.pack('<ff', 0., 0.))
        assert node.wheels.rebases == 1
        spin()
        assert len(odom) == 2
        now[0] += .1
        feed(0x009, struct.pack('<ff', 0., 0.))
        feed(0x029, struct.pack('<ff', 0., 0.))
        spin()
        assert len(odom) == 3
        assert odom[-1].pose.pose.position.x == pytest.approx(odom[1].pose.pose.position.x)
        assert odom[-1].pose.covariance[0] > 1.
        # Only the realtime clock steps back; monotonic keeps moving forwards.
        pose_before = node.wheels.x
        steady = now[0] + .01
        monkeypatch.setattr(can_telemetry, 'monotonic', lambda: steady)
        now[0] -= 10.
        node.poll()
        assert node.clock_resets == 1
        assert node.wheels.x == pose_before
        feed(0x009, struct.pack('<ff', .01, 0.))
        feed(0x029, struct.pack('<ff', -.01, 0.))
        feed(0x301, struct.pack('<hhhH', 0, 0, 16384, 0))
        feed(0x302, struct.pack('<hhhH', 0, 0, 0, 0))
        spin()
        assert node.wheels.x == pose_before
        assert odom[-1].header.stamp.sec == int(now[0])
        assert imu[-1].header.stamp.sec == int(now[0])
    finally:
        node.close()
        assert bus.closed
        executor.shutdown()
        probe.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
