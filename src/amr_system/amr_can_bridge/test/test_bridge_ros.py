"""Opt-in ROS tests: no physical CAN, isolated domain 174, real dry-run executable."""

import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

import pytest

if os.environ.get('AMR_RUN_CAN_TESTS') != '1':
    pytest.skip('Set AMR_RUN_CAN_TESTS=1 to run ROS bridge tests', allow_module_level=True)

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from amr_can_bridge import cmd_vel_to_can


@pytest.fixture
def ros_context():
    assert os.environ.get('ROS_DOMAIN_ID') == '174'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    yield
    rclpy.shutdown()


def test_standalone_watchdog_and_sigterm(ros_context):
    node = rclpy.create_node('bridge_probe')
    publisher = node.create_publisher(Twist, '/cmd_vel', 1)
    received = []
    node.create_subscription(String, '/cmd_vel_to_can/tx_preview',
                             lambda msg: received.append(msg.data), 20)
    executable = Path(get_package_prefix('amr_can_bridge')) / 'lib/amr_can_bridge/cmd_vel_to_can'
    log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen(
        [str(executable), '--ros-args', '-p', 'dry_run:=true'],
        stdout=log, stderr=subprocess.STDOUT,
    )

    def drive(command=None, duration=0.55):
        received.clear()
        end = time.monotonic() + duration
        next_send = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if command is not None and now >= next_send:
                message = Twist()
                message.linear.x, message.angular.z = command
                publisher.publish(message)
                next_send = now + 0.04
            rclpy.spin_once(node, timeout_sec=0.01)
        assert len(received) >= 3
        result = []
        for line in received[-3:]:
            assert line.startswith('DRY_RUN can0 201#')
            result.append(struct.unpack('<ff', bytes.fromhex(line.split('#')[1].split()[0])))
        return result

    def expect(values, target):
        assert all(value == pytest.approx(target, abs=1e-6) for value in values)

    try:
        deadline = time.monotonic() + 15.0
        while publisher.get_subscription_count() < 1 or node.count_publishers(
                '/cmd_vel_to_can/tx_preview') < 1:
            assert process.poll() is None
            assert time.monotonic() < deadline
            rclpy.spin_once(node, timeout_sec=0.1)
        expect(drive((0.2, 0.4)), (0.0, 0.0))
        expect(drive((0.0, 0.0)), (0.0, 0.0))
        expect(drive((0.2, 0.4)), (0.2, 0.4))
        expect(drive(duration=0.85), (0.0, 0.0))
        assert 'reason=command_timeout' in received[-1]
        expect(drive((-0.2, -0.4)), (-0.2, -0.4))
        expect(drive((0.0, 0.0)), (0.0, 0.0))
        expect(drive((-1.0, -1.0)), (-0.2, -0.4))
        expect(drive((float('nan'), 0.0)), (0.0, 0.0))
        assert 'reason=invalid_command' in received[-1]
        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=5.0) == 0
        log.seek(0)
        assert '5 shutdown stop frames previewed' in log.read()
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        log.seek(0)
        print(log.read())
        log.close()
        node.destroy_node()


def test_send_failure_requires_recovery_stop_and_fresh_zero(ros_context, monkeypatch):
    class FakeSender:
        dry_run = False
        channel = 'fake'
        can_id = 0x201
        fail = False

        def __init__(self, **kwargs):
            self.sent = []

        def send(self, payload):
            if self.fail:
                raise OSError('simulated write failure')
            self.sent.append(payload)

        def close(self, **kwargs):
            return []

    monkeypatch.setattr(cmd_vel_to_can, 'CanSender', FakeSender)
    bridge = cmd_vel_to_can.CmdVelToCan()
    stop = Twist()
    moving = Twist()
    moving.linear.x = 0.2
    moving.angular.z = 0.4
    try:
        bridge.on_command(stop)
        bridge.on_command(moving)
        bridge.tick()
        assert bridge.sender.sent[-1].hex() == 'cdcc4c3ecdcccc3e'
        bridge.sender.fail = True
        bridge.tick()
        assert bridge.transport_fault
        bridge.on_command(stop)  # A zero before recovery must not rearm.
        bridge.sender.fail = False
        bridge.tick()
        assert bridge.sender.sent[-1] == bytes(8)
        assert not bridge.transport_fault
        bridge.on_command(moving)
        bridge.tick()
        assert bridge.sender.sent[-1] == bytes(8)
        bridge.on_command(stop)
        bridge.on_command(moving)
        bridge.tick()
        assert bridge.sender.sent[-1].hex() == 'cdcc4c3ecdcccc3e'
    finally:
        bridge.close()
        bridge.destroy_node()


@pytest.mark.parametrize('flags', [None, '0x280'])
def test_startup_interface_failure_exits_cleanly(monkeypatch, tmp_path, capfd, flags):
    from amr_can_bridge import can_sender
    monkeypatch.setattr(can_sender, 'NET_CLASS_PATH', tmp_path)
    interface = tmp_path / 'can0'
    interface.mkdir()
    if flags is not None:
        (interface / 'flags').write_text(flags)
    with pytest.raises(SystemExit) as error:
        cmd_vel_to_can.main(args=[])
    assert error.value.code == 1
    assert not rclpy.ok()
    output = capfd.readouterr()
    text = output.out + output.err
    assert 'LIVE CAN TX' not in text
    assert 'CAN TX failed' not in text
    assert 'can0' in text
