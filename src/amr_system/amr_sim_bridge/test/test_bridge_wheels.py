"""Test the actual simulated telemetry callback without Isaac, DDS or actuators."""
import math
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from amr_can_bridge.telemetry import WheelOdometry
from amr_sim_bridge.bridge import SimBridge
from amr_sim_bridge.model import PhysXWheelAngle


def test_raw_joint_angles_are_preserved_and_wheel_odom_crosses_boundary():
    sent = []
    bridge = SimpleNamespace(
        sim_time=0., joint_names=['left_drive_wheel_joint', 'right_drive_wheel_joint'],
        joint_pub='joint', wheel_pub='wheel', counts={'wheel': 0},
        model={'linear_velocity_variance': .0025, 'angular_velocity_variance': .01},
        odometry=WheelOdometry(left_sign=1., right_sign=1.),
        wheel_angles={side: PhysXWheelAngle() for side in ['left', 'right']},
        delayed=lambda publisher, message: sent.append((publisher, message)))
    SimBridge.wheels(bridge, [6.1, 6.1], [3., 3.])
    bridge.sim_time = .1
    SimBridge.wheels(bridge, [6.4-4*math.pi]*2, [3., 3.])
    joints = [msg for publisher, msg in sent if publisher == 'joint']
    wheels = [msg for publisher, msg in sent if publisher == 'wheel']
    assert len(joints) == len(wheels) == bridge.counts['wheel'] == 2
    assert list(joints[1].position) == pytest.approx([6.4-4*math.pi]*2)
    assert wheels[1].pose.pose.position.x == pytest.approx(.3*.086)
    assert wheels[1].twist.twist.linear.x == pytest.approx(3*.086)
    assert bridge.odometry.rebases == 0
