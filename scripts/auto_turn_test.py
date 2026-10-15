#!/usr/bin/env python3
"""While AUTO is enabled, repeat right 2 s then left 2 s.

Run with: python3 scripts/auto_turn_test.py
Publishes only /cmd_vel_auto (Twist); control_mux owns the final /cmd_vel.
RB OFF cancels the sequence. RB ON starts a new sequence from the right turn.
"""

from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class AutoTurnTest(Node):
    def __init__(self):
        super().__init__('auto_turn_test')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_auto', 1)
        self.mode = 'MANUAL'
        self.started = None
        self.phase = None
        self.mode_subscription = self.create_subscription(
            String, '/control_mux/mode', self.on_mode,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.timer = self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.get_logger().info('Waiting for AUTO (RB): repeat right 2 s, left 2 s. v=0, |w|=0.20.')

    def on_mode(self, message):
        if message.data != self.mode:
            self.mode = message.data
            self.started = monotonic() if self.mode == 'AUTO' else None
            self.phase = None
            self.publisher.publish(Twist())

    def tick(self):
        message = Twist()
        elapsed = monotonic() - self.started if self.started is not None else None
        if elapsed is None:
            phase = 'Waiting for AUTO'
        elif elapsed % 4.0 < 2.0:
            phase = 'Right turn'
            message.angular.z = -0.20
        else:
            phase = 'Left turn'
            message.angular.z = 0.20
        self.publisher.publish(message)
        if phase != self.phase:
            self.phase = phase
            self.get_logger().info(phase)


def main():
    rclpy.init()
    node = AutoTurnTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
