#!/usr/bin/env python3
"""Smoke test: publish cmd_vel for 3 seconds then stop.
Run with: python3 cmd_vel_test.py
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelTest(Node):
    def __init__(self):
        super().__init__('cmd_vel_test')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._count = 0
        self._timer = self.create_timer(0.1, self._tick)  # 10 Hz
        self.get_logger().info('Publishing forward 0.1 m/s for 3 seconds...')

    def _tick(self):
        self._count += 1
        if self._count <= 30:  # 3 seconds at 10 Hz
            cmd = Twist()
            cmd.linear.x = 0.1
            self._pub.publish(cmd)
            self.get_logger().info(f'[{self._count}/30] linear.x=0.1')
        elif self._count == 31:
            self._pub.publish(Twist())
            self.get_logger().info('STOP sent. Robot should have moved ~0.3m.')
        else:
            raise SystemExit(0)


def main():
    rclpy.init()
    node = CmdVelTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
