"""Standalone docking test — no Nav2, no SLAM, no TF needed.

Launches alongside aruco_dock_node. Automatically calls dock_to_a (marker 42)
or dock_to_b (marker 67), monitors progress, and reports result.

Usage:
  ros2 run g3_visual_servo dock_test              # defaults to marker 42
  ros2 run g3_visual_servo dock_test --ros-args -p marker:=67
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class DockTest(Node):

    def __init__(self):
        super().__init__('dock_test')
        self.declare_parameter('marker', 42)
        self._marker = self.get_parameter('marker').value

        if self._marker == 42:
            self._client = self.create_client(Trigger, '/aruco_dock/dock_to_a')
        elif self._marker == 67:
            self._client = self.create_client(Trigger, '/aruco_dock/dock_to_b')
        else:
            self.get_logger().error(f'Unknown marker {self._marker}, use 42 or 67')
            raise SystemExit(1)

        self.create_subscription(Bool, '/aruco_dock/done', self._done_cb, 10)
        self._called = False
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f'Dock test — will call dock to marker {self._marker}')

    def _tick(self):
        if self._called:
            return
        if not self._client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info('Waiting for aruco_dock service...')
            return
        self._called = True
        self.get_logger().info(f'Calling dock to marker {self._marker}...')
        future = self._client.call_async(Trigger.Request())
        future.add_done_callback(self._service_cb)

    def _service_cb(self, future):
        result = future.result()
        self.get_logger().info(f'Service response: {result.message}')

    def _done_cb(self, msg):
        if msg.data:
            self.get_logger().info('DOCKING COMPLETE!')
            raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = DockTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
