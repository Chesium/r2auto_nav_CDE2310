#!/usr/bin/env python3
"""Turn calibration smoke test.

Commands the robot to turn a fixed angle open-loop, then stops.
Measure the actual angle turned with a protractor / tape mark, then
compute the correction factor:

    TURN_SCALE = actual_degrees / commanded_degrees

Paste that value into aruco_dock_node.py as TURN_SCALE and multiply
all turn durations by it.

Usage:
    ros2 run g3_visual_servo turn_calibrate          # default: 90 deg CW
    ros2 run g3_visual_servo turn_calibrate --ros-args -p degrees:=180.0
    ros2 run g3_visual_servo turn_calibrate --ros-args -p degrees:=-90.0  # CCW
"""
import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import TwistStamped


MAX_ANGULAR = 0.5   # rad/s  — must match aruco_dock_node.py


class TurnCalibrate(Node):
    def __init__(self):
        super().__init__('turn_calibrate')

        self.declare_parameter(
            'degrees', 90.0,
            ParameterDescriptor(description='Degrees to turn. Positive=CW, negative=CCW'))

        degrees = self.get_parameter('degrees').value
        self._radians = math.radians(degrees)
        self._duration = abs(self._radians) / MAX_ANGULAR
        self._angular_z = -MAX_ANGULAR if degrees > 0 else MAX_ANGULAR

        self._pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self._start_ns = None
        self._done = False
        self._timer = self.create_timer(0.02, self._tick)  # 50 Hz

        self._countdown = 5  # seconds before starting
        self._counting = True

        self.get_logger().info(
            f'Turning {degrees:+.1f} deg  '
            f'(duration={self._duration:.3f}s  angular_z={self._angular_z:+.2f} rad/s)'
        )
        self.get_logger().info(
            'Starting in 5 seconds — open "ros2 topic echo /cmd_vel" in another terminal NOW.'
        )

    def _tick(self):
        now = self.get_clock().now().nanoseconds

        # countdown phase
        if self._counting:
            if self._start_ns is None:
                self._start_ns = now
            elapsed = (now - self._start_ns) / 1e9
            remaining = self._countdown - elapsed
            if remaining > 0:
                if int(remaining) != getattr(self, '_last_countdown', -1):
                    self._last_countdown = int(remaining)
                    self.get_logger().info(f'Starting in {int(remaining)+1}...')
                return
            self.get_logger().info('GO — turning now. Place reference mark!')
            self._counting = False
            self._start_ns = None  # reset for the actual turn
            return

        if self._start_ns is None:
            self._start_ns = now
            return

        elapsed = (now - self._start_ns) / 1e9

        if self._done:
            return

        if elapsed >= self._duration:
            self._send(0.0)
            self._done = True
            self.get_logger().info(
                f'DONE. Commanded {math.degrees(self._radians):+.1f} deg.\n'
                f'  -> Measure actual angle turned.\n'
                f'  -> TURN_SCALE = actual_deg / {math.degrees(self._radians):.1f}\n'
                f'  -> e.g. if robot turned 82 deg:  TURN_SCALE = {82/math.degrees(abs(self._radians)):.3f}'
            )
            raise SystemExit(0)
        else:
            self._send(self._angular_z)

    def _send(self, angular_z: float):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.angular.z = angular_z
        self._pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurnCalibrate()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
