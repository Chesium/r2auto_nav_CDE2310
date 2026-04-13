#!/usr/bin/env python3
"""
dock_action_client.py
=====================
Reads the marker's detected pose, transforms it to the map frame via TF,
and sends a DockRobot action goal with the correct dock position.

Usage:
    ros2 run g3_visual_servo dock_action_client
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped


class DockActionClient(Node):
    def __init__(self) -> None:
        super().__init__("dock_action_client")

        self.declare_parameter("dock_type", "aruco_dock")
        self.declare_parameter("dock_pose_topic", "/detected_dock_pose")
        self.declare_parameter("navigate_to_staging", False)
        self._dock_type = str(self.get_parameter("dock_type").value)
        self._navigate_to_staging = bool(
            self.get_parameter("navigate_to_staging").value
        )

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Action client
        self._action_client = ActionClient(self, DockRobot, "/dock_robot")

        # Subscribe to detected dock pose — grab ONE message then send goal
        self._got_pose = False
        topic = str(self.get_parameter("dock_pose_topic").value)
        self.create_subscription(PoseStamped, topic, self._pose_cb, 10)

        self.get_logger().info(
            f"Waiting for marker detection on {topic}..."
        )

    def _pose_cb(self, msg: PoseStamped) -> None:
        if self._got_pose:
            return
        self._got_pose = True

        # Transform detected pose from camera frame to map frame
        try:
            transform = self._tf_buffer.lookup_transform(
                "map", msg.header.frame_id, msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            pose_map = do_transform_pose_stamped(msg, transform)
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            self._got_pose = False
            return

        self.get_logger().info(
            f"Marker in map frame: x={pose_map.pose.position.x:.3f} "
            f"y={pose_map.pose.position.y:.3f} z={pose_map.pose.position.z:.3f}"
        )

        # Send dock goal
        self._send_dock_goal(pose_map)

    def _send_dock_goal(self, pose_map: PoseStamped) -> None:
        self.get_logger().info("Waiting for /dock_robot action server...")
        self._action_client.wait_for_server()

        goal = DockRobot.Goal()
        goal.use_dock_id = False
        goal.dock_type = self._dock_type
        goal.navigate_to_staging_pose = self._navigate_to_staging
        goal.dock_pose = pose_map

        self.get_logger().info(
            f"Sending DockRobot goal: dock_type={self._dock_type} "
            f"staging={self._navigate_to_staging} "
            f"pose=({pose_map.pose.position.x:.2f}, "
            f"{pose_map.pose.position.y:.2f})"
        )

        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            rclpy.shutdown()
            return
        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        state_names = {0: "NAV_TO_STAGING", 1: "INITIAL_PERCEPTION",
                       2: "CONTROLLING", 3: "AT_DOCK"}
        state = state_names.get(fb.state, f"UNKNOWN({fb.state})")
        self.get_logger().info(
            f"state={state} time={fb.docking_time.sec}s retries={fb.num_retries}",
            throttle_duration_sec=2.0,
        )

    def _result_cb(self, future) -> None:
        result = future.result().result
        if result.success:
            self.get_logger().info("DOCKING SUCCEEDED!")
        else:
            self.get_logger().error(
                f"DOCKING FAILED: error_code={result.error_code} "
                f"retries={result.num_retries}"
            )
        rclpy.shutdown()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DockActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == "__main__":
    main()
