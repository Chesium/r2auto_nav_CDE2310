#!/usr/bin/env python3
"""
nav2_mission_fsm.py
===================
FSM that coordinates frontier exploration and Nav2-based docking.

Unlike simple_mission_fsm (which uses aruco_dock_node's custom PI controller),
this FSM delegates docking entirely to Nav2's docking_server via the
/dock_robot action.  The aruco_dock_pose_publisher node must be running
alongside to feed /detected_dock_pose to the docking_server.

Mission:
    1. Explore the environment using the frontier_explorer node.
    2. When the aruco_dock_pose_publisher detects a marker (pose appears
       on /detected_dock_pose), stop exploration.
    3. Send a DockRobot action goal to Nav2's docking_server.
       Nav2 handles staging navigation + final approach — no custom /cmd_vel.
    4. When the action succeeds, the mission is complete.

States:
    INIT    — wait for frontier explorer service to be reachable
    EXPLORE — enable frontier exploration; watch /detected_dock_pose
    DOCK    — disable exploration; send /dock_robot action; wait for result
    DONE    — terminal

Required nodes running alongside:
    - frontier_explorer          (provides /exploration/set_enabled service)
    - aruco_dock_pose_publisher  (publishes /detected_dock_pose)
    - Nav2 stack with docking_server (provides /dock_robot action)

Parameters:
    dock_pose_topic    (str, default "/detected_dock_pose")
    dock_type          (str, default "aruco_dock")  — must match docking_server config
    navigate_to_staging_pose (bool, default true)
"""

from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool
from nav2_msgs.action import DockRobot

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import rclpy.time


class State(Enum):
    INIT = auto()
    EXPLORE = auto()
    DOCK = auto()
    DONE = auto()


class Nav2MissionFSM(Node):
    """Frontier-explore then Nav2-dock FSM."""

    TICK_PERIOD_SEC = 0.1  # 10 Hz

    def __init__(self) -> None:
        super().__init__("nav2_mission_fsm")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("dock_pose_topic", "/detected_dock_pose")
        self.declare_parameter("dock_type", "aruco_dock")
        self.declare_parameter("navigate_to_staging_pose", True)

        dock_pose_topic = self.get_parameter("dock_pose_topic").value
        self._dock_type = self.get_parameter("dock_type").value
        self._navigate_to_staging = self.get_parameter("navigate_to_staging_pose").value

        # ── State ─────────────────────────────────────────────────────────
        self._state: State = State.INIT
        self._explore_entered: bool = False
        self._explore_confirmed: bool = False
        self._dock_entered: bool = False

        # ── Detection flag + pose (written by subscription callback only) ─
        self._marker_detected: bool = False
        self._marker_map_pose: PoseStamped | None = None

        # ── Action result (written by action callbacks only) ─────────────
        self._dock_succeeded: bool = False
        self._dock_finished: bool = False
        self._retry_count: int = 0
        self._max_retries: int = 3

        # ── TF for converting robot pose to map frame ────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Subscribers ──────────────────────────────────────────────────
        # aruco_dock_pose_publisher publishes at ~30Hz with default QoS
        self.create_subscription(
            PoseStamped, dock_pose_topic, self._dock_pose_cb, 10
        )

        # ── Publishers ───────────────────────────────────────────────────
        self._state_pub = self.create_publisher(String, "/mission_state", 10)

        # ── Service client for frontier explorer ─────────────────────────
        self._explore_client = self.create_client(
            SetBool, "/exploration/set_enabled"
        )

        # ── Action client for Nav2 docking ───────────────────────────────
        self._dock_action_client = ActionClient(
            self, DockRobot, "/dock_robot"
        )

        # ── Heartbeat ────────────────────────────────────────────────────
        self.create_timer(self.TICK_PERIOD_SEC, self._tick)

        self.get_logger().info("nav2_mission_fsm started — state: INIT")

    # ═══════════════════════════ CALLBACKS ════════════════════════════════

    def _dock_pose_cb(self, msg: PoseStamped) -> None:
        """A marker was detected. Record the robot's current map-frame pose
        as the approximate dock location for the action goal."""
        if self._marker_detected:
            return  # already captured

        # Only trigger docking when the marker is close enough (< 2m).
        # This avoids switching out of EXPLORE when the marker is first
        # glimpsed from far away — let the explorer drive closer first.
        marker_dist = msg.pose.position.z  # Z = forward in camera optical frame
        if marker_dist <= 0.0 or marker_dist > 4.0:
            return

        # Get the robot's current pose in the map frame.
        # The docking_server needs a rough dock_pose in map frame so it can
        # navigate to the staging point.  The aruco_dock_pose_publisher
        # publishes in camera frame — we need map frame for the action goal.
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return  # TF not ready yet, wait for next frame

        # The dock is roughly where the robot is looking at.
        # Use the robot's pose + forward offset as a rough dock location.
        # The docking_server will refine with external detection.
        import math

        rx = tf_msg.transform.translation.x
        ry = tf_msg.transform.translation.y
        q = tf_msg.transform.rotation

        # Extract yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # marker_dist was already validated (0 < dist <= 2.0) at the top
        # of this callback, so use it directly.

        # Project dock position in map frame: robot pos + distance along heading
        dock_x = rx + marker_dist * math.cos(yaw)
        dock_y = ry + marker_dist * math.sin(yaw)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = dock_x
        pose.pose.position.y = dock_y
        pose.pose.position.z = 0.0
        # Dock orientation = the direction the robot approaches FROM.
        # The robot is facing the marker along its heading (yaw), so the
        # dock's yaw should match the robot's current heading — this tells
        # the docking_server "approach from this direction."
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._marker_map_pose = pose
        self._marker_detected = True
        self.get_logger().info(
            f"Marker detected! Estimated dock in map frame: "
            f"({dock_x:.2f}, {dock_y:.2f}), dist={marker_dist:.2f}m"
        )

    # ═══════════════════════════ TICK ═════════════════════════════════════

    def _tick(self) -> None:
        self._state_pub.publish(String(data=self._state.name))

        if self._state is State.INIT:
            self._handle_init()
        elif self._state is State.EXPLORE:
            self._handle_explore()
        elif self._state is State.DOCK:
            self._handle_dock()

    # ═══════════════════════════ STATE HANDLERS ═══════════════════════════

    def _handle_init(self) -> None:
        explore_ready = self._explore_client.service_is_ready()
        dock_ready = self._dock_action_client.server_is_ready()

        if explore_ready and dock_ready:
            self._transition_to(State.EXPLORE)
            return

        missing = []
        if not explore_ready:
            missing.append("/exploration/set_enabled")
        if not dock_ready:
            missing.append("/dock_robot action server")
        self.get_logger().warn(
            f"Waiting for: {', '.join(missing)}",
            throttle_duration_sec=2.0,
        )

    def _handle_explore(self) -> None:
        if not self._explore_confirmed:
            if not self._explore_entered:
                self._set_exploration(True)
                self._explore_entered = True
                self.get_logger().info(
                    "EXPLORE: exploration enable requested, watching for marker detection",
                    throttle_duration_sec=5.0,
                )
            return  # wait for confirmation before watching marker

        if self._marker_detected:
            self._transition_to(State.DOCK)

    def _handle_dock(self) -> None:
        if not self._dock_entered:
            self._set_exploration(False)
            self._send_dock_goal()
            self._dock_entered = True
            self.get_logger().info(
                "DOCK: exploration disabled, Nav2 dock_robot action sent"
            )

        if self._dock_finished:
            if self._dock_succeeded:
                self.get_logger().info("Docking SUCCEEDED — mission complete")
                self._transition_to(State.DONE)
            else:
                self._retry_count += 1
                if self._retry_count >= self._max_retries:
                    self.get_logger().error(
                        f"Docking FAILED after {self._max_retries} attempts — giving up"
                    )
                    self._transition_to(State.DONE)
                else:
                    self.get_logger().warn(
                        f"Docking FAILED (attempt {self._retry_count}/{self._max_retries}) "
                        f"— retrying with fresh detection"
                    )
                    self._marker_detected = False
                    self._marker_map_pose = None
                    self._dock_finished = False
                    self._dock_succeeded = False
                    self._dock_entered = False
                    self._explore_entered = False
                    self._explore_confirmed = False
                    self._transition_to(State.EXPLORE)

    # ═══════════════════════════ HELPERS ══════════════════════════════════

    def _transition_to(self, new_state: State) -> None:
        self.get_logger().info(f"FSM: {self._state.name} -> {new_state.name}")
        self._state = new_state

    def _set_exploration(self, enabled: bool) -> None:
        request = SetBool.Request()
        request.data = enabled
        future = self._explore_client.call_async(request)
        future.add_done_callback(
            lambda f: self._exploration_response_cb(f, enabled)
        )

    def _exploration_response_cb(self, future, enabled: bool) -> None:
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"set_exploration({enabled}) -> OK: {response.message}")
                if enabled:
                    self._explore_confirmed = True
            else:
                self.get_logger().warn(
                    f"set_exploration({enabled}) -> FAIL: {response.message} — will retry"
                )
                # Reset so _handle_explore retries on next tick
                self._explore_entered = False
        except Exception as exc:
            self.get_logger().error(f"set_exploration({enabled}) service error: {exc}")
            self._explore_entered = False

    def _send_dock_goal(self) -> None:
        """Send a DockRobot action goal to Nav2's docking_server."""
        goal = DockRobot.Goal()
        goal.use_dock_id = False
        goal.dock_pose = self._marker_map_pose
        goal.dock_type = self._dock_type
        goal.navigate_to_staging_pose = self._navigate_to_staging

        self.get_logger().info(
            f"Sending DockRobot goal: type={self._dock_type} "
            f"pose=({self._marker_map_pose.pose.position.x:.2f}, "
            f"{self._marker_map_pose.pose.position.y:.2f})"
        )

        send_future = self._dock_action_client.send_goal_async(
            goal, feedback_callback=self._dock_feedback_cb
        )
        send_future.add_done_callback(self._dock_goal_response_cb)

    def _dock_goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("DockRobot goal REJECTED by server")
            self._dock_finished = True
            self._dock_succeeded = False
            return

        self.get_logger().info("DockRobot goal accepted — docking in progress")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._dock_result_cb)

    def _dock_result_cb(self, future) -> None:
        result = future.result()
        status = result.status
        # action_msgs/GoalStatus: STATUS_SUCCEEDED = 4
        if status == 4:
            self.get_logger().info("DockRobot action SUCCEEDED")
            self._dock_succeeded = True
        else:
            self.get_logger().warn(f"DockRobot action finished with status {status}")
            self._dock_succeeded = False
        self._dock_finished = True

    def _dock_feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Dock feedback: state={fb.state} "
            f"time={fb.docking_time.sec}s retries={fb.num_retries}",
            throttle_duration_sec=2.0,
        )

    def _log_service_response(self, future, label: str) -> None:
        try:
            response = future.result()
            status = "OK" if response.success else "FAIL"
            self.get_logger().info(f"{label} -> {status}: {response.message}")
        except Exception as exc:
            self.get_logger().error(f"{label} service error: {exc}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Nav2MissionFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
