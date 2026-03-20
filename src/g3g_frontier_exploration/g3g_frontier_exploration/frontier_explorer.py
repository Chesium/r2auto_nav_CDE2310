"""Manual-trigger frontier exploration node for Nav2 simulation."""

from __future__ import annotations

import math
from typing import Iterable

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import SetBool
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from g3g_frontier_exploration.frontier_utils import ExpiringBlacklist
from g3g_frontier_exploration.frontier_utils import GridMeta
from g3g_frontier_exploration.frontier_utils import euclidean_distance
from g3g_frontier_exploration.frontier_utils import extract_frontier_clusters
from g3g_frontier_exploration.frontier_utils import index_to_world
from g3g_frontier_exploration.frontier_utils import make_goal_key


class FrontierExplorer(BasicNavigator):
    """Frontier exploration built on top of nav2_simple_commander."""

    def __init__(self) -> None:
        super().__init__(node_name="frontier_explorer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("frontier_min_cluster_size", 10)
        self.declare_parameter("min_goal_distance", 0.35)
        self.declare_parameter("planning_rate_hz", 1.0)
        self.declare_parameter("progress_rate_hz", 2.0)
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("blacklist_ttl_sec", 60.0)
        self.declare_parameter("completion_patience_cycles", 5)
        self.declare_parameter("autostart", False)

        self.map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self.global_frame = self.get_parameter("global_frame").get_parameter_value().string_value
        self.robot_base_frame = (
            self.get_parameter("robot_base_frame").get_parameter_value().string_value
        )
        self.occupied_threshold = (
            self.get_parameter("occupied_threshold").get_parameter_value().integer_value
        )
        self.frontier_min_cluster_size = (
            self.get_parameter("frontier_min_cluster_size").get_parameter_value().integer_value
        )
        self.min_goal_distance = (
            self.get_parameter("min_goal_distance").get_parameter_value().double_value
        )
        self.planning_rate_hz = (
            self.get_parameter("planning_rate_hz").get_parameter_value().double_value
        )
        self.progress_rate_hz = (
            self.get_parameter("progress_rate_hz").get_parameter_value().double_value
        )
        self.goal_timeout_sec = (
            self.get_parameter("goal_timeout_sec").get_parameter_value().double_value
        )
        self.blacklist_ttl_sec = (
            self.get_parameter("blacklist_ttl_sec").get_parameter_value().double_value
        )
        self.completion_patience_cycles = (
            self.get_parameter("completion_patience_cycles")
            .get_parameter_value()
            .integer_value
        )
        self.autostart = self.get_parameter("autostart").get_parameter_value().bool_value

        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._current_map: OccupancyGrid | None = None
        self._blacklist = ExpiringBlacklist()
        self._enabled = False
        self._system_ready = False
        self._no_frontier_cycles = 0
        self._autostart_consumed = False
        self._active_goal_pose: PoseStamped | None = None
        self._active_goal_key: tuple[int, int] | None = None
        self._active_goal_start_sec: float | None = None
        self._cancel_active_goal_requested = False
        self._last_startup_check_sec = 0.0
        self._last_planning_tick_sec = 0.0
        self._last_progress_tick_sec = 0.0
        self._startup_interval_sec = 0.5
        self._planning_interval_sec = 1.0 / max(self.planning_rate_hz, 0.1)
        self._progress_interval_sec = 1.0 / max(self.progress_rate_hz, 0.1)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            map_qos,
        )
        self._frontier_publisher = self.create_publisher(
            MarkerArray,
            "/exploration/frontiers",
            10,
        )
        self._goal_publisher = self.create_publisher(
            PoseStamped,
            "/exploration/current_goal",
            10,
        )
        self._toggle_service = self.create_service(
            SetBool,
            "/exploration/set_enabled",
            self._handle_set_enabled,
        )

        self._publish_markers([], None)
        self.info("Frontier explorer initialized and waiting in idle mode.")

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._current_map = msg

    def _handle_set_enabled(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            ready, reason = self._readiness_state()
            if not ready:
                response.success = False
                response.message = f"Explorer not ready: {reason}"
                return response

            if self._enabled:
                response.success = True
                response.message = "Exploration is already running."
                return response

            self._enabled = True
            self._no_frontier_cycles = 0
            response.success = True
            response.message = "Exploration enabled."
            self.info("Exploration enabled by service call.")
            return response

        self._enabled = False
        self._no_frontier_cycles = 0
        self._cancel_active_goal_requested = True
        self.info("Exploration disable was requested by service call.")
        response.success = True
        response.message = "Exploration disabled."
        return response

    def tick(self) -> None:
        now_sec = self._now_seconds()

        if now_sec - self._last_startup_check_sec >= self._startup_interval_sec:
            self._last_startup_check_sec = now_sec
            self._startup_tick()

        if self._cancel_active_goal_requested:
            self._cancel_active_goal_requested = False
            self._disable_exploration("Exploration disabled.", clear_markers=True)

        if now_sec - self._last_progress_tick_sec >= self._progress_interval_sec:
            self._last_progress_tick_sec = now_sec
            self._progress_tick()

        if now_sec - self._last_planning_tick_sec >= self._planning_interval_sec:
            self._last_planning_tick_sec = now_sec
            self._planning_tick()

    def _startup_tick(self) -> None:
        ready, reason = self._readiness_state()
        if ready and not self._system_ready:
            self._system_ready = True
            self.info("Explorer is ready: map, transform, and Nav2 servers are available.")
        elif not ready:
            self._system_ready = False
            self.debug(f"Explorer is still waiting for readiness: {reason}")

        if self.autostart and self._system_ready and not self._autostart_consumed and not self._enabled:
            self._autostart_consumed = True
            self._enabled = True
            self._no_frontier_cycles = 0
            self.info("Autostart is enabled. Beginning exploration.")

    def _readiness_state(self) -> tuple[bool, str]:
        if self._current_map is None:
            return (False, "no map received yet")

        if self._lookup_robot_pose() is None:
            return (False, f"missing transform {self.global_frame}->{self.robot_base_frame}")

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=0.0):
            return (False, "navigate_to_pose action server unavailable")

        if not self.compute_path_to_pose_client.wait_for_server(timeout_sec=0.0):
            return (False, "compute_path_to_pose action server unavailable")

        return (True, "ready")

    def _planning_tick(self) -> None:
        if not self._enabled or self._active_goal_pose is not None:
            return

        ready, _ = self._readiness_state()
        if not ready:
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None or self._current_map is None:
            return

        now_sec = self._now_seconds()
        self._blacklist.prune(now_sec)

        map_meta = GridMeta(
            width=self._current_map.info.width,
            height=self._current_map.info.height,
            resolution=self._current_map.info.resolution,
            origin_x=self._current_map.info.origin.position.x,
            origin_y=self._current_map.info.origin.position.y,
        )

        clusters = extract_frontier_clusters(
            self._current_map.data,
            map_meta,
            self.occupied_threshold,
            self.frontier_min_cluster_size,
        )

        self.info(f"clusters:{clusters}")

        ranked_candidates = []
        robot_xy = (
            robot_pose.pose.position.x,
            robot_pose.pose.position.y,
        )

        for cluster in clusters:
            goal_xy = cluster.goal_xy
            if euclidean_distance(robot_xy, goal_xy) < self.min_goal_distance:
                self.info(f"kick {goal_xy} @1")
                continue

            goal_key = make_goal_key(*goal_xy, resolution=map_meta.resolution)
            if self._blacklist.contains(goal_key, now_sec):
                self.info(f"kick {goal_xy} @2")
                continue

            goal_pose = self._make_pose_stamped(goal_xy[0], goal_xy[1])
            path = self.getPath(robot_pose, goal_pose, use_start=True)

            if path is None or not path.poses:
                self._blacklist.add(goal_key, now_sec, self.blacklist_ttl_sec, payload=goal_xy)
                self.info(f"kick {goal_xy} @3")
                continue

            ranked_candidates.append(
                (
                    self._path_length(path),
                    -cluster.size,
                    goal_pose,
                    goal_key,
                )
            )

        self._publish_markers(clusters, self._active_goal_pose)

        if not ranked_candidates:
            self._no_frontier_cycles += 1
            if self._no_frontier_cycles >= self.completion_patience_cycles:
                self.info("No valid frontiers remain. Exploration is complete.")
                self._disable_exploration(
                    "Exploration completed successfully.",
                    clear_markers=True,
                )
            return

        self._no_frontier_cycles = 0
        ranked_candidates.sort(key=lambda item: (item[0], item[1]))
        _, _, goal_pose, goal_key = ranked_candidates[0]

        if not self.goToPose(goal_pose):
            self.warn("Nav2 rejected the selected exploration goal. Blacklisting it temporarily.")
            self._blacklist.add(
                goal_key,
                now_sec,
                self.blacklist_ttl_sec,
                payload=(goal_pose.pose.position.x, goal_pose.pose.position.y),
            )
            self.clearAllCostmaps()
            self._publish_markers(clusters, None)
            return

        self._active_goal_pose = goal_pose
        self._active_goal_key = goal_key
        self._active_goal_start_sec = now_sec
        self._goal_publisher.publish(goal_pose)
        self._publish_markers(clusters, goal_pose)
        self.info(
            "Sent exploration goal at "
            f"({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})."
        )

    def _progress_tick(self) -> None:
        if not self._enabled or self._active_goal_pose is None:
            return

        now_sec = self._now_seconds()
        if self._active_goal_start_sec is not None:
            elapsed = now_sec - self._active_goal_start_sec
            if elapsed > self.goal_timeout_sec:
                self.warn("Active frontier goal timed out. Canceling and blacklisting it.")
                self.cancelTask()
                self.clearAllCostmaps()
                self._blacklist_active_goal(now_sec)
                self._clear_active_goal_state()
                return

        if not self.isTaskComplete():
            return

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.info("Reached active frontier goal successfully.")
            self._clear_active_goal_state()
            return

        if result == TaskResult.CANCELED and not self._enabled:
            self._clear_active_goal_state()
            return

        self.warn(f"Frontier navigation finished with result {result.name}. Retrying elsewhere.")
        self.clearAllCostmaps()
        self._blacklist_active_goal(now_sec)
        self._clear_active_goal_state()

    def _clear_active_goal_state(self) -> None:
        self._active_goal_pose = None
        self._active_goal_key = None
        self._active_goal_start_sec = None
        self._publish_markers([], None)

    def _blacklist_active_goal(self, now_sec: float) -> None:
        if self._active_goal_key is None or self._active_goal_pose is None:
            return
        goal_xy = (
            self._active_goal_pose.pose.position.x,
            self._active_goal_pose.pose.position.y,
        )
        self._blacklist.add(
            self._active_goal_key,
            now_sec,
            self.blacklist_ttl_sec,
            payload=goal_xy,
        )

    def _disable_exploration(self, message: str, clear_markers: bool) -> None:
        self._enabled = False
        self._no_frontier_cycles = 0
        if self._active_goal_pose is not None:
            self.cancelTask()
        self._clear_active_goal_state()
        if clear_markers:
            self._publish_markers([], None)
        self.info(message)

    def _lookup_robot_pose(self) -> PoseStamped | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time(),
            )
        except TransformException:
            return None

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = self.global_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _make_pose_stamped(self, x: float, y: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def _path_length(self, path: Path) -> float:
        total = 0.0
        for pose_a, pose_b in zip(path.poses, path.poses[1:]):
            dx = pose_b.pose.position.x - pose_a.pose.position.x
            dy = pose_b.pose.position.y - pose_a.pose.position.y
            total += math.hypot(dx, dy)
        return total

    def _publish_markers(
        self,
        clusters: Iterable,
        active_goal: PoseStamped | None,
    ) -> None:
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = self.global_frame
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        frontier_marker = Marker()
        frontier_marker.header.frame_id = self.global_frame
        frontier_marker.header.stamp = self.get_clock().now().to_msg()
        frontier_marker.ns = "frontiers"
        frontier_marker.id = 1
        frontier_marker.type = Marker.POINTS
        frontier_marker.action = Marker.ADD
        frontier_marker.scale.x = 0.05
        frontier_marker.scale.y = 0.05
        frontier_marker.color.r = 0.0
        frontier_marker.color.g = 0.75
        frontier_marker.color.b = 1.0
        frontier_marker.color.a = 0.9

        if self._current_map is not None:
            map_meta = GridMeta(
                width=self._current_map.info.width,
                height=self._current_map.info.height,
                resolution=self._current_map.info.resolution,
                origin_x=self._current_map.info.origin.position.x,
                origin_y=self._current_map.info.origin.position.y,
            )
            for cluster in clusters:
                for cell in cluster.cells:
                    x, y = index_to_world(map_meta, cell)
                    frontier_marker.points.append(Point(x=x, y=y, z=0.0))

        marker_array.markers.append(frontier_marker)

        blacklist_marker = Marker()
        blacklist_marker.header.frame_id = self.global_frame
        blacklist_marker.header.stamp = self.get_clock().now().to_msg()
        blacklist_marker.ns = "blacklisted_goals"
        blacklist_marker.id = 2
        blacklist_marker.type = Marker.SPHERE_LIST
        blacklist_marker.action = Marker.ADD
        blacklist_marker.scale.x = 0.16
        blacklist_marker.scale.y = 0.16
        blacklist_marker.scale.z = 0.16
        blacklist_marker.color.r = 0.55
        blacklist_marker.color.g = 0.55
        blacklist_marker.color.b = 0.55
        blacklist_marker.color.a = 0.8

        now_sec = self._now_seconds()
        for point_xy in self._blacklist.active_entries(now_sec).values():
            if point_xy is None:
                continue
            blacklist_marker.points.append(Point(x=point_xy[0], y=point_xy[1], z=0.0))

        marker_array.markers.append(blacklist_marker)

        active_marker = Marker()
        active_marker.header.frame_id = self.global_frame
        active_marker.header.stamp = self.get_clock().now().to_msg()
        active_marker.ns = "active_goal"
        active_marker.id = 3
        active_marker.type = Marker.SPHERE
        active_marker.action = Marker.ADD
        active_marker.scale.x = 0.22
        active_marker.scale.y = 0.22
        active_marker.scale.z = 0.22
        active_marker.color.r = 0.1
        active_marker.color.g = 1.0
        active_marker.color.b = 0.2
        active_marker.color.a = 0.95

        if active_goal is not None:
            active_marker.pose = active_goal.pose
            marker_array.markers.append(active_marker)

        self._frontier_publisher.publish(marker_array)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok() and node._active_goal_pose is not None:
            node.cancelTask()
        node.destroy_node()
        rclpy.shutdown()
