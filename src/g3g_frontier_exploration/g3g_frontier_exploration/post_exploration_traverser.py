"""Traverse reachable free space after frontier exploration completes."""

from __future__ import annotations

import math

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
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from g3g_frontier_exploration.frontier_utils import compute_clearance_to_occupied
from g3g_frontier_exploration.frontier_utils import count_cell_types_in_window
from g3g_frontier_exploration.frontier_utils import ExpiringBlacklist
from g3g_frontier_exploration.frontier_utils import extract_connected_free_component
from g3g_frontier_exploration.frontier_utils import extract_frontier_clusters
from g3g_frontier_exploration.frontier_utils import GridMeta
from g3g_frontier_exploration.frontier_utils import index_to_cell
from g3g_frontier_exploration.frontier_utils import index_to_world
from g3g_frontier_exploration.frontier_utils import make_goal_key
from g3g_frontier_exploration.frontier_utils import world_to_index


class PostExplorationTraverser(BasicNavigator):
    """Pick viewpoint goals after exploration has exhausted frontiers."""

    def __init__(self) -> None:
        super().__init__(node_name="post_exploration_traverser")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("exploration_complete_topic", "/exploration_complete")
        self.declare_parameter("exploration_enable_service", "/exploration/set_enabled")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("autostart_after_exploration_complete", True)
        self.declare_parameter("planning_rate_hz", 0.5)
        self.declare_parameter("progress_rate_hz", 2.0)
        self.declare_parameter("goal_timeout_sec", 45.0)
        self.declare_parameter("sample_spacing", 1.25)
        self.declare_parameter("min_goal_distance", 1.0)
        self.declare_parameter("max_path_evaluations", 24)
        self.declare_parameter("candidate_clearance_radius", 4)
        self.declare_parameter("clearance_lookup_radius", 8)
        self.declare_parameter("viewpoint_window_radius_cells", 8)
        self.declare_parameter("occupied_view_weight", 1.0)
        self.declare_parameter("unknown_view_weight", 2.5)
        self.declare_parameter("clearance_weight", 0.35)
        self.declare_parameter("path_length_weight", 0.8)
        self.declare_parameter("revisit_ttl_sec", 180.0)
        self.declare_parameter("failed_goal_ttl_sec", 60.0)
        self.declare_parameter("revisit_position_resolution", 0.75)
        self.declare_parameter("heading_search_radius_cells", 12)
        self.declare_parameter("frontier_reactivation_enabled", True)
        self.declare_parameter("frontier_reactivation_cycles", 3)
        self.declare_parameter("frontier_min_cluster_size", 8)
        self.declare_parameter("frontier_candidate_clearance_radius", 1)
        self.declare_parameter("frontier_window_radius_cells", 2)
        self.declare_parameter("min_frontier_free_neighbors", 3)
        self.declare_parameter("max_frontier_occupied_ratio", 0.45)
        self.declare_parameter("min_reachable_unknown_cells", 3)
        self.declare_parameter("min_frontier_unknown_span", 2)

        self.map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self.exploration_complete_topic = (
            self.get_parameter("exploration_complete_topic")
            .get_parameter_value()
            .string_value
        )
        self.exploration_enable_service = (
            self.get_parameter("exploration_enable_service")
            .get_parameter_value()
            .string_value
        )
        self.global_frame = (
            self.get_parameter("global_frame").get_parameter_value().string_value
        )
        self.robot_base_frame = (
            self.get_parameter("robot_base_frame").get_parameter_value().string_value
        )
        self.occupied_threshold = (
            self.get_parameter("occupied_threshold").get_parameter_value().integer_value
        )
        self.autostart_after_exploration_complete = (
            self.get_parameter("autostart_after_exploration_complete")
            .get_parameter_value()
            .bool_value
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
        self.sample_spacing = (
            self.get_parameter("sample_spacing").get_parameter_value().double_value
        )
        self.min_goal_distance = (
            self.get_parameter("min_goal_distance").get_parameter_value().double_value
        )
        self.max_path_evaluations = (
            self.get_parameter("max_path_evaluations").get_parameter_value().integer_value
        )
        self.candidate_clearance_radius = (
            self.get_parameter("candidate_clearance_radius")
            .get_parameter_value()
            .integer_value
        )
        self.clearance_lookup_radius = (
            self.get_parameter("clearance_lookup_radius")
            .get_parameter_value()
            .integer_value
        )
        self.viewpoint_window_radius_cells = (
            self.get_parameter("viewpoint_window_radius_cells")
            .get_parameter_value()
            .integer_value
        )
        self.occupied_view_weight = (
            self.get_parameter("occupied_view_weight")
            .get_parameter_value()
            .double_value
        )
        self.unknown_view_weight = (
            self.get_parameter("unknown_view_weight").get_parameter_value().double_value
        )
        self.clearance_weight = (
            self.get_parameter("clearance_weight").get_parameter_value().double_value
        )
        self.path_length_weight = (
            self.get_parameter("path_length_weight").get_parameter_value().double_value
        )
        self.revisit_ttl_sec = (
            self.get_parameter("revisit_ttl_sec").get_parameter_value().double_value
        )
        self.failed_goal_ttl_sec = (
            self.get_parameter("failed_goal_ttl_sec").get_parameter_value().double_value
        )
        self.revisit_position_resolution = (
            self.get_parameter("revisit_position_resolution")
            .get_parameter_value()
            .double_value
        )
        self.heading_search_radius_cells = (
            self.get_parameter("heading_search_radius_cells")
            .get_parameter_value()
            .integer_value
        )
        self.frontier_reactivation_enabled = (
            self.get_parameter("frontier_reactivation_enabled")
            .get_parameter_value()
            .bool_value
        )
        self.frontier_reactivation_cycles = (
            self.get_parameter("frontier_reactivation_cycles")
            .get_parameter_value()
            .integer_value
        )
        self.frontier_min_cluster_size = (
            self.get_parameter("frontier_min_cluster_size")
            .get_parameter_value()
            .integer_value
        )
        self.frontier_candidate_clearance_radius = (
            self.get_parameter("frontier_candidate_clearance_radius")
            .get_parameter_value()
            .integer_value
        )
        self.frontier_window_radius_cells = (
            self.get_parameter("frontier_window_radius_cells")
            .get_parameter_value()
            .integer_value
        )
        self.min_frontier_free_neighbors = (
            self.get_parameter("min_frontier_free_neighbors")
            .get_parameter_value()
            .integer_value
        )
        self.max_frontier_occupied_ratio = (
            self.get_parameter("max_frontier_occupied_ratio")
            .get_parameter_value()
            .double_value
        )
        self.min_reachable_unknown_cells = (
            self.get_parameter("min_reachable_unknown_cells")
            .get_parameter_value()
            .integer_value
        )
        self.min_frontier_unknown_span = (
            self.get_parameter("min_frontier_unknown_span")
            .get_parameter_value()
            .integer_value
        )

        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._current_map: OccupancyGrid | None = None
        self._exploration_complete = False
        self._enabled = False
        self._system_ready = False
        self._active_goal_pose: PoseStamped | None = None
        self._active_goal_start_sec: float | None = None
        self._goal_blacklist = ExpiringBlacklist()
        self._cancel_active_goal_requested = False
        self._last_startup_check_sec = 0.0
        self._last_planning_tick_sec = 0.0
        self._last_progress_tick_sec = 0.0
        self._startup_interval_sec = 0.5
        self._planning_interval_sec = 1.0 / max(self.planning_rate_hz, 0.1)
        self._progress_interval_sec = 1.0 / max(self.progress_rate_hz, 0.1)
        self._frontier_reactivation_counter = 0
        self._resume_future: Future | None = None

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            map_qos,
        )
        self._completion_subscription = self.create_subscription(
            Bool,
            self.exploration_complete_topic,
            self._completion_callback,
            map_qos,
        )
        self._goal_publisher = self.create_publisher(
            PoseStamped,
            "/post_exploration/current_goal",
            10,
        )
        self._exploration_toggle_client = self.create_client(
            SetBool,
            self.exploration_enable_service,
        )

        self.info(
            "Post-exploration traverser initialized and waiting for "
            "/exploration_complete."
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._current_map = msg

    def _completion_callback(self, msg: Bool) -> None:
        self._exploration_complete = msg.data
        if msg.data:
            if self.autostart_after_exploration_complete and not self._enabled:
                self._enabled = True
                self._frontier_reactivation_counter = 0
                self.info(
                    "Exploration completed. Starting post-exploration traversal."
                )
            return

        if self._enabled:
            self._cancel_active_goal_requested = True
            self.info(
                "Exploration completion reset. Pausing post-exploration traversal."
            )

    def tick(self) -> None:
        now_sec = self._now_seconds()

        if now_sec - self._last_startup_check_sec >= self._startup_interval_sec:
            self._last_startup_check_sec = now_sec
            self._startup_tick()

        if self._cancel_active_goal_requested:
            self._cancel_active_goal_requested = False
            self._disable_traversal("Traversal paused.", clear_goal=True)

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
            self.info(
                "Traverser is ready: map, transform, and Nav2 servers are available."
            )
        elif not ready:
            self._system_ready = False
            self.debug(f"Traverser is still waiting for readiness: {reason}")

        if (
            self.autostart_after_exploration_complete
            and self._exploration_complete
            and self._system_ready
            and not self._enabled
        ):
            self._enabled = True
            self.info(
                "Traversal autostart is enabled. Beginning post-exploration motion."
            )

    def _readiness_state(self) -> tuple[bool, str]:
        if self._current_map is None:
            return (False, "no map received yet")

        if self._lookup_robot_pose() is None:
            return (
                False,
                f"missing transform {self.global_frame}->{self.robot_base_frame}",
            )

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

        if self._should_reactivate_frontier(robot_pose):
            self._request_frontier_resume()
            return

        goal_pose = self._select_goal_pose(robot_pose)
        if goal_pose is None:
            self.debug("No post-exploration traversal goal available this cycle.")
            return

        if not self.goToPose(goal_pose):
            self.warn(
                "Nav2 rejected the selected post-exploration goal. Blacklisting it temporarily."
            )
            self._blacklist_pose(goal_pose, self.failed_goal_ttl_sec)
            self.clearAllCostmaps()
            return

        self._active_goal_pose = goal_pose
        self._active_goal_start_sec = self._now_seconds()
        self._goal_publisher.publish(goal_pose)
        self.info(
            "Sent post-exploration goal at "
            f"({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})."
        )

    def _should_reactivate_frontier(self, robot_pose: PoseStamped) -> bool:
        if not self.frontier_reactivation_enabled or self._current_map is None:
            return False

        robot_xy = (
            robot_pose.pose.position.x,
            robot_pose.pose.position.y,
        )
        map_meta = self._map_meta()
        clusters = extract_frontier_clusters(
            self._current_map.data,
            map_meta,
            self.occupied_threshold,
            self.frontier_min_cluster_size,
            window_radius_cells=self.frontier_window_radius_cells,
            min_frontier_free_neighbors=self.min_frontier_free_neighbors,
            max_frontier_occupied_ratio=self.max_frontier_occupied_ratio,
            min_reachable_unknown_cells=self.min_reachable_unknown_cells,
            min_unknown_span=self.min_frontier_unknown_span,
            candidate_clearance_radius=self.frontier_candidate_clearance_radius,
        )

        meaningful_clusters = [
            cluster
            for cluster in clusters
            if math.hypot(
                cluster.goal_xy[0] - robot_xy[0],
                cluster.goal_xy[1] - robot_xy[1],
            )
            >= self.min_goal_distance
        ]

        if not meaningful_clusters:
            self._frontier_reactivation_counter = 0
            return False

        self._frontier_reactivation_counter += 1
        return self._frontier_reactivation_counter >= self.frontier_reactivation_cycles

    def _request_frontier_resume(self) -> None:
        if self._resume_future is not None and not self._resume_future.done():
            return

        if not self._exploration_toggle_client.wait_for_service(timeout_sec=0.0):
            self.warn(
                "Frontier reactivation requested but /exploration/set_enabled is unavailable."
            )
            return

        request = SetBool.Request()
        request.data = True
        self._resume_future = self._exploration_toggle_client.call_async(request)
        self._resume_future.add_done_callback(self._handle_frontier_resume_response)
        self.info(
            "Frontiers appear to have reopened. Handing control back to frontier exploration."
        )

    def _handle_frontier_resume_response(self, future: Future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - defensive log path
            self.warn(f"Failed to resume frontier exploration: {exc}")
            return

        if not response.success:
            self.warn(f"Frontier exploration resume rejected: {response.message}")
            return

        self._exploration_complete = False
        self._frontier_reactivation_counter = 0
        self._disable_traversal(response.message, clear_goal=True)

    def _select_goal_pose(self, robot_pose: PoseStamped) -> PoseStamped | None:
        if self._current_map is None:
            return None

        robot_xy = (
            robot_pose.pose.position.x,
            robot_pose.pose.position.y,
        )
        map_meta = self._map_meta()
        robot_index = world_to_index(map_meta, robot_xy[0], robot_xy[1])
        if robot_index is None:
            return None

        component = extract_connected_free_component(
            self._current_map.data,
            map_meta,
            robot_index,
            self.occupied_threshold,
        )
        if not component.cells:
            return None

        sample_step_cells = max(1, int(round(self.sample_spacing / map_meta.resolution)))
        now_sec = self._now_seconds()
        self._goal_blacklist.prune(now_sec)

        tiled_candidates: dict[tuple[int, int], tuple[float, int, int, int]] = {}
        for cell in component.cells:
            row, col = index_to_cell(cell, map_meta.width)
            world_xy = index_to_world(map_meta, cell)
            distance_from_robot = math.hypot(
                world_xy[0] - robot_xy[0],
                world_xy[1] - robot_xy[1],
            )
            if distance_from_robot < self.min_goal_distance:
                continue

            goal_key = make_goal_key(
                world_xy[0],
                world_xy[1],
                self.revisit_position_resolution,
            )
            if self._goal_blacklist.contains(goal_key, now_sec):
                continue

            clearance_cells = compute_clearance_to_occupied(
                self._current_map.data,
                map_meta,
                cell,
                self.occupied_threshold,
                self.clearance_lookup_radius,
            )
            if clearance_cells < self.candidate_clearance_radius:
                continue

            _, occupied_count, unknown_count = count_cell_types_in_window(
                self._current_map.data,
                map_meta,
                cell,
                self.viewpoint_window_radius_cells,
                self.occupied_threshold,
            )
            if occupied_count == 0 and unknown_count == 0:
                continue

            heuristic = (
                (self.unknown_view_weight * unknown_count)
                + (self.occupied_view_weight * occupied_count)
                + (self.clearance_weight * clearance_cells)
                - distance_from_robot
            )
            tile_key = (row // sample_step_cells, col // sample_step_cells)
            candidate = (
                heuristic,
                cell,
                occupied_count,
                unknown_count,
            )
            current_best = tiled_candidates.get(tile_key)
            if current_best is None or candidate[0] > current_best[0]:
                tiled_candidates[tile_key] = candidate

        ranked_candidates = sorted(
            tiled_candidates.values(),
            key=lambda item: (-item[0], item[1]),
        )[: max(self.max_path_evaluations, 1)]

        best_goal: PoseStamped | None = None
        best_score: float | None = None
        best_path_length: float | None = None
        for _, cell, occupied_count, unknown_count in ranked_candidates:
            goal_xy = index_to_world(map_meta, cell)
            yaw = self._goal_yaw_for_cell(map_meta, cell, robot_xy)
            goal_pose = self._make_pose_stamped(goal_xy[0], goal_xy[1], yaw)
            path = self.getPath(robot_pose, goal_pose, use_start=True)
            if path is None or not path.poses:
                continue

            path_length = self._path_length(path)
            score = (
                (self.unknown_view_weight * unknown_count)
                + (self.occupied_view_weight * occupied_count)
                - (self.path_length_weight * path_length)
            )
            if (
                best_goal is None
                or score > best_score
                or (
                    score == best_score
                    and best_path_length is not None
                    and path_length < best_path_length
                )
            ):
                best_goal = goal_pose
                best_score = score
                best_path_length = path_length

        return best_goal

    def _goal_yaw_for_cell(
        self,
        map_meta: GridMeta,
        cell_index: int,
        robot_xy: tuple[float, float],
    ) -> float:
        if self._current_map is None:
            return 0.0

        center_row, center_col = index_to_cell(cell_index, map_meta.width)
        best_occupied_distance_sq: int | None = None
        best_unknown_distance_sq: int | None = None
        best_target_xy: tuple[float, float] | None = None

        for row in range(
            center_row - self.heading_search_radius_cells,
            center_row + self.heading_search_radius_cells + 1,
        ):
            if row < 0 or row >= map_meta.height:
                continue
            for col in range(
                center_col - self.heading_search_radius_cells,
                center_col + self.heading_search_radius_cells + 1,
            ):
                if col < 0 or col >= map_meta.width:
                    continue
                target_index = (row * map_meta.width) + col
                if target_index == cell_index:
                    continue

                value = self._current_map.data[target_index]
                distance_sq = ((row - center_row) ** 2) + ((col - center_col) ** 2)
                if value >= self.occupied_threshold:
                    if (
                        best_occupied_distance_sq is None
                        or distance_sq < best_occupied_distance_sq
                    ):
                        best_occupied_distance_sq = distance_sq
                        best_target_xy = index_to_world(map_meta, target_index)
                elif value == -1 and best_occupied_distance_sq is None:
                    if (
                        best_unknown_distance_sq is None
                        or distance_sq < best_unknown_distance_sq
                    ):
                        best_unknown_distance_sq = distance_sq
                        best_target_xy = index_to_world(map_meta, target_index)

        goal_xy = index_to_world(map_meta, cell_index)
        if best_target_xy is not None:
            return math.atan2(
                best_target_xy[1] - goal_xy[1],
                best_target_xy[0] - goal_xy[0],
            )

        return math.atan2(goal_xy[1] - robot_xy[1], goal_xy[0] - robot_xy[0])

    def _progress_tick(self) -> None:
        if not self._enabled or self._active_goal_pose is None:
            return

        now_sec = self._now_seconds()
        if self._active_goal_start_sec is not None:
            elapsed = now_sec - self._active_goal_start_sec
            if elapsed > self.goal_timeout_sec:
                self.warn(
                    "Active post-exploration goal timed out. Canceling and blacklisting it."
                )
                self.cancelTask()
                self.clearAllCostmaps()
                self._blacklist_pose(self._active_goal_pose, self.failed_goal_ttl_sec)
                self._clear_active_goal_state()
                return

        if not self.isTaskComplete():
            return

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.info("Reached post-exploration goal successfully.")
            self._blacklist_pose(self._active_goal_pose, self.revisit_ttl_sec)
            self._clear_active_goal_state()
            return

        if result == TaskResult.CANCELED and not self._enabled:
            self._clear_active_goal_state()
            return

        self.warn(
            f"Post-exploration navigation finished with result {result.name}. Retrying elsewhere."
        )
        self.clearAllCostmaps()
        self._blacklist_pose(self._active_goal_pose, self.failed_goal_ttl_sec)
        self._clear_active_goal_state()

    def _blacklist_pose(self, goal_pose: PoseStamped, ttl: float) -> None:
        key = make_goal_key(
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            self.revisit_position_resolution,
        )
        self._goal_blacklist.add(key, self._now_seconds(), ttl)

    def _clear_active_goal_state(self) -> None:
        self._active_goal_pose = None
        self._active_goal_start_sec = None

    def _disable_traversal(self, message: str, clear_goal: bool) -> None:
        self._enabled = False
        self._frontier_reactivation_counter = 0
        if self._active_goal_pose is not None:
            self.cancelTask()
        if clear_goal:
            self._clear_active_goal_state()
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

    def _make_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _map_meta(self) -> GridMeta:
        return GridMeta(
            width=self._current_map.info.width,
            height=self._current_map.info.height,
            resolution=self._current_map.info.resolution,
            origin_x=self._current_map.info.origin.position.x,
            origin_y=self._current_map.info.origin.position.y,
        )

    def _path_length(self, path: Path) -> float:
        total = 0.0
        for pose_a, pose_b in zip(path.poses, path.poses[1:]):
            dx = pose_b.pose.position.x - pose_a.pose.position.x
            dy = pose_b.pose.position.y - pose_a.pose.position.y
            total += math.hypot(dx, dy)
        return total

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PostExplorationTraverser()
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
