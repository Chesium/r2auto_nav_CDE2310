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
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from g3g_frontier_exploration.frontier_utils import count_unknown_cells_near_point
from g3g_frontier_exploration.frontier_utils import evaluate_component_encapsulation
from g3g_frontier_exploration.frontier_utils import ExpiringBlacklist
from g3g_frontier_exploration.frontier_utils import evaluate_frontier_cell
from g3g_frontier_exploration.frontier_utils import GridMeta
from g3g_frontier_exploration.frontier_utils import euclidean_distance
from g3g_frontier_exploration.frontier_utils import extract_frontier_clusters
from g3g_frontier_exploration.frontier_utils import index_to_world
from g3g_frontier_exploration.frontier_utils import make_cluster_key
from g3g_frontier_exploration.frontier_utils import select_cluster_goal_cell


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
        self.declare_parameter("frontier_region_resolution", 0.5)
        self.declare_parameter("information_gain_radius", 0.6)
        self.declare_parameter("min_information_gain_cells", 2)
        self.declare_parameter("frontier_window_radius_cells", 2)
        self.declare_parameter("min_frontier_free_neighbors", 3)
        self.declare_parameter("max_frontier_occupied_ratio", 0.45)
        self.declare_parameter("min_reachable_unknown_cells", 3)
        self.declare_parameter("min_frontier_unknown_span", 2)
        self.declare_parameter("candidate_clearance_radius", 1)
        self.declare_parameter("cluster_fail_limit", 3)
        self.declare_parameter("cluster_blacklist_ttl_sec", 180.0)
        self.declare_parameter("invalid_goal_blacklist_ttl_sec", 90.0)
        self.declare_parameter("encapsulation_check_enabled", True)
        self.declare_parameter("encapsulation_occupied_threshold", 45)
        self.declare_parameter("encapsulation_confirmation_cycles", 3)
        self.declare_parameter("encapsulation_max_unknown_holes", 2)
        self.declare_parameter("encapsulation_max_unknown_hole_cells", 6)
        self.declare_parameter("planning_rate_hz", 1.0)
        self.declare_parameter("progress_rate_hz", 2.0)
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("blacklist_ttl_sec", 60.0)
        self.declare_parameter("completion_patience_cycles", 5)
        self.declare_parameter("autostart", False)

        self.map_topic = (
            self.get_parameter("map_topic").get_parameter_value().string_value
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
        self.frontier_min_cluster_size = (
            self.get_parameter("frontier_min_cluster_size")
            .get_parameter_value()
            .integer_value
        )
        self.min_goal_distance = (
            self.get_parameter("min_goal_distance").get_parameter_value().double_value
        )
        self.frontier_region_resolution = (
            self.get_parameter("frontier_region_resolution")
            .get_parameter_value()
            .double_value
        )
        self.information_gain_radius = (
            self.get_parameter("information_gain_radius")
            .get_parameter_value()
            .double_value
        )
        self.min_information_gain_cells = (
            self.get_parameter("min_information_gain_cells")
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
        self.candidate_clearance_radius = (
            self.get_parameter("candidate_clearance_radius")
            .get_parameter_value()
            .integer_value
        )
        self.cluster_fail_limit = (
            self.get_parameter("cluster_fail_limit")
            .get_parameter_value()
            .integer_value
        )
        self.cluster_blacklist_ttl_sec = (
            self.get_parameter("cluster_blacklist_ttl_sec")
            .get_parameter_value()
            .double_value
        )
        self.invalid_goal_blacklist_ttl_sec = (
            self.get_parameter("invalid_goal_blacklist_ttl_sec")
            .get_parameter_value()
            .double_value
        )
        self.encapsulation_check_enabled = (
            self.get_parameter("encapsulation_check_enabled")
            .get_parameter_value()
            .bool_value
        )
        self.encapsulation_occupied_threshold = (
            self.get_parameter("encapsulation_occupied_threshold")
            .get_parameter_value()
            .integer_value
        )
        self.encapsulation_confirmation_cycles = (
            self.get_parameter("encapsulation_confirmation_cycles")
            .get_parameter_value()
            .integer_value
        )
        self.encapsulation_max_unknown_holes = (
            self.get_parameter("encapsulation_max_unknown_holes")
            .get_parameter_value()
            .integer_value
        )
        self.encapsulation_max_unknown_hole_cells = (
            self.get_parameter("encapsulation_max_unknown_hole_cells")
            .get_parameter_value()
            .integer_value
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
        self.autostart = (
            self.get_parameter("autostart").get_parameter_value().bool_value
        )

        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._current_map: OccupancyGrid | None = None
        self._cluster_blacklist = ExpiringBlacklist()
        self._cluster_failure_counts: dict[tuple[int, ...], int] = {}
        self._exhausted_clusters: set[tuple[int, ...]] = set()
        self._enabled = False
        self._system_ready = False
        self._no_frontier_cycles = 0
        self._encapsulation_cycles = 0
        self._autostart_consumed = False
        self._active_goal_pose: PoseStamped | None = None
        self._active_cluster_key: tuple[int, ...] | None = None
        self._active_frontier_xy: tuple[float, float] | None = None
        self._active_unknown_count_before: int | None = None
        self._active_cluster_cells: tuple[int, ...] | None = None
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
        self._completion_publisher = self.create_publisher(
            Bool,
            "/exploration_complete",
            map_qos,
        )
        self._toggle_service = self.create_service(
            SetBool,
            "/exploration/set_enabled",
            self._handle_set_enabled,
        )

        self._publish_markers([], None)
        self._set_exploration_complete(False)
        self.info("Frontier explorer initialized and waiting in idle mode.")

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._current_map = msg

    def _handle_set_enabled(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
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
            self._encapsulation_cycles = 0
            self._cluster_failure_counts.clear()
            self._exhausted_clusters.clear()
            self._set_exploration_complete(False)
            response.success = True
            response.message = "Exploration enabled."
            self.info("Exploration enabled by service call.")
            return response

        self._enabled = False
        self._no_frontier_cycles = 0
        self._cancel_active_goal_requested = True
        self._set_exploration_complete(False)
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
            self.info(
                "Explorer is ready: map, transform, and Nav2 servers are available."
            )
        elif not ready:
            self._system_ready = False
            self.debug(f"Explorer is still waiting for readiness: {reason}")

        if (
            self.autostart
            and self._system_ready
            and not self._autostart_consumed
            and not self._enabled
        ):
            self._autostart_consumed = True
            self._enabled = True
            self._no_frontier_cycles = 0
            self._set_exploration_complete(False)
            self.info("Autostart is enabled. Beginning exploration.")

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

        now_sec = self._now_seconds()
        self._cluster_blacklist.prune(now_sec)

        map_meta = GridMeta(
            width=self._current_map.info.width,
            height=self._current_map.info.height,
            resolution=self._current_map.info.resolution,
            origin_x=self._current_map.info.origin.position.x,
            origin_y=self._current_map.info.origin.position.y,
        )
        robot_xy = (
            robot_pose.pose.position.x,
            robot_pose.pose.position.y,
        )

        if self.encapsulation_check_enabled:
            encapsulation = evaluate_component_encapsulation(
                self._current_map.data,
                map_meta,
                robot_xy,
                self.encapsulation_occupied_threshold,
                self.encapsulation_max_unknown_holes,
                self.encapsulation_max_unknown_hole_cells,
            )
            if encapsulation.is_enclosed:
                self._encapsulation_cycles += 1
                if self._encapsulation_cycles == 1:
                    self.info(
                        "Encapsulation bypass detected enclosed reachable space. "
                        "Confirming before completion."
                    )
                if self._encapsulation_cycles >= self.encapsulation_confirmation_cycles:
                    self.info(
                        "Encapsulation confirmed with "
                        f"{encapsulation.unknown_hole_count} small unknown hole clusters "
                        f"(largest size {encapsulation.largest_unknown_hole_size})."
                    )
                    self._complete_exploration(
                        "Reachable space is enclosed; treating exploration as complete.",
                        clear_markers=True,
                    )
                    return
            else:
                if self._encapsulation_cycles > 0:
                    self.info(
                        "Encapsulation bypass reset because reachable space is no longer enclosed."
                    )
                self._encapsulation_cycles = 0

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
            candidate_clearance_radius=self.candidate_clearance_radius,
        )

        ranked_candidates = []

        for cluster in clusters:
            selected_goal_cell = select_cluster_goal_cell(
                cluster.cells,
                map_meta,
                robot_xy,
                self.min_goal_distance,
            )
            cluster_key = make_cluster_key(
                cluster.cells,
                map_meta,
                selected_goal_cell,
                self.frontier_region_resolution,
            )
            if cluster_key in self._exhausted_clusters:
                continue
            if self._cluster_blacklist.contains(cluster_key, now_sec):
                continue

            candidate_cell = self._select_valid_goal_cell(
                cluster.cells,
                selected_goal_cell,
                map_meta,
            )
            if candidate_cell is None:
                self._blacklist_cluster(
                    cluster_key,
                    now_sec,
                    self.invalid_goal_blacklist_ttl_sec,
                    payload=index_to_world(map_meta, selected_goal_cell),
                )
                continue

            goal_xy = index_to_world(map_meta, candidate_cell)
            if euclidean_distance(robot_xy, goal_xy) < self.min_goal_distance:
                continue

            goal_pose = self._make_pose_stamped(goal_xy[0], goal_xy[1])
            path = self.getPath(robot_pose, goal_pose, use_start=True)

            if path is None or not path.poses:
                self._blacklist_cluster(
                    cluster_key,
                    now_sec,
                    self.invalid_goal_blacklist_ttl_sec,
                    payload=goal_xy,
                )
                continue

            ranked_candidates.append(
                (
                    self._path_length(path),
                    -cluster.size,
                    goal_pose,
                    cluster_key,
                    goal_xy,
                    cluster.cells,
                    count_unknown_cells_near_point(
                        self._current_map.data,
                        map_meta,
                        goal_xy,
                        self.information_gain_radius,
                    ),
                )
            )

        self._publish_markers(clusters, self._active_goal_pose)

        if not ranked_candidates:
            self._no_frontier_cycles += 1
            if self._no_frontier_cycles >= self.completion_patience_cycles:
                self.info("No valid frontiers remain. Exploration is complete.")
                self._complete_exploration(
                    "Exploration completed successfully.",
                    clear_markers=True,
                )
            return

        self._no_frontier_cycles = 0
        ranked_candidates.sort(key=lambda item: (item[0], item[1]))
        _, _, goal_pose, cluster_key, frontier_xy, cluster_cells, unknown_count_before = (
            ranked_candidates[0]
        )

        if not self.goToPose(goal_pose):
            self.warn(
                "Nav2 rejected the selected exploration goal. Blacklisting it temporarily."
            )
            self._blacklist_cluster(
                cluster_key,
                now_sec,
                self.invalid_goal_blacklist_ttl_sec,
                payload=(goal_pose.pose.position.x, goal_pose.pose.position.y),
            )
            self.clearAllCostmaps()
            self._publish_markers(clusters, None)
            return

        self._active_goal_pose = goal_pose
        self._active_cluster_key = cluster_key
        self._active_frontier_xy = frontier_xy
        self._active_unknown_count_before = unknown_count_before
        self._active_cluster_cells = cluster_cells
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
                self.warn(
                    "Active frontier goal timed out. Canceling and blacklisting it."
                )
                self.cancelTask()
                self.clearAllCostmaps()
                self._blacklist_active_cluster(now_sec, self.cluster_blacklist_ttl_sec)
                self._clear_active_goal_state()
                return

        if not self.isTaskComplete():
            return

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            if self._goal_produced_information_gain():
                self.info("Reached active frontier goal successfully.")
            else:
                self.warn(
                    "Goal reached but nearby unknown space did not shrink enough. "
                    "Blacklisting this frontier cluster."
                )
                self._blacklist_active_cluster(now_sec, self.cluster_blacklist_ttl_sec)
            self._clear_active_goal_state()
            return

        if result == TaskResult.CANCELED and not self._enabled:
            self._clear_active_goal_state()
            return

        self.warn(
            f"Frontier navigation finished with result {result.name}. Retrying elsewhere."
        )
        self.clearAllCostmaps()
        self._blacklist_active_cluster(now_sec, self.cluster_blacklist_ttl_sec)
        self._clear_active_goal_state()

    def _clear_active_goal_state(self) -> None:
        self._active_goal_pose = None
        self._active_cluster_key = None
        self._active_frontier_xy = None
        self._active_unknown_count_before = None
        self._active_cluster_cells = None
        self._active_goal_start_sec = None
        self._publish_markers([], None)

    def _blacklist_active_cluster(self, now_sec: float, ttl: float) -> None:
        if self._active_cluster_key is None or self._active_goal_pose is None:
            return
        goal_xy = (
            self._active_goal_pose.pose.position.x,
            self._active_goal_pose.pose.position.y,
        )
        self._blacklist_cluster(
            self._active_cluster_key,
            now_sec,
            ttl,
            payload=goal_xy,
        )

    def _blacklist_cluster(
        self,
        cluster_key: tuple[int, ...],
        now_sec: float,
        ttl: float,
        payload: tuple[float, float] | None,
    ) -> None:
        self._cluster_blacklist.add(
            cluster_key,
            now_sec,
            ttl,
            payload=payload,
        )
        failures = self._cluster_failure_counts.get(cluster_key, 0) + 1
        self._cluster_failure_counts[cluster_key] = failures
        if failures >= self.cluster_fail_limit:
            self._exhausted_clusters.add(cluster_key)

    def _select_valid_goal_cell(
        self,
        cluster_cells: tuple[int, ...],
        preferred_cell: int,
        map_meta: GridMeta,
    ) -> int | None:
        preferred_metrics = evaluate_frontier_cell(
            self._current_map.data,
            map_meta,
            preferred_cell,
            self.occupied_threshold,
            self.frontier_window_radius_cells,
            self.min_frontier_free_neighbors,
            self.max_frontier_occupied_ratio,
            self.min_reachable_unknown_cells,
            self.min_frontier_unknown_span,
            self.candidate_clearance_radius,
        )
        if preferred_metrics.is_valid:
            return preferred_cell

        preferred_xy = index_to_world(map_meta, preferred_cell)
        ranked_alternatives = sorted(
            cluster_cells,
            key=lambda cell: (
                euclidean_distance(index_to_world(map_meta, cell), preferred_xy),
                cell,
            ),
        )
        for cell in ranked_alternatives:
            metrics = evaluate_frontier_cell(
                self._current_map.data,
                map_meta,
                cell,
                self.occupied_threshold,
                self.frontier_window_radius_cells,
                self.min_frontier_free_neighbors,
                self.max_frontier_occupied_ratio,
                self.min_reachable_unknown_cells,
                self.min_frontier_unknown_span,
                self.candidate_clearance_radius,
            )
            if metrics.is_valid:
                return cell

        return None

    def _goal_produced_information_gain(self) -> bool:
        if (
            self._current_map is None
            or self._active_frontier_xy is None
            or self._active_unknown_count_before is None
        ):
            return True

        map_meta = GridMeta(
            width=self._current_map.info.width,
            height=self._current_map.info.height,
            resolution=self._current_map.info.resolution,
            origin_x=self._current_map.info.origin.position.x,
            origin_y=self._current_map.info.origin.position.y,
        )
        unknown_count_after = count_unknown_cells_near_point(
            self._current_map.data,
            map_meta,
            self._active_frontier_xy,
            self.information_gain_radius,
        )
        return (
            self._active_unknown_count_before - unknown_count_after
            >= self.min_information_gain_cells
        )

    def _disable_exploration(self, message: str, clear_markers: bool) -> None:
        self._enabled = False
        self._no_frontier_cycles = 0
        self._encapsulation_cycles = 0
        if self._active_goal_pose is not None:
            self.cancelTask()
        self._clear_active_goal_state()
        if clear_markers:
            self._publish_markers([], None)
        self.info(message)

    def _complete_exploration(self, message: str, clear_markers: bool) -> None:
        self._set_exploration_complete(True)
        self._disable_exploration(message, clear_markers)

    def _set_exploration_complete(self, completed: bool) -> None:
        msg = Bool()
        msg.data = completed
        self._completion_publisher.publish(msg)

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
        for point_xy in self._cluster_blacklist.active_entries(now_sec).values():
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
