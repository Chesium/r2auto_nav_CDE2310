#!/usr/bin/env python3
"""Simple ArUco visual servo docking.

Detects an ArUco marker, drives toward it with PI control, and stops
at a configurable distance (default 30 cm).  No Nav2 docking server,
no TF lookups, no station-pose logic.  Triggered via a service call.

States: IDLE → APPROACHING → DONE / FAILED
"""

from __future__ import annotations

import math
from enum import Enum, auto

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger


# ── marker/station mapping ──────────────────────────────────────────────────
MARKER_STATION_MAP = {42: "A", 67: "B"}


# ── marker geometry ──────────────────────────────────────────────────────────

def _marker_object_points(size: float) -> np.ndarray:
    h = size / 2.0
    return np.array(
        [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
        dtype=np.float64,
    )


def _wrap_angle(rad: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (rad + math.pi) % (2 * math.pi) - math.pi


def _ccw_angle(from_rad: float, to_rad: float) -> float:
    """Counter-clockwise angle from from_rad to to_rad in [0, 2pi)."""
    return (to_rad - from_rad) % (2 * math.pi)


def _angle_0_2pi(rad: float) -> float:
    """Wrap angle to [0, 2pi)."""
    return rad % (2 * math.pi)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract planar yaw from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# ── OpenCV compat shims ─────────────────────────────────────────────────────

def _get_dictionary(name: str):
    const = getattr(aruco, name, None)
    if const is None:
        raise ValueError(f"Unknown dictionary: {name!r}")
    if hasattr(aruco, "Dictionary_get"):
        return aruco.Dictionary_get(const)
    return aruco.getPredefinedDictionary(const)


def _make_detector_params():
    if hasattr(aruco, "DetectorParameters_create"):
        params = aruco.DetectorParameters_create()
    else:
        params = aruco.DetectorParameters()
    # Subpixel corner refinement — improves solvePnP accuracy at close range
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 30
    return params


# ── state enum ───────────────────────────────────────────────────────────────

class _State(Enum):
    IDLE = auto()
    APPROACHING = auto()
    DONE = auto()
    FAILED = auto()


# ── node ─────────────────────────────────────────────────────────────────────

class SimpleArucoDock(Node):

    def __init__(self) -> None:
        super().__init__("simple_aruco_dock")

        # Parameters
        self.declare_parameter("image_topic", "/usb_cam/image_raw")
        self.declare_parameter("camera_info_topic", "/usb_cam/camera_info")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("target_marker_id", 42)
        self.declare_parameter("marker_size", 0.038)
        self.declare_parameter("dictionary", "DICT_4X4_100")
        self.declare_parameter("dock_distance", 0.30)
        self.declare_parameter("dist_tolerance", 0.04)
        self.declare_parameter("camera_forward_offset", 0.08)
        self.declare_parameter("bearing_tolerance_deg", 3.0)
        self.declare_parameter("dwell_frames", 5)
        self.declare_parameter("approach_timeout", 300.0)
        self.declare_parameter("kp_angular", 1.2)
        self.declare_parameter("kp_linear", 0.5)
        self.declare_parameter("ki_linear", 0.08)
        self.declare_parameter("max_linear", 0.02)
        self.declare_parameter("max_angular", 0.05)
        self.declare_parameter("max_bearing_for_drive_deg", 15.0)
        self.declare_parameter("ema_alpha", 0.3)
        self.declare_parameter("lost_hold", 3)
        self.declare_parameter("lost_stop", 10)
        self.declare_parameter("use_stamped_cmd_vel", True)
        self.declare_parameter("final_heading_offset_deg", -90.0)
        self.declare_parameter("post_turn_speed", 0.12)
        self.declare_parameter("post_turn_kp", 0.5)
        self.declare_parameter("post_turn_min_angle_deg", 1.0)
        self.declare_parameter("post_turn_yaw_tolerance_deg", 2.0)
        self.declare_parameter("post_shift_speed", 0.04)
        self.declare_parameter("post_shift_min_distance", 0.02)
        self.declare_parameter("post_shift_distance_tolerance", 0.01)

        # Read parameters
        image_topic = str(self.get_parameter("image_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        # Active target — set by dock_to_a / dock_to_b services
        self._active_target_id: int | None = None
        marker_size = float(self.get_parameter("marker_size").value)
        dict_name = str(self.get_parameter("dictionary").value)
        self._dock_dist = float(self.get_parameter("dock_distance").value)
        self._dist_tol = float(self.get_parameter("dist_tolerance").value)
        self._cam_forward_offset = float(self.get_parameter("camera_forward_offset").value)
        self._bearing_tol = math.radians(float(self.get_parameter("bearing_tolerance_deg").value))
        self._final_heading_offset = math.radians(float(self.get_parameter("final_heading_offset_deg").value))
        self._post_turn_speed = float(self.get_parameter("post_turn_speed").value)
        self._post_turn_kp = float(self.get_parameter("post_turn_kp").value)
        self._post_turn_min_angle = math.radians(float(self.get_parameter("post_turn_min_angle_deg").value))
        self._post_turn_yaw_tol = math.radians(float(self.get_parameter("post_turn_yaw_tolerance_deg").value))
        self._post_shift_speed = float(self.get_parameter("post_shift_speed").value)
        self._post_shift_min_dist = float(self.get_parameter("post_shift_min_distance").value)
        self._post_shift_dist_tol = float(self.get_parameter("post_shift_distance_tolerance").value)
        self._dwell_frames = int(self.get_parameter("dwell_frames").value)
        self._approach_timeout = float(self.get_parameter("approach_timeout").value)
        self._kp_ang = float(self.get_parameter("kp_angular").value)
        self._kp_lin = float(self.get_parameter("kp_linear").value)
        self._ki_lin = float(self.get_parameter("ki_linear").value)
        self._max_lin = float(self.get_parameter("max_linear").value)
        self._max_ang = float(self.get_parameter("max_angular").value)
        self._max_bearing = math.radians(float(self.get_parameter("max_bearing_for_drive_deg").value))
        self._ema_alpha = float(self.get_parameter("ema_alpha").value)
        self._lost_hold = int(self.get_parameter("lost_hold").value)
        self._lost_stop = int(self.get_parameter("lost_stop").value)
        self._use_stamped = bool(self.get_parameter("use_stamped_cmd_vel").value)

        # Detection resources
        self._bridge = CvBridge()
        self._obj_pts = _marker_object_points(marker_size)
        self._aruco_dict = _get_dictionary(dict_name)
        self._det_params = _make_detector_params()

        # Camera intrinsics
        self._cam_mtx: np.ndarray | None = None
        self._dist_coeffs: np.ndarray | None = None

        # State
        self._state = _State.IDLE
        self._bearing_ema: float | None = None
        self._distance_ema: float | None = None
        self._last_distance_cam: float | None = None
        self._last_lateral_offset: float | None = None
        self._normal_yaw: float | None = None
        self._integral = 0.0
        self._lost_count = 0
        self._dwell_count = 0
        self._approach_start: float | None = None
        self._odom_yaw: float | None = None
        self._odom_x: float | None = None
        self._odom_y: float | None = None
        self._post_turn_target_yaw: float | None = None
        self._post_turn_angle: float = 0.0
        self._post_turn_timer = None
        self._post_shift_target_dist: float | None = None
        self._post_shift_start_x: float | None = None
        self._post_shift_start_y: float | None = None
        self._post_shift_heading_yaw: float | None = None
        self._post_shift_timer = None

        # ROS I/O
        self.create_subscription(CameraInfo, info_topic, self._info_cb, 10)
        self.create_subscription(Image, image_topic, self._image_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        cmd_type = TwistStamped if self._use_stamped else Twist
        self._cmd_pub = self.create_publisher(cmd_type, "/cmd_vel", 10)
        self._done_pub = self.create_publisher(Bool, "/aruco_dock/done", 10)
        self._debug_pub = self.create_publisher(String, "/aruco_debug", 10)
        self._debug_img_pub = self.create_publisher(CompressedImage, "/aruco_debug/image_raw/compressed", 10)

        # Passive detection — always publishes when marker is visible, even in IDLE
        self._marker_visible_pub = self.create_publisher(Bool, "/aruco_dock/marker_visible", 10)
        self._marker_id_pub = self.create_publisher(Int32, "/aruco_dock/marker_id", 10)
        self._marker_distance_pub = self.create_publisher(Float32, "/aruco_dock/marker_distance", 10)

        # Station pose publishers — FSM uses these to detect stations
        self._station_a_pose_pub = self.create_publisher(PoseStamped, "/station_a_pose", 10)
        self._station_b_pose_pub = self.create_publisher(PoseStamped, "/station_b_pose", 10)

        # Services matching upstream aruco_dock_node interface
        self.create_service(Trigger, "/aruco_dock/dock_to_a", self._dock_to_a_cb)
        self.create_service(Trigger, "/aruco_dock/dock_to_b", self._dock_to_b_cb)
        self.create_service(Trigger, "/aruco_dock/scan", self._scan_cb)
        # Keep simple_dock services for backward compatibility
        self.create_service(Trigger, "/simple_dock/start", self._start_cb)
        self.create_service(Trigger, "/simple_dock/stop", self._stop_cb)

        self.get_logger().info(
            f"SimpleArucoDock ready. dock_dist={self._dock_dist}m. "
            f"Call /aruco_dock/dock_to_a or /aruco_dock/dock_to_b to begin."
        )

    # ── services ─────────────────────────────────────────────────────────

    def _begin_approach(self, marker_id: int, label: str, resp):
        """Shared logic for dock_to_a, dock_to_b, and legacy start."""
        if self._state == _State.APPROACHING:
            self.get_logger().warning(
                f"Already approaching — resetting and restarting for {label}")
            self._send_cmd(0.0, 0.0)
            self._cancel_post_turn()
            self._cancel_post_shift()
        self._active_target_id = marker_id
        self.get_logger().info(f"Docking started — {label} (marker {marker_id})")
        self._state = _State.APPROACHING
        self._bearing_ema = None
        self._distance_ema = None
        self._last_distance_cam = None
        self._last_lateral_offset = None
        self._normal_yaw = None
        self._post_turn_target_yaw = None
        self._post_shift_target_dist = None
        self._integral = 0.0
        self._lost_count = 0
        self._dwell_count = 0
        self._approach_start = self.get_clock().now().nanoseconds / 1e9
        self._cancel_post_turn()
        self._cancel_post_shift()
        resp.success = True
        resp.message = f"Approaching {label}"
        return resp

    def _dock_to_a_cb(self, _req, resp):
        return self._begin_approach(42, "Station A", resp)

    def _dock_to_b_cb(self, _req, resp):
        return self._begin_approach(67, "Station B", resp)

    def _scan_cb(self, _req, resp):
        """Stop docking approach and return to passive scanning."""
        self._send_cmd(0.0, 0.0)
        self._state = _State.IDLE
        self._active_target_id = None
        self._cancel_post_turn()
        self._cancel_post_shift()
        self.get_logger().info("Returned to scan mode")
        resp.success = True
        resp.message = "Scanning"
        return resp

    def _start_cb(self, _req, resp):
        """Legacy /simple_dock/start — uses whatever active target is set."""
        target = self._active_target_id
        if target is None:
            target = int(self.get_parameter("target_marker_id").value)
        label = MARKER_STATION_MAP.get(target, f"marker {target}")
        return self._begin_approach(target, label, resp)

    def _stop_cb(self, _req, resp):
        self._send_cmd(0.0, 0.0)
        self._state = _State.IDLE
        self.get_logger().info("Docking aborted by user.")
        self._cancel_post_turn()
        self._cancel_post_shift()
        resp.success = True
        resp.message = "Stopped"
        return resp

    # ── camera callbacks ─────────────────────────────────────────────────

    def _info_cb(self, msg: CameraInfo) -> None:
        if self._cam_mtx is not None:
            return
        self._cam_mtx = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        d = np.array(msg.d, dtype=np.float64)
        self._dist_coeffs = d if d.size > 0 else np.zeros(5, dtype=np.float64)
        self.get_logger().info("Camera intrinsics received.")

    def _odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._odom_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y

    def _image_cb(self, msg: Image) -> None:
        if self._cam_mtx is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Run detectMarkers once — used for both passive scanning and active approach
        gray_c = np.ascontiguousarray(gray, dtype=np.uint8)
        all_corners, all_ids, _ = aruco.detectMarkers(
            gray_c, self._aruco_dict, parameters=self._det_params
        )

        # Passive scanning: publish station poses for any detected marker
        self._scan_and_publish_stations(all_corners, all_ids)

        # Active approach: find the target marker for visual servo
        result = self._detect_target(all_corners, all_ids)

        # Publish passive detection info
        if result is not None:
            _, distance_raw, _, _, _, _ = result
            mid = self._active_target_id if self._active_target_id else -1
            self._marker_visible_pub.publish(Bool(data=True))
            self._marker_id_pub.publish(Int32(data=mid))
            self._marker_distance_pub.publish(Float32(data=float(distance_raw)))
        else:
            self._marker_visible_pub.publish(Bool(data=False))

        # Only run control loop when actively docking
        if self._state == _State.APPROACHING:
            self._process_frame_with_result(frame, result)

    # ── detection ────────────────────────────────────────────────────────

    def _scan_and_publish_stations(self, corners, ids) -> None:
        """Publish /station_a_pose or /station_b_pose every frame a known marker is visible."""
        if ids is None or self._cam_mtx is None:
            return

        cam = np.ascontiguousarray(self._cam_mtx, dtype=np.float64)
        dist_c = np.ascontiguousarray(self._dist_coeffs, dtype=np.float64)
        obj = np.ascontiguousarray(self._obj_pts, dtype=np.float64)

        for i, mid in enumerate(ids.flatten()):
            mid = int(mid)
            station_id = MARKER_STATION_MAP.get(mid)
            if station_id is None:
                continue

            img_pts = np.ascontiguousarray(corners[i][0], dtype=np.float64)
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    obj, img_pts, cam, dist_c, flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
            except cv2.error:
                continue
            if not ok or tvec[2, 0] <= 0:
                continue

            t = tvec.flatten()
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "camera_frame"
            pose.pose.position.x = float(t[0])
            pose.pose.position.y = float(t[1])
            pose.pose.position.z = float(t[2])

            if station_id == "A":
                self._station_a_pose_pub.publish(pose)
            else:
                self._station_b_pose_pub.publish(pose)

    def _detect_target(self, corners, ids):
        """Find the active target marker among pre-detected markers.
        Returns (bearing_rad, distance_m, corners, ids, rvec, tvec) or None."""
        if ids is None or self._active_target_id is None:
            return None

        cam = np.ascontiguousarray(self._cam_mtx, dtype=np.float64)
        dist_c = np.ascontiguousarray(self._dist_coeffs, dtype=np.float64)
        obj = np.ascontiguousarray(self._obj_pts, dtype=np.float64)

        for i, mid in enumerate(ids.flatten()):
            if int(mid) != self._active_target_id:
                continue
            img_pts = np.ascontiguousarray(corners[i][0], dtype=np.float64)
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    obj, img_pts, cam, dist_c, flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
            except cv2.error:
                return None
            if not ok or tvec[2, 0] <= 0:
                return None
            t = tvec.flatten()
            bearing = float(math.atan2(t[0], t[2]))
            distance_cam = float(t[2])
            distance = max(distance_cam - self._cam_forward_offset, 0.0)
            self._last_distance_cam = distance_cam
            self._last_lateral_offset = float(t[0])
            return bearing, distance, corners, ids, rvec, tvec
        return None

    @staticmethod
    def _marker_normal_yaw(rvec: np.ndarray) -> float:
        """Yaw of marker plane normal in camera frame."""
        rvec_c = np.ascontiguousarray(rvec, dtype=np.float64)
        R, _ = cv2.Rodrigues(rvec_c)
        normal = R[:, 2]
        return float(math.atan2(normal[0], normal[2]))

    # ── main loop ────────────────────────────────────────────────────────

    def _process_frame_with_result(self, frame: np.ndarray, result) -> None:
        debug_frame = frame.copy()

        # Timeout check
        now = self.get_clock().now().nanoseconds / 1e9
        if self._approach_start and (now - self._approach_start) > self._approach_timeout:
            self._finish(success=False, reason="Approach timeout")
            return

        if result is not None:
            bearing_raw, distance_raw, corners, ids, rvec, tvec = result

            # Draw debug annotations
            aruco.drawDetectedMarkers(debug_frame, corners, ids)
            cv2.drawFrameAxes(debug_frame, self._cam_mtx, self._dist_coeffs,
                              rvec, tvec, 0.05)
            self._lost_count = 0

            # EMA update
            a = self._ema_alpha
            if self._bearing_ema is None:
                self._bearing_ema = bearing_raw
                self._distance_ema = distance_raw
            else:
                self._bearing_ema = a * bearing_raw + (1 - a) * self._bearing_ema
                self._distance_ema = a * distance_raw + (1 - a) * self._distance_ema

            normal_yaw = self._marker_normal_yaw(rvec)
            self._normal_yaw = normal_yaw

            dist_err = self._distance_ema - self._dock_dist
            holding_position = self._distance_ema <= self._dock_dist + self._dist_tol
            bearing_err = self._bearing_ema
            bearing_tol = self._bearing_tol

            # Check done FIRST — before any control output
            # Use raw distance too (not just EMA) to catch overshoot
            at_dist = abs(dist_err) < self._dist_tol or distance_raw <= self._dock_dist
            at_bearing = abs(bearing_err) < bearing_tol
            if at_dist and at_bearing:
                self._dwell_count += 1
                if self._dwell_count >= self._dwell_frames:
                    self._finish(success=True,
                                 reason=f"Docked at {self._distance_ema:.3f}m "
                                        f"(raw={distance_raw:.3f}m)")
                    return
            else:
                self._dwell_count = 0

            angular_z = float(np.clip(
                -self._kp_ang * bearing_err, -self._max_ang, self._max_ang
            ))

            if holding_position:
                linear_x = 0.0
                self._integral = 0.0
            else:
                # Integral (distance only) with basic anti-windup
                integrate = abs(self._bearing_ema) < self._max_bearing
                new_integral = self._integral
                if integrate:
                    new_integral += dist_err * (1.0 / 30.0)
                    max_int = 0.10 / max(self._ki_lin, 1e-6)
                    new_integral = float(np.clip(new_integral, -max_int, max_int))

                pi_cmd = self._kp_lin * dist_err + self._ki_lin * (new_integral if integrate else self._integral)
                linear_x = float(np.clip(pi_cmd, 0.0, self._max_lin))

                if integrate and 0.0 < linear_x < self._max_lin:
                    self._integral = new_integral

            self._send_cmd(linear_x, angular_z)

            cam_dist = self._last_distance_cam if self._last_distance_cam is not None else float("nan")

            # Debug
            dbg = (
                f"d={self._distance_ema:.3f} raw={distance_raw:.3f} "
                f"cam={cam_dist:.3f} "
                f"x={self._last_lateral_offset or 0.0:+.3f} "
                f"b={math.degrees(self._bearing_ema):+.1f} "
                f"n={math.degrees(self._normal_yaw or 0.0):+.1f} "
                f"err={dist_err:+.3f} cmd=({linear_x:.3f},{angular_z:.3f}) "
                f"hold={holding_position} dwell={self._dwell_count}/{self._dwell_frames}"
            )
            self._debug_pub.publish(String(data=dbg))

            # Text overlay on debug image
            cv2.putText(debug_frame, f"d={self._distance_ema:.2f}m b={math.degrees(self._bearing_ema):+.1f}deg",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0), 2)
            cv2.putText(debug_frame, f"cam={cam_dist:.2f}m x={self._last_lateral_offset or 0.0:+.2f}m",
                        (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
            cv2.putText(debug_frame, f"n={math.degrees(self._normal_yaw or 0.0):+.1f}deg err={dist_err:+.3f} dwell={self._dwell_count}/{self._dwell_frames}",
                        (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug_frame, self._state.name,
                        (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            # Marker lost
            self._lost_count += 1
            if self._lost_count >= self._lost_stop:
                self._finish(success=False, reason="Marker lost")
                return
            elif self._lost_count >= self._lost_hold:
                self._send_cmd(0.0, 0.0)
                self._integral = 0.0  # zero integral on loss
            # else: coast with last cmd_vel

            self._debug_pub.publish(String(data=f"LOST count={self._lost_count}"))
            cv2.putText(debug_frame, f"LOST ({self._lost_count})",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Publish debug image
        self._publish_debug_image(debug_frame)

    # ── helpers ──────────────────────────────────────────────────────────

    def _finish(self, *, success: bool, reason: str) -> None:
        self._send_cmd(0.0, 0.0)
        self._cancel_post_turn()
        self._cancel_post_shift()
        self._state = _State.DONE if success else _State.FAILED
        msg = f"Docking {'DONE' if success else 'FAILED'}: {reason}"
        if success:
            self.get_logger().info(msg)
            self._start_post_turn()
        else:
            self.get_logger().warning(msg)
            self._done_pub.publish(Bool(data=False))

    def _publish_debug_image(self, frame: np.ndarray) -> None:
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        self._debug_img_pub.publish(msg)

    def _send_cmd(self, linear_x: float, angular_z: float) -> None:
        if self._use_stamped:
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "base_link"
            cmd.twist.linear.x = linear_x
            cmd.twist.angular.z = angular_z
        else:
            cmd = Twist()
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z
        self._cmd_pub.publish(cmd)

    # ── post-dock turn --------------------------------------------------

    def _start_post_turn(self) -> None:
        if self._post_turn_speed <= 0.0:
            self._start_post_shift()
            return
        if self._odom_yaw is None or self._normal_yaw is None:
            self.get_logger().warning("Skipping post-dock turn: missing odom yaw or marker normal")
            self._start_post_shift()
            return
        # Compute turn in camera frame: how much CCW rotation brings the
        # marker normal (n) to the desired target angle, then apply the same
        # rotation to odom_yaw.  This works because the camera is fixed on
        # the robot.
        #
        # final_heading_offset_deg sets the target angle for n in camera frame:
        #   -90  → wall ends up on your left  (270° CCW = 90° CW)
        #    90  → wall ends up on your right (90° CCW)
        n = self._normal_yaw
        target_n = self._final_heading_offset
        angle = _ccw_angle(_angle_0_2pi(n), _angle_0_2pi(target_n))
        if angle < self._post_turn_min_angle:
            self._start_post_shift()
            return
        self._post_turn_angle = angle
        self._post_turn_target_yaw = _wrap_angle(self._odom_yaw + angle)
        if self._post_turn_timer is None:
            self._post_turn_timer = self.create_timer(0.02, self._post_turn_step)
        self.get_logger().info(
            f"Post-dock turn: n={math.degrees(n):+.1f}deg "
            f"target_n={math.degrees(target_n):+.1f}deg "
            f"ccw={math.degrees(angle):.1f}deg "
            f"target_yaw={math.degrees(self._post_turn_target_yaw):+.1f}deg"
        )

    def _post_turn_step(self) -> None:
        if self._post_turn_target_yaw is None or self._odom_yaw is None:
            self._cancel_post_turn()
            return
        yaw_err = _wrap_angle(self._post_turn_target_yaw - self._odom_yaw)
        ccw_err = _ccw_angle(self._odom_yaw, self._post_turn_target_yaw)
        self._debug_pub.publish(
            String(
                data=(
                    f"POST_TURN yaw={math.degrees(self._odom_yaw):+.1f} "
                    f"target={math.degrees(self._post_turn_target_yaw):+.1f} "
                    f"err={math.degrees(yaw_err):+.1f} "
                    f"ccw={math.degrees(ccw_err):.1f}"
                )
            )
        )
        if abs(yaw_err) <= self._post_turn_yaw_tol or ccw_err <= self._post_turn_yaw_tol:
            self._send_cmd(0.0, 0.0)
            self._cancel_post_turn()
            self.get_logger().info("Post-dock turn complete")
            self._start_post_shift()
        else:
            angular_z = float(np.clip(self._post_turn_kp * ccw_err, 0.0, self._post_turn_speed))
            self._send_cmd(0.0, angular_z)

    def _cancel_post_turn(self) -> None:
        if self._post_turn_timer is not None:
            self._post_turn_timer.cancel()
            self._post_turn_timer = None
        self._post_turn_target_yaw = None

    # ── post-dock shift -------------------------------------------------

    def _publish_done(self) -> None:
        self.get_logger().info("Post-dock sequence complete — publishing done")
        self._done_pub.publish(Bool(data=True))

    def _start_post_shift(self) -> None:
        if self._post_shift_speed <= 0.0:
            self._publish_done()
            return
        if self._last_lateral_offset is None:
            self.get_logger().warning("Skipping post-dock shift: no lateral offset available")
            self._publish_done()
            return
        if self._odom_x is None or self._odom_y is None:
            self.get_logger().warning("Skipping post-dock shift: no odom position received yet")
            self._publish_done()
            return
        # The camera is cam_forward_offset (r) ahead of the pivot point.
        # After turning by angle `a` (CCW), the camera sweeps an arc and
        # its projection along the new forward direction shifts.  The exact
        # shift needed to re-align the camera with the marker is:
        #
        #   shift = -x * sin(a) - r * (1 - cos(a))
        #
        # where x = last_lateral_offset, a = CCW turn angle, r = cam offset.
        # For 90° CW (a=270°): shift =  x - r
        # For 90° CCW (a=90°): shift = -x - r
        a = self._post_turn_angle
        x = self._last_lateral_offset
        r = self._cam_forward_offset
        self._post_shift_target_dist = -x * math.sin(a) - r * (1.0 - math.cos(a))
        if abs(self._post_shift_target_dist) < self._post_shift_min_dist:
            self._post_shift_target_dist = None
            self._publish_done()
            return
        self._post_shift_start_x = self._odom_x
        self._post_shift_start_y = self._odom_y
        self._post_shift_heading_yaw = self._odom_yaw
        if self._post_shift_timer is None:
            self._post_shift_timer = self.create_timer(0.02, self._post_shift_step)
        self.get_logger().info(
            f"Post-dock shift: x={x:+.3f}m r={r:.3f}m "
            f"a={math.degrees(a):.0f}deg "
            f"move={self._post_shift_target_dist:+.3f}m"
        )

    def _post_shift_step(self) -> None:
        if (
            self._post_shift_target_dist is None
            or self._post_shift_start_x is None
            or self._post_shift_start_y is None
            or self._post_shift_heading_yaw is None
            or self._odom_x is None
            or self._odom_y is None
        ):
            self._cancel_post_shift()
            return
        dx = self._odom_x - self._post_shift_start_x
        dy = self._odom_y - self._post_shift_start_y
        progress = (
            dx * math.cos(self._post_shift_heading_yaw)
            + dy * math.sin(self._post_shift_heading_yaw)
        )
        remaining = self._post_shift_target_dist - progress
        self._debug_pub.publish(
            String(
                data=(
                    f"POST_SHIFT target={self._post_shift_target_dist:+.3f} "
                    f"progress={progress:+.3f} rem={remaining:+.3f}"
                )
            )
        )
        if abs(remaining) <= self._post_shift_dist_tol:
            self._send_cmd(0.0, 0.0)
            self._cancel_post_shift()
            self.get_logger().info("Post-dock shift complete")
            self._done_pub.publish(Bool(data=True))
        else:
            linear_x = math.copysign(self._post_shift_speed, remaining)
            self._send_cmd(linear_x, 0.0)

    def _cancel_post_shift(self) -> None:
        if self._post_shift_timer is not None:
            self._post_shift_timer.cancel()
            self._post_shift_timer = None
        self._post_shift_target_dist = None
        self._post_shift_start_x = None
        self._post_shift_start_y = None
        self._post_shift_heading_yaw = None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimpleArucoDock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
