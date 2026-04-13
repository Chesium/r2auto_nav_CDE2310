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
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger


# ── marker geometry ──────────────────────────────────────────────────────────

def _marker_object_points(size: float) -> np.ndarray:
    h = size / 2.0
    return np.array(
        [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
        dtype=np.float64,
    )


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
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("target_marker_id", 42)
        self.declare_parameter("marker_size", 0.067)
        self.declare_parameter("dictionary", "DICT_4X4_100")
        self.declare_parameter("dock_distance", 0.30)
        self.declare_parameter("dist_tolerance", 0.04)
        self.declare_parameter("bearing_tolerance_deg", 3.0)
        self.declare_parameter("dwell_frames", 5)
        self.declare_parameter("approach_timeout", 30.0)
        self.declare_parameter("kp_angular", 1.2)
        self.declare_parameter("kp_linear", 0.5)
        self.declare_parameter("ki_linear", 0.08)
        self.declare_parameter("max_linear", 0.15)
        self.declare_parameter("max_angular", 0.8)
        self.declare_parameter("max_bearing_for_drive_deg", 15.0)
        self.declare_parameter("ema_alpha", 0.3)
        self.declare_parameter("lost_hold", 3)
        self.declare_parameter("lost_stop", 10)
        self.declare_parameter("use_stamped_cmd_vel", False)

        # Read parameters
        image_topic = str(self.get_parameter("image_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        self._target_id = int(self.get_parameter("target_marker_id").value)
        marker_size = float(self.get_parameter("marker_size").value)
        dict_name = str(self.get_parameter("dictionary").value)
        self._dock_dist = float(self.get_parameter("dock_distance").value)
        self._dist_tol = float(self.get_parameter("dist_tolerance").value)
        self._bearing_tol = math.radians(float(self.get_parameter("bearing_tolerance_deg").value))
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
        self._integral = 0.0
        self._lost_count = 0
        self._dwell_count = 0
        self._approach_start: float | None = None

        # ROS I/O
        self.create_subscription(CameraInfo, info_topic, self._info_cb, 10)
        self.create_subscription(Image, image_topic, self._image_cb, 10)
        cmd_type = TwistStamped if self._use_stamped else Twist
        self._cmd_pub = self.create_publisher(cmd_type, "/cmd_vel", 10)
        self._done_pub = self.create_publisher(Bool, "/aruco_dock/done", 10)
        self._debug_pub = self.create_publisher(String, "/aruco_debug", 10)
        self._debug_img_pub = self.create_publisher(CompressedImage, "/aruco_debug/image_raw/compressed", 10)

        # Passive detection — always publishes when marker is visible, even in IDLE
        # FSM subscribes to these to know when to call /simple_dock/start
        self._marker_visible_pub = self.create_publisher(Bool, "/aruco_dock/marker_visible", 10)
        self._marker_id_pub = self.create_publisher(Int32, "/aruco_dock/marker_id", 10)
        self._marker_distance_pub = self.create_publisher(Float32, "/aruco_dock/marker_distance", 10)

        self.create_service(Trigger, "/simple_dock/start", self._start_cb)
        self.create_service(Trigger, "/simple_dock/stop", self._stop_cb)

        self.get_logger().info(
            f"SimpleArucoDock ready. marker={self._target_id} "
            f"dock_dist={self._dock_dist}m. Call /simple_dock/start to begin."
        )

    # ── services ─────────────────────────────────────────────────────────

    def _start_cb(self, _req, resp):
        if self._state == _State.APPROACHING:
            resp.success = False
            resp.message = "Already docking"
            return resp
        self.get_logger().info("Docking started!")
        self._state = _State.APPROACHING
        self._bearing_ema = None
        self._distance_ema = None
        self._integral = 0.0
        self._lost_count = 0
        self._dwell_count = 0
        self._approach_start = self.get_clock().now().nanoseconds / 1e9
        resp.success = True
        resp.message = "Approaching marker"
        return resp

    def _stop_cb(self, _req, resp):
        self._send_cmd(0.0, 0.0)
        self._state = _State.IDLE
        self.get_logger().info("Docking aborted by user.")
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

    def _image_cb(self, msg: Image) -> None:
        if self._cam_mtx is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge: {e}")
            return

        # Always run detection for passive scanning
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        result = self._detect(gray)

        # Publish passive detection info (FSM reads these)
        if result is not None:
            _, distance_raw, _, _, _, _ = result
            self._marker_visible_pub.publish(Bool(data=True))
            self._marker_id_pub.publish(Int32(data=self._target_id))
            self._marker_distance_pub.publish(Float32(data=float(distance_raw)))
        else:
            self._marker_visible_pub.publish(Bool(data=False))

        # Only run control loop when actively docking
        if self._state == _State.APPROACHING:
            self._process_frame_with_result(frame, result)

    # ── detection ────────────────────────────────────────────────────────

    def _detect(self, gray: np.ndarray):
        """Return (bearing_rad, distance_m, corners, ids, rvec, tvec) or None."""
        gray_c = np.ascontiguousarray(gray, dtype=np.uint8)
        corners, ids, _ = aruco.detectMarkers(
            gray_c, self._aruco_dict, parameters=self._det_params
        )
        if ids is None:
            return None

        cam = np.ascontiguousarray(self._cam_mtx, dtype=np.float64)
        dist = np.ascontiguousarray(self._dist_coeffs, dtype=np.float64)
        obj = np.ascontiguousarray(self._obj_pts, dtype=np.float64)

        for i, mid in enumerate(ids.flatten()):
            if int(mid) != self._target_id:
                continue
            img_pts = np.ascontiguousarray(corners[i][0], dtype=np.float64)
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    obj, img_pts, cam, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
            except cv2.error:
                return None
            if not ok or tvec[2, 0] <= 0:
                return None
            t = tvec.flatten()
            bearing = float(math.atan2(t[0], t[2]))
            distance = float(np.linalg.norm(t))
            return bearing, distance, corners, ids, rvec, tvec
        return None

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

            dist_err = self._distance_ema - self._dock_dist

            # Check done FIRST — before any control output
            # Use raw distance too (not just EMA) to catch overshoot
            at_dist = abs(dist_err) < self._dist_tol or distance_raw <= self._dock_dist
            at_bearing = abs(self._bearing_ema) < self._bearing_tol
            if at_dist and at_bearing:
                self._dwell_count += 1
                if self._dwell_count >= self._dwell_frames:
                    self._finish(success=True,
                                 reason=f"Docked at {self._distance_ema:.3f}m "
                                        f"(raw={distance_raw:.3f}m)")
                    return
            else:
                self._dwell_count = 0

            # If we're already at or past the dock distance, STOP immediately
            if self._distance_ema <= self._dock_dist + self._dist_tol:
                self._send_cmd(0.0, 0.0)
                dbg = (
                    f"d={self._distance_ema:.3f} raw={distance_raw:.3f} "
                    f"b={math.degrees(self._bearing_ema):+.1f} "
                    f"HOLDING — at dock dist, dwell={self._dwell_count}/{self._dwell_frames}"
                )
                self._debug_pub.publish(String(data=dbg))
                cv2.putText(debug_frame, f"HOLDING d={self._distance_ema:.2f}m dwell={self._dwell_count}/{self._dwell_frames}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                self._publish_debug_image(debug_frame)
                return

            # Integral (distance only, only accumulate when far)
            self._integral += dist_err * (1.0 / 30.0)
            max_int = 0.10 / max(self._ki_lin, 1e-6)
            self._integral = float(np.clip(self._integral, -max_int, max_int))

            # Control
            angular_z = float(np.clip(
                -self._kp_ang * self._bearing_ema, -self._max_ang, self._max_ang
            ))

            if abs(self._bearing_ema) < self._max_bearing:
                linear_x = float(np.clip(
                    self._kp_lin * dist_err + self._ki_lin * self._integral,
                    0.0, self._max_lin,
                ))
            else:
                linear_x = 0.0

            self._send_cmd(linear_x, angular_z)

            # Debug
            dbg = (
                f"d={self._distance_ema:.3f} raw={distance_raw:.3f} "
                f"b={math.degrees(self._bearing_ema):+.1f} "
                f"err={dist_err:+.3f} cmd=({linear_x:.3f},{angular_z:.3f}) "
                f"dwell={self._dwell_count}/{self._dwell_frames}"
            )
            self._debug_pub.publish(String(data=dbg))

            # Text overlay on debug image
            cv2.putText(debug_frame, f"d={self._distance_ema:.2f}m b={math.degrees(self._bearing_ema):+.1f}deg",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(debug_frame, f"err={dist_err:+.3f} dwell={self._dwell_count}/{self._dwell_frames}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
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
        self._state = _State.DONE if success else _State.FAILED
        self._done_pub.publish(Bool(data=success))
        level = self.get_logger().info if success else self.get_logger().warn
        level(f"Docking {'DONE' if success else 'FAILED'}: {reason}")

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
