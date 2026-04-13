#!/usr/bin/env python3
"""
aruco_dock_pose_publisher.py
============================
Minimal ArUco detector that publishes the target marker's pose as
``geometry_msgs/PoseStamped`` on ``/detected_dock_pose`` for consumption by
Nav2's ``opennav_docking`` server (``SimpleNonChargingDock`` plugin with
``use_external_detection_pose: true``).

Responsibilities
----------------
* Subscribe to a camera image topic + camera_info topic.
* On each frame, detect ArUco markers. If the configured ``target_marker_id``
  is present, run solvePnP to get the marker pose in the camera optical frame.
* Publish the pose on ``/detected_dock_pose`` with the image's own timestamp
  so the docking server's ``external_detection_timeout`` freshness check
  works correctly.
* Nothing else. No state machine. No ``/cmd_vel``. No services. No TF lookups.
  No station pose publishing. This node is purely detection → republish.

Intended usage
--------------
Run alongside (not inside) a Nav2 stack that already has ``docking_server``
active. See the launch files in this package for sim + hardware wiring.

Parameters (all declared, override via launch or CLI)
-----------------------------------------------------
* ``image_topic``        (str)   camera image topic. Default ``/camera/image_raw``.
* ``camera_info_topic``  (str)   camera info topic. Default ``/camera/camera_info``.
* ``dock_pose_topic``    (str)   output pose topic. Default ``/detected_dock_pose``.
* ``target_marker_id``   (int)   which ArUco id to track. Default 42.
* ``marker_size``        (float) physical marker edge length in meters. Default 0.165.
* ``dictionary``         (str)   cv2.aruco dictionary name (e.g. DICT_4X4_100).
* ``output_frame_id``    (str)   override header.frame_id on the published pose.
                                 Empty string means reuse the image header's
                                 frame_id (preferred). Default "".
"""

from __future__ import annotations

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


# 3D object points for a square planar marker centered at the origin,
# lying in the XY plane with +Z pointing out of the marker face.
# Same layout as aruco_dock_node.py so behaviour matches.
def _marker_object_points(size: float) -> np.ndarray:
    half = size / 2.0
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def _resolve_dictionary(name: str):
    """Map a string like 'DICT_4X4_100' to a cv2.aruco dictionary object.

    Handles both the OpenCV 4.6 (Dictionary_get) and 4.7+ (getPredefinedDictionary)
    APIs, matching the compatibility shim in aruco_dock_node.py.
    """
    const = getattr(aruco, name, None)
    if const is None:
        raise ValueError(f"Unknown cv2.aruco dictionary name: {name!r}")
    if hasattr(aruco, "Dictionary_get"):
        return aruco.Dictionary_get(const)
    return aruco.getPredefinedDictionary(const)


def _make_detector_params():
    """Create a cv2.aruco detector parameter object across OpenCV versions."""
    if hasattr(aruco, "DetectorParameters_create"):
        return aruco.DetectorParameters_create()
    return aruco.DetectorParameters()


class ArucoDockPosePublisher(Node):
    """Pure detection → PoseStamped republisher for Nav2's docking server."""

    def __init__(self) -> None:
        super().__init__("aruco_dock_pose_publisher")

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("dock_pose_topic", "/detected_dock_pose")
        self.declare_parameter("target_marker_id", 42)
        self.declare_parameter("marker_size", 0.165)
        self.declare_parameter("dictionary", "DICT_4X4_100")
        self.declare_parameter("output_frame_id", "")
        self.declare_parameter("dock_offset_z", 0.30)  # stop this far in front of marker

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        dock_pose_topic = self.get_parameter("dock_pose_topic").value
        self._target_id: int = int(self.get_parameter("target_marker_id").value)
        marker_size: float = float(self.get_parameter("marker_size").value)
        dictionary_name: str = str(self.get_parameter("dictionary").value)
        self._output_frame_override: str = str(
            self.get_parameter("output_frame_id").value
        )
        self._dock_offset_z: float = float(
            self.get_parameter("dock_offset_z").value
        )

        # ── Detection resources ──────────────────────────────────────────
        self._bridge = CvBridge()
        self._object_points = _marker_object_points(marker_size)
        self._aruco_dict = _resolve_dictionary(dictionary_name)
        self._detector_params = _make_detector_params()

        # ── Camera intrinsics (populated from CameraInfo) ────────────────
        self._camera_matrix: np.ndarray | None = None
        self._dist_coeffs: np.ndarray | None = None

        # ── Diagnostics ──────────────────────────────────────────────────
        self._frames_seen = 0
        self._detections_published = 0
        self._last_log_time = self.get_clock().now()

        # ── ROS I/O ──────────────────────────────────────────────────────
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(Image, image_topic, self._image_cb, 10)
        self._pose_pub = self.create_publisher(PoseStamped, dock_pose_topic, 10)
        self._debug_pub = self.create_publisher(String, "/aruco_debug", 10)

        self.get_logger().info(
            f"aruco_dock_pose_publisher up. image={image_topic!r} "
            f"info={camera_info_topic!r} out={dock_pose_topic!r} "
            f"id={self._target_id} size={marker_size}m dict={dictionary_name}"
        )

    # ── Callbacks ────────────────────────────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if self._camera_matrix is not None:
            return
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        distortion = np.array(msg.d, dtype=np.float64)
        self._dist_coeffs = distortion if distortion.size > 0 else np.zeros(5, dtype=np.float64)
        self.get_logger().info("Camera intrinsics received.")

    def _image_cb(self, msg: Image) -> None:
        if self._camera_matrix is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001 — cv_bridge raises many exception types
            self.get_logger().error(f"cv_bridge failed: {exc}")
            return

        self._frames_seen += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pose = self._detect_target_pose(gray)

        if pose is not None:
            out = PoseStamped()
            out.header.stamp = msg.header.stamp  # freshness matters to docking_server
            out.header.frame_id = (
                self._output_frame_override or msg.header.frame_id
            )
            position, quaternion = pose
            dist_3d = float(np.linalg.norm(position))
            dist_2d = float(np.sqrt(position[0]**2 + position[2]**2))
            out.pose.position.x = float(position[0])
            out.pose.position.y = float(position[1])
            out.pose.position.z = float(position[2])
            out.pose.orientation.x = float(quaternion[0])
            out.pose.orientation.y = float(quaternion[1])
            out.pose.orientation.z = float(quaternion[2])
            out.pose.orientation.w = float(quaternion[3])
            self._pose_pub.publish(out)
            self._detections_published += 1

            # Debug: publish distance + pose info for Foxglove
            dbg = String()
            dbg.data = (
                f"d3d={dist_3d:.3f} d2d={dist_2d:.3f} "
                f"x={position[0]:.3f} y={position[1]:.3f} z={position[2]:.3f} "
                f"qx={quaternion[0]:.3f} qy={quaternion[1]:.3f} "
                f"qz={quaternion[2]:.3f} qw={quaternion[3]:.3f}"
            )
            self._debug_pub.publish(dbg)
        else:
            dbg = String()
            dbg.data = "NO_DETECTION"
            self._debug_pub.publish(dbg)

        self._log_rate_if_due()

    # ── Detection pipeline ───────────────────────────────────────────────

    def _detect_target_pose(
        self, gray: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """Return (translation_xyz, quaternion_xyzw) for the target marker, or None."""
        # OpenCV 4.6 segfaults on non-contiguous arrays; match aruco_dock_node.py
        gray_contig = np.ascontiguousarray(gray, dtype=np.uint8)
        corners, ids, _rejected = aruco.detectMarkers(
            gray_contig, self._aruco_dict, parameters=self._detector_params
        )

        if ids is None:
            return None

        # Convert camera matrix + distortion to contiguous float64 (OpenCV 4.6 fix)
        cam_mtx = np.ascontiguousarray(self._camera_matrix, dtype=np.float64)
        dist = np.ascontiguousarray(self._dist_coeffs, dtype=np.float64)
        obj_pts = np.ascontiguousarray(self._object_points, dtype=np.float64)

        for i, marker_id in enumerate(ids.flatten()):
            if int(marker_id) != self._target_id:
                continue
            image_points = np.ascontiguousarray(corners[i][0], dtype=np.float64)
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts,
                    image_points,
                    cam_mtx,
                    dist,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
            except cv2.error:
                return None
            if not ok:
                return None
            if tvec[2, 0] <= 0:
                return None  # marker behind camera → bogus

            translation = tvec.flatten()
            # Offset target toward camera so robot stops
            # dock_offset_z meters in front of the marker.
            # Camera optical frame: Z = depth (forward).
            translation[2] = max(0.01, translation[2] - self._dock_offset_z)
            # Don't use solvePnP orientation — it encodes the marker's
            # own frame which confuses the docking controller.  Instead
            # compute a "face the marker" orientation: dock X-axis
            # points from camera toward the marker in the camera optical
            # frame's XZ plane (Z = forward, X = right).
            quaternion = _approach_quaternion(translation)
            return translation, quaternion

        return None

    # ── Diagnostics helpers ──────────────────────────────────────────────

    def _log_rate_if_due(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self._last_log_time).nanoseconds / 1e9
        if elapsed < 2.0:
            return
        if self._frames_seen == 0:
            return
        det_rate = self._detections_published / elapsed
        frame_rate = self._frames_seen / elapsed
        self.get_logger().info(
            f"frames={self._frames_seen} dets={self._detections_published} "
            f"frame_rate={frame_rate:.1f}Hz det_rate={det_rate:.1f}Hz"
        )
        self._frames_seen = 0
        self._detections_published = 0
        self._last_log_time = now


def _approach_quaternion(translation: np.ndarray) -> np.ndarray:
    """Compute a dock orientation from the marker's position in camera optical frame.

    The dock's X-axis should point from the camera toward the marker so
    that the docking controller drives the robot forward to the marker.

    In camera optical frame: Z = forward, X = right, Y = down.
    We rotate the standard X-axis (1,0,0) to point toward (tx, 0, tz)
    via a rotation around Y.
    """
    tx, tz = float(translation[0]), float(translation[2])
    # Angle to rotate X-axis toward (tx, 0, tz) around Y
    theta = -np.arctan2(tz, tx)
    qy = np.sin(theta / 2.0)
    qw = np.cos(theta / 2.0)
    return np.array([0.0, qy, 0.0, qw], dtype=np.float64)


def _rvec_to_quaternion(rvec: np.ndarray) -> np.ndarray:
    """Convert a Rodrigues rotation vector to a quaternion (x, y, z, w)."""
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    # Standard matrix-to-quaternion conversion. Using OpenCV rather than
    # tf_transformations so we don't add a new runtime dependency.
    m = rotation_matrix
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ArucoDockPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
