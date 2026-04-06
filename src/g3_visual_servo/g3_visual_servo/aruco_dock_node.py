"""ArUco marker visual servoing for autonomous docking.

Ported from Arnav-Jhajharia/cde2310_sim_ws (tb3_cv) and adapted for:
  - ROS 2 Jazzy
  - TurtleBot3 Waffle (camera on /camera/image_raw)
  - Gazebo Harmonic (ogre2 rendering)

State machine:
  SEARCHING → LOCKING → STEP_SETTLE → STEP_TURN / STEP_DRIVE → STEP_SETTLE → ... → VERIFY → DONE

Architecture: step-and-correct.
  The robot executes a fixed-duration burst (turn or drive), stops, waits for
  the robot to physically settle, reads PnP, then decides the next burst.
  This tolerates slow camera framerates and cmd_vel actuation latency.
"""

import rclpy
from rclpy.node import Node
from enum import Enum, auto

import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, TwistStamped


# --------------- tunables ---------------
MARKER_SIZE   = 0.165   # marker square side (metres)
DOCK_DIST     = 0.30    # desired stop distance from marker (metres)
TARGET_MARKER = 42      # ArUco ID to track
APPROACH_DIST_TOL   = 0.06   # metres  — within this → VERIFY
ALIGN_ANGLE_TOL_DEG = 5.0    # degrees — within this → drive step

LOCK_N              = 8      # pose samples before committing
BEARING_OUTLIER_DEG = 10.0   # [FIX-7] reject samples this far from running median

# Step-and-correct parameters
STEP_LINEAR_SECS  = 0.4   # drive burst duration (seconds)
STEP_ANGULAR_SECS = 0.3   # turn burst duration (seconds)
STEP_LINEAR_VEL   = 0.08  # m/s during drive burst
STEP_ANGULAR_VEL  = 0.25  # rad/s during turn burst
STEP_SETTLE_SECS  = 0.5   # pause after burst before reading PnP

ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

MARKER_OBJECT_POINTS = np.array([
    [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
    [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
], dtype=np.float32)

CAMERA_IMAGE_TOPIC = '/usb_cam/image_raw'
CAMERA_INFO_TOPIC  = '/usb_cam/camera_info'

# How many consecutive frames we can lose the marker before aborting
MAX_LOST_FRAMES = 15


class State(Enum):
    SEARCHING    = auto()
    LOCKING      = auto()
    STEP_TURN    = auto()   # burst: rotate toward bearing
    STEP_DRIVE   = auto()   # burst: drive forward
    STEP_SETTLE  = auto()   # pause: wait for robot to stop, then read PnP
    VERIFY       = auto()   # within tolerance: final PnP check
    DONE         = auto()
    FAILED       = auto()


def _heading_from_rvec(rvec: np.ndarray) -> float:
    """Yaw error in radians: 0 when robot faces marker squarely.

    Derived from the marker's rotation matrix column that represents
    the marker normal (Z-axis) projected into the camera XZ plane.
    Positive → turn CCW (positive angular.z) to square up.

    [FIX-3] Removed unjustified * 1.5 scale factor.
    The correct formulation: the angle between the marker's Z-axis
    projection onto the camera XZ-plane and the camera's own Z-axis.
    """
    R, _ = cv2.Rodrigues(rvec)
    # R[:,2] is the marker Z-axis in camera frame.
    # Project onto XZ plane and compute signed angle.
    return float(np.arctan2(R[0, 2], R[2, 2]))


class ArucoDockNode(Node):

    def __init__(self):
        super().__init__('aruco_dock')

        self._bridge         = CvBridge()
        self._camera_matrix  = None
        self._dist_coeffs    = None
        self._state          = State.SEARCHING
        self._process_next   = False
        self._last_process_ns = 0
        self._min_process_interval_ns = int(0.1 * 1e9)  # 10 Hz max

        # Debian-patched OpenCV 4.6 ships DetectorParameters() only —
        # DetectorParameters_create() was removed upstream.
        self._detector_params = aruco.DetectorParameters()

        # LOCKING accumulators
        self._lock_bearings: list[float] = []
        self._lock_dists:    list[float] = []
        self._lock_headings: list[float] = []
        self._lock_ticks = 0

        # [FIX-5] Guard against double-commit
        self._lock_committed = False

        # Step-and-correct state
        self._lost_frames  = 0
        self._step_start_ns = 0

        # Subscriptions
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self._camera_info_cb, 10)
        self.create_subscription(
            Image, CAMERA_IMAGE_TOPIC, self._image_cb, 10)

        # Publishers
        self._debug_pub   = self.create_publisher(
            Image, '/aruco_debug/image_raw', 10)
        self._cmd_vel_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', 10)

        # 1 Hz management tick (SEARCHING pulse + LOCKING timeout)
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f'ArUco dock node started — looking for marker {TARGET_MARKER} '
            f'on {CAMERA_IMAGE_TOPIC}')

    # ------------------------------------------------------------------ #
    # 1 Hz management tick
    # ------------------------------------------------------------------ #

    def _tick(self):
        if self._state == State.SEARCHING:
            self._process_next = True

        elif self._state == State.LOCKING:
            self._lock_ticks += 1
            if self._lock_ticks >= 5 and len(self._lock_bearings) > 0:
                self.get_logger().warn(
                    f'LOCKING timeout — committing '
                    f'{len(self._lock_bearings)} samples.')
                self._commit_lock()

    def _reset(self):
        self._state           = State.SEARCHING
        self._lock_bearings   = []
        self._lock_dists      = []
        self._lock_headings   = []
        self._lock_ticks      = 0
        self._lock_committed  = False
        self._lost_frames     = 0

    def _commit_lock(self):
        """Transition to closed-loop APPROACH using median of accumulated samples."""
        # [FIX-5] Idempotent — ignore if already committed this lock cycle.
        if self._lock_committed:
            return
        self._lock_committed = True

        bearing = float(np.median(self._lock_bearings))
        dist    = float(np.median(self._lock_dists))
        heading = float(np.median(self._lock_headings))

        self.get_logger().info(
            f'Lock committed: bearing={np.degrees(bearing):+.1f}deg  '
            f'dist={dist:.3f}m  heading={np.degrees(heading):+.1f}deg  '
            f'→ STEP_SETTLE for initial PnP read')

        self._lost_frames  = 0
        self._step_start_ns = self.get_clock().now().nanoseconds
        self._state = State.STEP_SETTLE

    # ------------------------------------------------------------------ #
    # Camera callbacks
    # ------------------------------------------------------------------ #

    def _camera_info_cb(self, msg: CameraInfo):
        if self._camera_matrix is None:
            self._camera_matrix = np.array(
                msg.k, dtype=np.float64).reshape(3, 3)
            self._dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received.')

    def _image_cb(self, msg: Image):
        if self._camera_matrix is None:
            return
        if self._state not in (
                State.SEARCHING, State.LOCKING,
                State.APPROACH,  State.ALIGN, State.VERIFY):
            return

        now_ns = self.get_clock().now().nanoseconds

        # Rate-limit processing
        if self._state == State.SEARCHING and not self._process_next:
            return
        if (now_ns - self._last_process_ns) < self._min_process_interval_ns:
            return

        self._process_next    = False
        self._last_process_ns = now_ns

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return

        self._process_frame(frame, stamp=msg.header.stamp)

    # ------------------------------------------------------------------ #
    # Vision pipeline
    # ------------------------------------------------------------------ #

    def _detect_marker(self, gray: np.ndarray):
        """Detect TARGET_MARKER and return (rvec, tvec) or (None, None).

        [FIX-2] Solution selection: pick the IPPE_SQUARE solution where
        tvec[2] > 0 (marker is in front of the camera). The original
        R[2,2] < 0 check was incorrect and could silently fall back to
        the wrong solution.
        """
        corners, ids, rejected = aruco.detectMarkers(
            gray, ARUCO_DICT, parameters=self._detector_params)

        self.get_logger().info(
            f'detectMarkers: ids={ids.flatten().tolist() if ids is not None else None}')

        if ids is None:
            return None, None, corners, ids, rejected

        for i, mid in enumerate(ids.flatten()):
            if mid != TARGET_MARKER:
                continue

            image_points = corners[i][0].astype(np.float32)
            retval, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                MARKER_OBJECT_POINTS, image_points,
                self._camera_matrix, self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not retval or len(rvecs) == 0:
                continue

            # [FIX-2] Select solution with marker in front of camera (tvec Z > 0).
            rvec, tvec = rvecs[0], tvecs[0]  # fallback to first
            for r, t in zip(rvecs, tvecs):
                if t[2, 0] > 0:
                    rvec, tvec = r, t
                    break

            return rvec, tvec, corners, ids, rejected

        return None, None, corners, ids, rejected

    def _process_frame(self, frame: np.ndarray, stamp=None):
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        debug = frame.copy()

        rvec, tvec, corners, ids, rejected = self._detect_marker(gray)

        if ids is not None:
            aruco.drawDetectedMarkers(debug, corners, ids)
        if rejected:
            for rej in rejected:
                pts = rej[0].astype(int)
                for j in range(4):
                    cv2.line(debug, tuple(pts[j]), tuple(pts[(j+1) % 4]),
                             (0, 0, 255), 1)

        if rvec is not None:
            cv2.drawFrameAxes(
                debug, self._camera_matrix, self._dist_coeffs,
                rvec, tvec, MARKER_SIZE * 0.5)
            t       = tvec.flatten()
            dist    = float(np.linalg.norm(t))
            heading = _heading_from_rvec(rvec)
            lateral = t[0]
            cv2.putText(debug,
                f'dist={dist:.2f}m hdg={np.degrees(heading):+.1f}deg lat={lateral:+.3f}m',
                (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.putText(debug, f'State: {self._state.name}',
            (5, debug.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (255, 255, 255), 1)

        # ---- SEARCHING ----
        if self._state == State.SEARCHING:
            if rvec is None:
                self.get_logger().info('No marker detected.')
                self._publish_debug(debug, stamp)
                return
            self.get_logger().info('Marker found — beginning LOCKING.')
            self._lock_bearings  = []
            self._lock_dists     = []
            self._lock_headings  = []
            self._lock_ticks     = 0
            self._lock_committed = False
            self._state          = State.LOCKING

        # ---- LOCKING ----
        if self._state == State.LOCKING:
            if rvec is not None:
                R_mat, _ = cv2.Rodrigues(rvec)
                # Point DOCK_DIST in front of marker face
                dock_offset = DOCK_DIST * R_mat[:, 2].flatten() + tvec.flatten()
                dock_lat    = float(dock_offset[0])
                dock_z      = float(dock_offset[2])

                if dock_z > 0:
                    bearing = float(np.arctan2(dock_lat, dock_z))
                    dist    = float(np.sqrt(dock_lat**2 + dock_z**2))
                    heading = _heading_from_rvec(rvec)

                    # [FIX-7] Reject outliers relative to running median
                    accept = True
                    if len(self._lock_bearings) >= 2:
                        med = float(np.median(self._lock_bearings))
                        if abs(np.degrees(bearing - med)) > BEARING_OUTLIER_DEG:
                            self.get_logger().warn(
                                f'LOCKING: outlier rejected '
                                f'(bearing={np.degrees(bearing):+.1f}deg '
                                f'vs median={np.degrees(med):+.1f}deg)')
                            accept = False

                    if accept:
                        self._lock_bearings.append(bearing)
                        self._lock_dists.append(dist)
                        self._lock_headings.append(heading)
                        self.get_logger().info(
                            f'  LOCKING [{len(self._lock_bearings)}/{LOCK_N}]  '
                            f'bearing={np.degrees(bearing):+.1f}deg  '
                            f'dist={dist:.3f}m  '
                            f'heading={np.degrees(heading):+.1f}deg')
                        if len(self._lock_bearings) >= LOCK_N:
                            self._commit_lock()

            cv2.putText(debug,
                f'LOCKING {len(self._lock_bearings)}/{LOCK_N}',
                (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            self._publish_debug(debug, stamp)
            return

        # ---- STEP_SETTLE — robot has stopped, read PnP and decide next action ----
        if self._state == State.STEP_SETTLE:
            now_ns = self.get_clock().now().nanoseconds
            elapsed = (now_ns - self._step_start_ns) / 1e9
            if elapsed < STEP_SETTLE_SECS:
                # Not settled yet — wait
                self._publish_debug(debug, stamp)
                return

            if rvec is None:
                self._lost_frames += 1
                self.get_logger().warn(
                    f'STEP_SETTLE: no marker ({self._lost_frames}/5)')
                if self._lost_frames >= 5:
                    self.get_logger().error('Cannot see marker — aborting.')
                    self._state = State.FAILED
                self._publish_debug(debug, stamp)
                return

            self._lost_frames = 0
            t       = tvec.flatten()
            dist    = float(np.linalg.norm(t))
            lateral = t[0]
            bearing = float(np.arctan2(lateral, t[2]))
            heading = _heading_from_rvec(rvec)

            self.get_logger().info(
                f'SETTLE read: dist={dist:.3f}m  '
                f'bearing={np.degrees(bearing):+.1f}deg  '
                f'heading={np.degrees(heading):+.1f}deg')

            # Within dock tolerance → VERIFY
            if dist <= DOCK_DIST + APPROACH_DIST_TOL:
                self.get_logger().info('Within dock distance → VERIFY.')
                self._state = State.VERIFY
                self._publish_debug(debug, stamp)
                return

            # Need to turn first to face the dock point
            if abs(np.degrees(bearing)) > ALIGN_ANGLE_TOL_DEG:
                self.get_logger().info(
                    f'Turning {np.degrees(bearing):+.1f}deg to face dock point.')
                duration = abs(bearing) / STEP_ANGULAR_VEL
                turn_secs = min(duration, STEP_ANGULAR_SECS)
                self._send_cmd(
                    angular_z=-STEP_ANGULAR_VEL * np.sign(bearing))
                self._step_start_ns = self.get_clock().now().nanoseconds
                # Schedule stop + settle
                self.create_timer(
                    turn_secs,
                    lambda: self._end_step())
                self._state = State.STEP_TURN
            else:
                # Heading acceptable — drive a step forward
                self.get_logger().info(
                    f'Driving step toward marker (dist={dist:.3f}m).')
                self._send_cmd(linear_x=STEP_LINEAR_VEL)
                self._step_start_ns = self.get_clock().now().nanoseconds
                self.create_timer(
                    STEP_LINEAR_SECS,
                    lambda: self._end_step())
                self._state = State.STEP_DRIVE

            self._publish_debug(debug, stamp)
            return

        # STEP_TURN and STEP_DRIVE are timer-driven — image_cb does nothing
        if self._state in (State.STEP_TURN, State.STEP_DRIVE):
            self._publish_debug(debug, stamp)
            return

        # ---- VERIFY — final sanity check before declaring DONE ----
        # [FIX-6] One-shot PnP verification. Reset to SEARCHING if tolerance not met.
        if self._state == State.VERIFY:
            if rvec is None:
                self.get_logger().warn('VERIFY: no marker — retrying APPROACH.')
                self._reset()
                self._publish_debug(debug, stamp)
                return

            t    = tvec.flatten()
            dist = float(np.linalg.norm(t))
            self.get_logger().info(
                f'VERIFY: final dist={dist:.3f}m '
                f'(tol±{VERIFY_DIST_TOL}m from {DOCK_DIST}m)')

            if abs(dist - DOCK_DIST) <= VERIFY_DIST_TOL:
                self._stop()
                self.get_logger().info('Docking VERIFIED — DONE.')
                self._state = State.DONE
            else:
                self.get_logger().warn(
                    f'VERIFY failed (dist={dist:.3f}m) — resetting to SEARCHING.')
                self._reset()

            self._publish_debug(debug, stamp)
            return

        self._publish_debug(debug, stamp)

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _end_step(self):
        """Stop the robot and transition to STEP_SETTLE for next PnP read."""
        if self._state in (State.STEP_TURN, State.STEP_DRIVE):
            self._stop()
            self.get_logger().info('Step done — settling.')
            self._step_start_ns = self.get_clock().now().nanoseconds
            self._state = State.STEP_SETTLE

    def _send_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x  = linear_x
        cmd.twist.angular.z = angular_z
        self._cmd_vel_pub.publish(cmd)
        if linear_x != 0.0 or angular_z != 0.0:
            self.get_logger().info(
                f'CMD_VEL: linear={linear_x:.3f} angular={angular_z:.3f}')

    def _stop(self):
        self._send_cmd()

    def _publish_debug(self, frame: np.ndarray, stamp=None):
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        if stamp is not None:
            msg.header.stamp = stamp
        msg.header.frame_id = 'camera_rgb_optical_frame'
        self._debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDockNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()