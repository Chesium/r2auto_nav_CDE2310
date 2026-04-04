"""ArUco marker visual servoing for autonomous docking.

Ported from Arnav-Jhajharia/cde2310_sim_ws (tb3_cv) and adapted for:
  - ROS 2 Jazzy
  - TurtleBot3 Waffle (camera on /camera/image_raw)
  - Gazebo Harmonic (ogre2 rendering)

State machine:
  SEARCHING → LOCKING → TURN_TO_DOCK → DRIVE_TO_DOCK →
  TURN_TO_MARKER → TURN_RIGHT → STRAFE → TURN_LEFT → DONE
"""

import rclpy
from rclpy.node import Node
from enum import Enum, auto

import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


# --------------- tunables ---------------
MARKER_SIZE = 0.165       # marker square side (metres)
DOCK_DIST = 0.50          # stop distance from marker (metres)
STRAFE_DIST = 0.15        # lateral approach (metres)
TARGET_MARKER = 42        # ArUco ID to track
MAX_LINEAR = 0.12         # m/s
MAX_ANGULAR = 0.5         # rad/s
LOCK_N = 8                # pose samples before committing

ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

TURN_90_SECS = (np.pi / 2) / MAX_ANGULAR
STRAFE_SECS = STRAFE_DIST / MAX_LINEAR

MARKER_OBJECT_POINTS = np.array([
    [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
    [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
], dtype=np.float32)

# --------------- camera topics ---------------
# Adapt these if your camera publishes on different topics.
CAMERA_IMAGE_TOPIC = '/usb_cam/image_raw'
CAMERA_INFO_TOPIC = '/usb_cam/camera_info'


class State(Enum):
    SEARCHING = auto()
    LOCKING = auto()
    TURN_TO_DOCK = auto()
    DRIVE_TO_DOCK = auto()
    TURN_TO_MARKER = auto()
    TURN_RIGHT = auto()
    STRAFE = auto()
    TURN_LEFT = auto()
    DONE = auto()


def _heading_from_rvec(rvec: np.ndarray) -> float:
    """Yaw error: 0 when robot faces marker squarely.

    Positive → turn CCW (positive angular.z) to square up.
    Scale 1.5 compensates for PnP underestimation at shallow angles.
    """
    R, _ = cv2.Rodrigues(rvec)
    return float(np.arctan2(R[0, 2], -R[2, 2])) * 1.5


class ArucoDockNode(Node):

    def __init__(self):
        super().__init__('aruco_dock')

        self._bridge = CvBridge()
        self._camera_matrix = None
        self._dist_coeffs = None
        self._state = State.SEARCHING
        self._process_next = False
        self._last_process_ns = 0
        self._min_process_interval_ns = int(0.2 * 1e9)  # 5 Hz max

        # LOCKING accumulators
        self._lock_bearings: list[float] = []
        self._lock_dists: list[float] = []
        self._lock_headings: list[float] = []
        self._lock_ticks = 0

        # Navigation plan (set at commit_lock)
        self._target_bearing = 0.0
        self._target_dist = 0.0
        self._target_final_turn = 0.0
        self._phase_start_ns = 0

        # Subscriptions
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self._camera_info_cb, 10)
        self.create_subscription(
            Image, CAMERA_IMAGE_TOPIC, self._image_cb, 10)

        # Publishers
        self._debug_pub = self.create_publisher(
            Image, '/aruco_debug/image_raw', 10)
        self._cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # Timers
        self.create_timer(1.0, self._tick)
        self.create_timer(0.05, self._open_loop_tick)

        self.get_logger().info(
            f'ArUco dock node started — looking for marker {TARGET_MARKER} '
            f'on {CAMERA_IMAGE_TOPIC}')

    # ------------------------------------------------------------------ #
    # 1 Hz tick — controls SEARCHING and LOCKING timeout
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
        self._state = State.SEARCHING
        self._lock_bearings = []
        self._lock_dists = []
        self._lock_headings = []
        self._lock_ticks = 0

    def _commit_lock(self):
        """Compute navigation plan from accumulated pose samples."""
        bearing = float(np.median(self._lock_bearings))
        dist = float(np.median(self._lock_dists))
        heading = float(np.median(self._lock_headings))
        final_turn = heading + bearing

        self._target_bearing = bearing
        self._target_dist = dist
        self._target_final_turn = final_turn

        self.get_logger().info(
            f'Plan: turn {np.degrees(bearing):+.1f}deg -> '
            f'drive {dist:.3f}m -> '
            f'turn {np.degrees(final_turn):+.1f}deg to face marker')

        self._phase_start_ns = self.get_clock().now().nanoseconds
        self._state = State.TURN_TO_DOCK

    # ------------------------------------------------------------------ #
    # 50 Hz tick — open-loop velocity commands
    # ------------------------------------------------------------------ #

    def _open_loop_tick(self):
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self._phase_start_ns) / 1e9

        if self._state == State.TURN_TO_DOCK:
            duration = abs(self._target_bearing) / MAX_ANGULAR
            if elapsed >= duration:
                self._stop()
                self.get_logger().info('Turn to dock done — driving.')
                self._phase_start_ns = self.get_clock().now().nanoseconds
                self._state = State.DRIVE_TO_DOCK
            else:
                cmd = Twist()
                cmd.angular.z = float(
                    -MAX_ANGULAR * np.sign(self._target_bearing))
                self._cmd_vel_pub.publish(cmd)

        elif self._state == State.DRIVE_TO_DOCK:
            duration = self._target_dist / MAX_LINEAR
            if elapsed >= duration:
                self._stop()
                self.get_logger().info('At dock — turning to face marker.')
                self._phase_start_ns = self.get_clock().now().nanoseconds
                self._state = State.TURN_TO_MARKER
            else:
                cmd = Twist()
                cmd.linear.x = MAX_LINEAR
                self._cmd_vel_pub.publish(cmd)

        elif self._state == State.TURN_TO_MARKER:
            duration = abs(self._target_final_turn) / MAX_ANGULAR
            if elapsed >= duration:
                self._stop()
                self.get_logger().info('Facing marker — turning right.')
                self._phase_start_ns = self.get_clock().now().nanoseconds
                self._state = State.TURN_RIGHT
            else:
                cmd = Twist()
                cmd.angular.z = float(
                    MAX_ANGULAR * np.sign(self._target_final_turn))
                self._cmd_vel_pub.publish(cmd)

        elif self._state == State.TURN_RIGHT:
            if elapsed >= TURN_90_SECS:
                self._stop()
                self.get_logger().info('Turn right done — strafing.')
                self._phase_start_ns = self.get_clock().now().nanoseconds
                self._state = State.STRAFE
            else:
                cmd = Twist()
                cmd.angular.z = -MAX_ANGULAR
                self._cmd_vel_pub.publish(cmd)

        elif self._state == State.STRAFE:
            if elapsed >= STRAFE_SECS:
                self._stop()
                self.get_logger().info('Strafe done — turning left.')
                self._phase_start_ns = self.get_clock().now().nanoseconds
                self._state = State.TURN_LEFT
            else:
                cmd = Twist()
                cmd.linear.x = MAX_LINEAR
                self._cmd_vel_pub.publish(cmd)

        elif self._state == State.TURN_LEFT:
            if elapsed >= TURN_90_SECS:
                self._stop()
                self.get_logger().info('Docking complete.')
                self._state = State.DONE
            else:
                cmd = Twist()
                cmd.angular.z = MAX_ANGULAR
                self._cmd_vel_pub.publish(cmd)

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
        if self._state not in (State.SEARCHING, State.LOCKING):
            return
        now_ns = self.get_clock().now().nanoseconds
        if self._state == State.SEARCHING and not self._process_next:
            return
        if self._state == State.LOCKING:
            if (now_ns - self._last_process_ns) < self._min_process_interval_ns:
                return
        self._process_next = False
        self._last_process_ns = now_ns
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return
        self.get_logger().info(
            f'Processing frame {frame.shape} in state {self._state.name}')
        self._process_frame(frame, stamp=msg.header.stamp)

    # ------------------------------------------------------------------ #
    # Vision pipeline
    # ------------------------------------------------------------------ #

    def _detect_marker(self, gray: np.ndarray):
        detector_params = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(
            gray, ARUCO_DICT, parameters=detector_params)
        self.get_logger().info(
            f'detectMarkers: ids={ids.flatten().tolist() if ids is not None else None}')
        if ids is None:
            return None, None
        for i, mid in enumerate(ids.flatten()):
            if mid != TARGET_MARKER:
                continue
            image_points = corners[i][0].astype(np.float32)
            retval, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                MARKER_OBJECT_POINTS, image_points,
                self._camera_matrix, self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not retval:
                continue
            rvec, tvec = rvecs[0], tvecs[0]
            for r, t in zip(rvecs, tvecs):
                R_tmp, _ = cv2.Rodrigues(r)
                if R_tmp[2, 2] < 0:
                    rvec, tvec = r, t
                    break
            return rvec, tvec
        return None, None

    def _process_frame(self, frame: np.ndarray, stamp=None):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        debug = frame.copy()

        rvec, tvec = self._detect_marker(gray)
        if rvec is not None:
            cv2.drawFrameAxes(
                debug, self._camera_matrix, self._dist_coeffs,
                rvec, tvec, MARKER_SIZE * 0.5)

        if self._state == State.SEARCHING:
            if rvec is None:
                self.get_logger().info('No marker detected.')
                self._publish_debug(debug, stamp)
                return
            self.get_logger().info('Marker found — locking estimate.')
            self._lock_bearings = []
            self._lock_dists = []
            self._lock_headings = []
            self._lock_ticks = 0
            self._state = State.LOCKING

        if self._state == State.LOCKING:
            if rvec is not None:
                R_mat, _ = cv2.Rodrigues(rvec)
                dock_offset = (
                    DOCK_DIST * R_mat[:, 2].flatten() + tvec.flatten())
                dock_lat = float(dock_offset[0])
                dock_z = float(dock_offset[2])
                if dock_z > 0:
                    bearing = float(np.arctan2(dock_lat, dock_z))
                    dist = float(np.sqrt(dock_lat**2 + dock_z**2))
                    heading = _heading_from_rvec(rvec)
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
            cv2.putText(
                debug,
                f'LOCKING {len(self._lock_bearings)}/{LOCK_N}',
                (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        self._publish_debug(debug, stamp)

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _stop(self):
        self._cmd_vel_pub.publish(Twist())

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
