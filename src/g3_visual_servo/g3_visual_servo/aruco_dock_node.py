"""ArUco marker visual servoing for autonomous docking.

State machine:
  SEARCHING -> LOCKING -> STEP_SETTLE -> STEP_TURN / STEP_DRIVE -> STEP_SETTLE -> ... -> VERIFY -> DONE

Architecture: step-and-correct.
  Execute a fixed-duration burst (turn or drive), stop, wait for robot to
  physically settle, read PnP, decide next burst. Tolerates slow camera
  framerates and cmd_vel actuation latency.
"""

import rclpy
from rclpy.node import Node
from enum import Enum, auto

import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TwistStamped


# --------------- tunables ---------------
MARKER_SIZE   = 0.165
DOCK_DIST     = 0.30
TARGET_MARKER = 42

LOCK_N              = 8
BEARING_OUTLIER_DEG = 10.0

STEP_LINEAR_SECS  = 0.4
STEP_ANGULAR_SECS = 0.3
STEP_LINEAR_VEL   = 0.08
STEP_ANGULAR_VEL  = 0.25
STEP_SETTLE_SECS  = 0.6

APPROACH_DIST_TOL   = 0.06
ALIGN_ANGLE_TOL_DEG = 5.0
VERIFY_DIST_TOL     = 0.08

ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

MARKER_OBJECT_POINTS = np.array([
    [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
    [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
], dtype=np.float32)

CAMERA_IMAGE_TOPIC = '/usb_cam/image_raw'
CAMERA_INFO_TOPIC  = '/usb_cam/camera_info'


class State(Enum):
    SEARCHING   = auto()
    LOCKING     = auto()
    STEP_SETTLE = auto()
    STEP_TURN   = auto()
    STEP_DRIVE  = auto()
    VERIFY      = auto()
    DONE        = auto()
    FAILED      = auto()


def _heading_from_rvec(rvec):
    R, _ = cv2.Rodrigues(rvec)
    return float(np.arctan2(R[0, 2], R[2, 2]))


class ArucoDockNode(Node):

    def __init__(self):
        super().__init__('aruco_dock')

        self._bridge        = CvBridge()
        self._camera_matrix = None
        self._dist_coeffs   = None
        self._state         = State.SEARCHING

        self._process_next            = False
        self._last_process_ns         = 0
        self._min_process_interval_ns = int(0.1 * 1e9)

        self._detector_params = aruco.DetectorParameters()

        self._lock_bearings  = []
        self._lock_dists     = []
        self._lock_headings  = []
        self._lock_ticks     = 0
        self._lock_committed = False

        self._step_start_ns = 0
        self._lost_frames   = 0

        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._camera_info_cb, 10)
        self.create_subscription(Image,      CAMERA_IMAGE_TOPIC, self._image_cb,       10)

        self._debug_pub   = self.create_publisher(Image,        '/aruco_debug/image_raw', 10)
        self._cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel',               10)

        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f'ArUco dock node started - looking for marker {TARGET_MARKER} '
            f'on {CAMERA_IMAGE_TOPIC}')

    def _tick(self):
        if self._state == State.SEARCHING:
            self._process_next = True
        elif self._state == State.LOCKING:
            self._lock_ticks += 1
            if self._lock_ticks >= 5 and len(self._lock_bearings) > 0:
                self.get_logger().warn(
                    f'LOCKING timeout - committing {len(self._lock_bearings)} samples.')
                self._commit_lock()

    def _reset(self):
        self._state          = State.SEARCHING
        self._lock_bearings  = []
        self._lock_dists     = []
        self._lock_headings  = []
        self._lock_ticks     = 0
        self._lock_committed = False
        self._lost_frames    = 0

    def _commit_lock(self):
        if self._lock_committed:
            return
        self._lock_committed = True

        bearing = float(np.median(self._lock_bearings))
        dist    = float(np.median(self._lock_dists))
        heading = float(np.median(self._lock_headings))

        self.get_logger().info(
            f'Lock committed: bearing={np.degrees(bearing):+.1f}deg  '
            f'dist={dist:.3f}m  heading={np.degrees(heading):+.1f}deg  -> STEP_SETTLE')

        self._lost_frames   = 0
        self._step_start_ns = self.get_clock().now().nanoseconds
        self._state         = State.STEP_SETTLE

    def _end_step(self):
        if self._state in (State.STEP_TURN, State.STEP_DRIVE):
            self._stop()
            self.get_logger().info('Step done - settling.')
            self._step_start_ns = self.get_clock().now().nanoseconds
            self._state = State.STEP_SETTLE

    def _start_turn(self, bearing_rad):
        duration = min(abs(bearing_rad) / STEP_ANGULAR_VEL, STEP_ANGULAR_SECS)
        self._send_cmd(angular_z=-STEP_ANGULAR_VEL * np.sign(bearing_rad))
        self._state = State.STEP_TURN
        self.create_timer(duration, lambda: self._end_step())

    def _start_drive(self):
        self._send_cmd(linear_x=STEP_LINEAR_VEL)
        self._state = State.STEP_DRIVE
        self.create_timer(STEP_LINEAR_SECS, lambda: self._end_step())

    def _camera_info_cb(self, msg):
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._dist_coeffs   = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received.')

    def _image_cb(self, msg):
        if self._camera_matrix is None:
            return
        if self._state not in (
                State.SEARCHING,   State.LOCKING,
                State.STEP_SETTLE, State.STEP_TURN,
                State.STEP_DRIVE,  State.VERIFY):
            return

        now_ns = self.get_clock().now().nanoseconds
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

    def _detect_marker(self, gray):
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
            rvec, tvec = rvecs[0], tvecs[0]
            for r, t in zip(rvecs, tvecs):
                if t[2, 0] > 0:
                    rvec, tvec = r, t
                    break
            return rvec, tvec, corners, ids, rejected

        return None, None, corners, ids, rejected

    def _process_frame(self, frame, stamp=None):
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        debug = frame.copy()

        rvec, tvec, corners, ids, rejected = self._detect_marker(gray)

        if ids is not None:
            aruco.drawDetectedMarkers(debug, corners, ids)
        if rejected:
            for rej in rejected:
                pts = rej[0].astype(int)
                for j in range(4):
                    cv2.line(debug, tuple(pts[j]), tuple(pts[(j+1) % 4]), (0, 0, 255), 1)

        if rvec is not None:
            cv2.drawFrameAxes(
                debug, self._camera_matrix, self._dist_coeffs,
                rvec, tvec, MARKER_SIZE * 0.5)
            t    = tvec.flatten()
            dist = float(np.linalg.norm(t))
            hdg  = _heading_from_rvec(rvec)
            cv2.putText(debug,
                f'dist={dist:.2f}m hdg={np.degrees(hdg):+.1f}deg',
                (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.putText(debug, f'State: {self._state.name}',
            (5, debug.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        if self._state == State.SEARCHING:
            if rvec is None:
                self.get_logger().info('No marker detected.')
                self._publish_debug(debug, stamp)
                return
            self.get_logger().info('Marker found - beginning LOCKING.')
            self._lock_bearings  = []
            self._lock_dists     = []
            self._lock_headings  = []
            self._lock_ticks     = 0
            self._lock_committed = False
            self._state          = State.LOCKING

        if self._state == State.LOCKING:
            if rvec is not None:
                R_mat, _ = cv2.Rodrigues(rvec)
                dock_offset = DOCK_DIST * R_mat[:, 2].flatten() + tvec.flatten()
                dock_lat    = float(dock_offset[0])
                dock_z      = float(dock_offset[2])

                if dock_z > 0:
                    bearing = float(np.arctan2(dock_lat, dock_z))
                    dist    = float(np.sqrt(dock_lat**2 + dock_z**2))
                    heading = _heading_from_rvec(rvec)

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

        if self._state in (State.STEP_TURN, State.STEP_DRIVE):
            self._publish_debug(debug, stamp)
            return

        if self._state == State.STEP_SETTLE:
            now_ns  = self.get_clock().now().nanoseconds
            elapsed = (now_ns - self._step_start_ns) / 1e9
            if elapsed < STEP_SETTLE_SECS:
                self._publish_debug(debug, stamp)
                return

            if rvec is None:
                self._lost_frames += 1
                self.get_logger().warn(
                    f'STEP_SETTLE: no marker ({self._lost_frames}/5)')
                if self._lost_frames >= 5:
                    self.get_logger().error('Cannot see marker - aborting.')
                    self._state = State.FAILED
                self._publish_debug(debug, stamp)
                return

            self._lost_frames = 0
            t       = tvec.flatten()
            dist    = float(np.linalg.norm(t))
            lateral = t[0]
            bearing = float(np.arctan2(lateral, t[2]))

            self.get_logger().info(
                f'SETTLE read: dist={dist:.3f}m  bearing={np.degrees(bearing):+.1f}deg')

            if dist <= DOCK_DIST + APPROACH_DIST_TOL:
                self.get_logger().info('Within dock distance -> VERIFY.')
                self._state = State.VERIFY
                self._publish_debug(debug, stamp)
                return

            if abs(np.degrees(bearing)) > ALIGN_ANGLE_TOL_DEG:
                self.get_logger().info(f'Turning {np.degrees(bearing):+.1f}deg.')
                self._start_turn(bearing)
            else:
                self.get_logger().info(f'Driving step (dist={dist:.3f}m).')
                self._start_drive()

            self._publish_debug(debug, stamp)
            return

        if self._state == State.VERIFY:
            if rvec is None:
                self.get_logger().warn('VERIFY: no marker - resetting.')
                self._reset()
                self._publish_debug(debug, stamp)
                return

            t    = tvec.flatten()
            dist = float(np.linalg.norm(t))
            self.get_logger().info(f'VERIFY: dist={dist:.3f}m')

            if abs(dist - DOCK_DIST) <= VERIFY_DIST_TOL:
                self._stop()
                self.get_logger().info('Docking VERIFIED - DONE.')
                self._state = State.DONE
            else:
                self.get_logger().warn(
                    f'VERIFY failed (dist={dist:.3f}m) - resetting.')
                self._reset()

            self._publish_debug(debug, stamp)
            return

        self._publish_debug(debug, stamp)

    def _send_cmd(self, linear_x=0.0, angular_z=0.0):
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

    def _publish_debug(self, frame, stamp=None):
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