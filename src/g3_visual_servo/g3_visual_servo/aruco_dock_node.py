"""ArUco marker visual servoing for autonomous docking.

Supports two markers:
  42 -> Station A  (/station_a_pose)
  67 -> Station B  (/station_b_pose)

Two modes:
  Passive (SCANNING): detect markers, publish robot's map-frame pose once per
    station.  No movement.
  Active (APPROACHING, triggered by /aruco_dock/dock_to_a or dock_to_b):
    closed-loop PI control to DOCK_DIST from the target marker, then
    publish /aruco_dock/done = True and go IDLE.

States: IDLE -> SCANNING -> APPROACHING -> DONE -> IDLE
"""

import rclpy
from rclpy.node import Node
from enum import Enum, auto

import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import rclpy.time


# --------------- marker -> station mapping ---------------
MARKER_STATION_MAP = {42: 'A', 67: 'B'}

# --------------- geometry ---------------
MARKER_SIZE = 0.165
DOCK_DIST   = 0.30

MARKER_OBJECT_POINTS = np.array([
    [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
    [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
], dtype=np.float32)

# --------------- control gains ---------------
KP_ANGULAR            = 1.2
KP_LINEAR             = 0.5
KI_LINEAR             = 0.08
MAX_LINEAR            = 0.15
MAX_ANGULAR           = 0.8
MAX_BEARING_FOR_DRIVE = np.radians(15.0)
DONE_DIST_TOL         = 0.04
DONE_ANGLE_TOL        = np.radians(3.0)
INTEGRAL_CLAMP        = 0.10

# --------------- smoothing / lost marker ---------------
EMA_ALPHA           = 0.3
SCAN_CONFIRM_FRAMES = 3
LOST_HOLD           = 3
LOST_STOP           = 10
LOST_GIVE_UP        = 30

CAMERA_IMAGE_TOPIC = '/usb_cam/image_raw'
CAMERA_INFO_TOPIC  = '/usb_cam/camera_info'


class State(Enum):
    IDLE        = auto()
    SCANNING    = auto()
    APPROACHING = auto()
    DONE        = auto()


def _heading_from_rvec(rvec):
    R, _ = cv2.Rodrigues(rvec)
    return float(np.arctan2(R[0, 2], R[2, 2]))


class ArucoDockNode(Node):

    def __init__(self):
        super().__init__('aruco_dock')

        self._bridge        = CvBridge()
        self._camera_matrix = None
        self._dist_coeffs   = None

        # Instance-level to avoid OpenCV 4.6 C extension corruption
        self._aruco_dict      = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self._detector_params = aruco.DetectorParameters()

        self._state            = State.SCANNING
        self._active_target_id = None
        self._passive_published = set()

        # Smoothing / control state
        self._bearing_ema  = None
        self._distance_ema = None
        self._integral     = 0.0
        self._lost_count   = 0
        self._scan_confirm = 0

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._camera_info_cb, 10)
        self.create_subscription(Image,      CAMERA_IMAGE_TOPIC, self._image_cb,       10)

        self._debug_pub   = self.create_publisher(Image, '/aruco_debug/image_raw', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel',               10)
        self._done_pub    = self.create_publisher(Bool,  '/aruco_dock/done',        10)

        _latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._station_pose_pubs = {
            'A': self.create_publisher(PoseStamped, '/station_a_pose', _latched),
            'B': self.create_publisher(PoseStamped, '/station_b_pose', _latched),
        }

        self.create_service(Trigger, '/aruco_dock/dock_to_a', self._dock_to_a_cb)
        self.create_service(Trigger, '/aruco_dock/dock_to_b', self._dock_to_b_cb)
        self.create_service(Trigger, '/aruco_dock/scan',      self._scan_cb)

        self.get_logger().info(
            f'ArUco dock node started — scanning for markers {list(MARKER_STATION_MAP.keys())}')

    # ── services ─────────────────────────────────────────────────────────────

    def _dock_to_a_cb(self, request, response):
        return self._activate(42, response)

    def _dock_to_b_cb(self, request, response):
        return self._activate(67, response)

    def _scan_cb(self, request, response):
        self.get_logger().info('Scan requested — returning to SCANNING')
        self._state            = State.SCANNING
        self._active_target_id = None
        self._reset_control()
        response.success = True
        response.message = 'Scanning'
        return response

    def _activate(self, marker_id, response):
        station = MARKER_STATION_MAP[marker_id]
        self.get_logger().info(f'Docking activated for Station {station} (marker {marker_id})')
        self._active_target_id = marker_id
        self._state            = State.SCANNING
        self._reset_control()
        response.success = True
        response.message = f'Docking to Station {station}'
        return response

    def _reset_control(self):
        self._bearing_ema  = None
        self._distance_ema = None
        self._integral     = 0.0
        self._lost_count   = 0
        self._scan_confirm = 0

    # ── camera callbacks ─────────────────────────────────────────────────────

    def _camera_info_cb(self, msg):
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            d = np.array(msg.d, dtype=np.float64)
            self._dist_coeffs = d if d.size > 0 else np.zeros(5, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received.')

    def _image_cb(self, msg):
        if self._camera_matrix is None:
            return
        if self._state in (State.IDLE, State.DONE):
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return
        self._process_frame(frame, msg.header.stamp)

    # ── detection ────────────────────────────────────────────────────────────

    def _detect_marker(self, gray):
        gray = np.ascontiguousarray(gray)
        corners, ids, rejected = aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._detector_params)

        if ids is None:
            return None, None, None, corners, ids, rejected

        targets = ({self._active_target_id}
                   if self._active_target_id is not None
                   else set(MARKER_STATION_MAP.keys()))

        for i, mid in enumerate(ids.flatten()):
            if mid not in targets:
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
            return rvec, tvec, int(mid), corners, ids, rejected

        return None, None, None, corners, ids, rejected

    # ── main frame processing ────────────────────────────────────────────────

    def _process_frame(self, frame, stamp):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        debug = frame.copy()

        rvec, tvec, detected_id, corners, ids, rejected = self._detect_marker(gray)

        # Draw debug overlays
        if ids is not None:
            aruco.drawDetectedMarkers(debug, corners, ids)
        if rvec is not None:
            cv2.drawFrameAxes(debug, self._camera_matrix, self._dist_coeffs,
                              rvec, tvec, MARKER_SIZE * 0.5)
            t = tvec.flatten()
            dist = float(np.linalg.norm(t))
            station = MARKER_STATION_MAP.get(detected_id, '?')
            cv2.putText(debug, f'Stn {station} d={dist:.2f}m',
                        (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        mode_str = (f'Active-{MARKER_STATION_MAP.get(self._active_target_id, "?")}'
                    if self._active_target_id else 'Passive')
        cv2.putText(debug, f'{self._state.name} [{mode_str}]',
                    (5, debug.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # ── SCANNING ────────────────────────────────────────────────────
        if self._state == State.SCANNING:
            if rvec is not None:
                self._scan_confirm += 1
                if self._scan_confirm >= SCAN_CONFIRM_FRAMES:
                    self._publish_station_pose(detected_id)
                    if self._active_target_id is not None:
                        self.get_logger().info(
                            f'Marker {detected_id} confirmed — APPROACHING')
                        self._state = State.APPROACHING
                        t = tvec.flatten()
                        self._bearing_ema  = float(np.arctan2(t[0], t[2]))
                        self._distance_ema = float(np.linalg.norm(t))
                        self._integral     = 0.0
                        self._lost_count   = 0
            else:
                self._scan_confirm = 0

        # ── APPROACHING ─────────────────────────────────────────────────
        elif self._state == State.APPROACHING:
            if rvec is not None and detected_id == self._active_target_id:
                self._lost_count = 0
                t = tvec.flatten()
                bearing_raw  = float(np.arctan2(t[0], t[2]))
                distance_raw = float(np.linalg.norm(t))

                # EMA update
                if self._bearing_ema is None:
                    self._bearing_ema  = bearing_raw
                    self._distance_ema = distance_raw
                else:
                    self._bearing_ema  = EMA_ALPHA * bearing_raw  + (1 - EMA_ALPHA) * self._bearing_ema
                    self._distance_ema = EMA_ALPHA * distance_raw + (1 - EMA_ALPHA) * self._distance_ema

                dist_err = self._distance_ema - DOCK_DIST

                # Accumulate integral (distance error only)
                self._integral += dist_err * (1.0 / 30.0)  # ~30 Hz
                self._integral = np.clip(self._integral, -INTEGRAL_CLAMP / KI_LINEAR,
                                         INTEGRAL_CLAMP / KI_LINEAR)

                # Check done
                if abs(dist_err) < DONE_DIST_TOL and abs(self._bearing_ema) < DONE_ANGLE_TOL:
                    self._stop()
                    station = MARKER_STATION_MAP.get(self._active_target_id, '?')
                    self.get_logger().info(f'Docking DONE — Station {station} '
                                           f'(d={self._distance_ema:.3f}m)')
                    self._state = State.DONE
                    self._done_pub.publish(Bool(data=True))
                    self._publish_debug(debug, stamp)
                    return

                # PI control
                angular_z = float(np.clip(-KP_ANGULAR * self._bearing_ema,
                                          -MAX_ANGULAR, MAX_ANGULAR))

                if abs(self._bearing_ema) < MAX_BEARING_FOR_DRIVE and dist_err > 0:
                    linear_x = float(np.clip(
                        KP_LINEAR * dist_err + KI_LINEAR * self._integral,
                        0.0, MAX_LINEAR))
                else:
                    linear_x = 0.0

                self._send_cmd(linear_x, angular_z)

                cv2.putText(debug, f'err d={dist_err:+.3f}m b={np.degrees(self._bearing_ema):+.1f}deg',
                            (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            else:
                # Marker lost
                self._lost_count += 1
                if self._lost_count >= LOST_GIVE_UP:
                    self._stop()
                    self.get_logger().warn('Marker lost for 1s — back to SCANNING')
                    self._state = State.SCANNING
                    self._reset_control()
                elif self._lost_count >= LOST_STOP:
                    self._stop()
                # else: hold last cmd_vel for a few frames

        self._publish_debug(debug, stamp)

    # ── station pose publisher ───────────────────────────────────────────────

    def _publish_station_pose(self, marker_id):
        station = MARKER_STATION_MAP.get(marker_id)
        if station is None or station in self._passive_published:
            return
        try:
            tf_msg = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(f'TF unavailable, cannot publish Station {station} pose: {exc}')
            return

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = tf_msg.transform.translation.x
        pose.pose.position.y = tf_msg.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = tf_msg.transform.rotation

        self._station_pose_pubs[station].publish(pose)
        self._passive_published.add(station)
        self.get_logger().info(
            f'Published Station {station} pose: '
            f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')

    # ── helpers ──────────────────────────────────────────────────────────────

    def _send_cmd(self, linear_x=0.0, angular_z=0.0):
        cmd = Twist()
        cmd.linear.x  = linear_x
        cmd.angular.z = angular_z
        self._cmd_vel_pub.publish(cmd)

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
