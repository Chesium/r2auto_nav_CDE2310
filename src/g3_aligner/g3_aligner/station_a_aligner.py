#!/usr/bin/env python3
"""
station_a_aligner.py
====================
Station A: detect tin receptacle circle, align bot via linear P-control.
This node owns /cmd_vel during alignment.
Firing is handled entirely by the mission controller FSM via /fire_launcher service.

CAMERA ORIENTATION:
  RPi camera mounted rotated 90° LEFT on the bot.
  Alignment uses Y-axis (cy) instead of X-axis (cx).
  offset_y > 0 (circle below frame centre) → bot drives BACKWARD
  offset_y < 0 (circle above frame centre) → bot drives FORWARD
  Negate KP_LINEAR if bot moves the wrong way.

INTEGRATION FLOW:
  1. Node detects circle and runs Y-axis P-control via /cmd_vel.
  2. Once stably aligned for ALIGN_STABLE_FRAMES consecutive frames,
     calls /receptacle/notify_aligned (Trigger service) ONCE to notify FSM.
  3. FSM receives the notify → transitions to FIRE_AT_A → calls /fire_launcher.
  4. This node continues publishing /receptacle/tin_ready for FSM monitoring.

TOPICS PUBLISHED:
  /receptacle/offset     Int32  — Y-axis pixel offset (FSM monitoring)
  /receptacle/tin_ready  Bool   — True when aligned (FSM fire gate)
  /receptacle/annotated  Image  — debug feed with horizontal alignment lines
  /cmd_vel               Twist  — linear.x only (fwd/bwd); angular.z always 0

SERVICE CALLED (once on stable alignment):
  /receptacle/notify_aligned  Trigger  — tells FSM to transition to FIRE_AT_A
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np


class HoughDetectorStationA(Node):

    def __init__(self):
        super().__init__("hough_detector_station_a")

        # ── Linear P-control ──────────────────────────────────────────────
        # Y-axis: camera rotated 90°, so cy drives fwd/bwd.
        self.KP_LINEAR           = 0.002
        self.MAX_LINEAR_VEL      = 0.08
        self.ALIGN_THRESHOLD     = 15     # pixels on Y-axis
        self.ALIGN_STABLE_FRAMES = 5      # consecutive aligned frames before notifying FSM
        self.align_stable_count  = 0
        self.notified            = False  # service called only once per alignment

        # ── Hough params — tuned for 22–38 cm distance, camera rotated 90° ─
        self.DP       = 1.2
        self.MIN_DIST = 100
        self.PARAM1   = 50
        self.PARAM2   = 35
        self.MIN_R    = 55   
        self.MAX_R    = 75

        # ── Temporal filter ────────────────────────────────────────────────
        self.CONFIRM_FRAMES   = 6
        self.consecutive_hits = 0
        self.confirmed        = False
        self.was_confirmed    = False   # for instant NO DETECT log on transition

        # ── EMA smoother on cy (Y-axis) ────────────────────────────────────
        self.EMA_ALPHA = 0.35
        self.ema_cy    = None

        # ── Darkness check disabled ────────────────────────────────────────
        # Not needed for Station A: static receptacle in controlled position.
        # Darkness check was causing false rejections with RPi cam lighting.
        self.DARKNESS_CHECK = False

        # ── General ────────────────────────────────────────────────────────
        self.bridge      = CvBridge()
        self.frame_count = 0

        # ── QoS: depth=2 prevents frame queue pile-up on RPi ──────────────
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscriber ────────────────────────────────────────────────────
        self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed",
            self.image_callback, cam_qos)

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_offset    = self.create_publisher(Int32,  "/receptacle/offset",    10)
        self.pub_tin_ready = self.create_publisher(Bool,   "/receptacle/tin_ready", 10)
        self.pub_annotated = self.create_publisher(Image,  "/receptacle/annotated", 10)
        self.pub_cmd_vel   = self.create_publisher(Twist,  "/cmd_vel",              10)

        # ── Service client: notify FSM once when stably aligned ───────────
        self.notify_client = self.create_client(Trigger, "/receptacle/notify_aligned")

        self.get_logger().info("=" * 60)
        self.get_logger().info("STATION A ALIGNER — Y-axis, FSM fires via service")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Radius : {self.MIN_R}–{self.MAX_R}px (tuned 22-38cm)")
        self.get_logger().info(f"PARAM2 : {self.PARAM2}")
        self.get_logger().info(f"Align  : {self.ALIGN_THRESHOLD}px Y-axis | "
                               f"stable={self.ALIGN_STABLE_FRAMES}f")
        self.get_logger().info("=" * 60)

    # ── Image callback ─────────────────────────────────────────────────────────
    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count == 1:
            self.get_logger().info("✓ First frame received")

        try:
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                return
        except Exception as e:
            self.get_logger().error(f"Decode: {e}")
            return

        height, width = frame.shape[:2]
        center_y = height // 2   # Y-axis: camera rotated 90°

        # Detect circle — cy_raw is None if nothing found this frame
        cx_raw, cy_raw, r, annotated = self._detect_circle(frame)

        # ── Temporal filter ─────────────────────────────────────────────────
        if cy_raw is not None:
            self.consecutive_hits += 1
            if self.consecutive_hits >= self.CONFIRM_FRAMES:
                self.confirmed = True
        else:
            self.consecutive_hits = max(0, self.consecutive_hits - 1)
            if self.consecutive_hits == 0:
                self.confirmed = False
                self.ema_cy    = None   # clear EMA — no stale position reuse

        # ── EMA smoother on cy ──────────────────────────────────────────────
        if cy_raw is not None:
            self.ema_cy = cy_raw if self.ema_cy is None else (
                self.EMA_ALPHA * cy_raw + (1 - self.EMA_ALPHA) * self.ema_cy)

        # ── Default safe values ─────────────────────────────────────────────
        offset_msg    = Int32(); offset_msg.data    = 9999
        tin_ready_msg = Bool();  tin_ready_msg.data = False
        cmd           = Twist()   # zero velocity

        if self.confirmed and self.ema_cy is not None:
            cy_smooth  = int(round(self.ema_cy))
            offset_y   = cy_smooth - center_y
            is_aligned = abs(offset_y) < self.ALIGN_THRESHOLD

            # P-control on Y-axis (camera rotated 90°):
            # offset_y > 0 → circle below centre → drive BACKWARD
            # offset_y < 0 → circle above centre → drive FORWARD
            linear_vel = max(-self.MAX_LINEAR_VEL,
                             min(self.MAX_LINEAR_VEL, self.KP_LINEAR * offset_y))
            cmd.linear.x  = 0.0 if is_aligned else linear_vel
            cmd.angular.z = 0.0

            offset_msg.data    = offset_y
            tin_ready_msg.data = is_aligned

            # Stable frame debounce — call service ONCE when threshold reached
            if is_aligned:
                self.align_stable_count += 1
            else:
                self.align_stable_count = 0
                self.notified = False   # reset so re-alignment can notify again

            if self.align_stable_count >= self.ALIGN_STABLE_FRAMES and not self.notified:
                self.notified = True
                self.get_logger().info(
                    f"✓ Stably aligned ({self.ALIGN_STABLE_FRAMES}f) — notifying FSM")
                self._call_notify_service()

            # Annotate: horizontal lines for Y-axis alignment
            direction = "BWD" if offset_y > 0 else "FWD"
            cv2.line(annotated, (0, cy_smooth), (width, cy_smooth), (255, 255, 0), 1)
            cv2.line(annotated, (0, center_y),  (width, center_y),  (0, 200, 255), 1)
            sc = (0, 255, 0) if is_aligned else (0, 100, 255)
            cv2.putText(annotated,
                f"off={offset_y:+d}  {'ALIGNED' if is_aligned else direction}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, sc, 2)
            cv2.putText(annotated,
                f"hits={self.consecutive_hits}  stable={self.align_stable_count}/{self.ALIGN_STABLE_FRAMES}",
                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, sc, 1)

            # Log only on fresh detection frames
            if cy_raw is not None and self.frame_count % 10 == 0:
                self.get_logger().info(
                    f"[A] off={offset_y:+d}px "
                    f"{'ALIGNED ✓' if is_aligned else direction} | "
                    f"lin={linear_vel:+.4f} | "
                    f"stable={self.align_stable_count}/{self.ALIGN_STABLE_FRAMES} | "
                    f"hits={self.consecutive_hits}")

        else:
            cv2.putText(annotated, "NO RECEPTACLE DETECTED",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

            # Log immediately on first lost frame, then throttle
            just_lost = self.was_confirmed and not self.confirmed
            if just_lost or self.frame_count % 30 == 0:
                self.get_logger().warn(
                    f"[A] NO RECEPTACLE DETECTED | "
                    f"hits={self.consecutive_hits}/{self.CONFIRM_FRAMES}")

        self.was_confirmed = self.confirmed

        # Publish every frame
        self.pub_cmd_vel.publish(cmd)
        self.pub_offset.publish(offset_msg)
        self.pub_tin_ready.publish(tin_ready_msg)

        try:
            ann = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            ann.header = msg.header
            self.pub_annotated.publish(ann)
        except Exception:
            pass

    # ── Notify FSM of stable alignment (called once) ──────────────────────────
    def _call_notify_service(self):
        if not self.notify_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn(
                "[ALIGN] /receptacle/notify_aligned not available — will retry next frame")
            self.notified = False   # allow retry
            return
        future = self.notify_client.call_async(Trigger.Request())
        future.add_done_callback(self._notify_cb)

    def _notify_cb(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info("[ALIGN] FSM notified ✓")
            else:
                self.get_logger().warn(f"[ALIGN] FSM rejected: {resp.message} — will retry")
                self.notified = False   # allow retry once FSM reaches ALIGN_AT_A
        except Exception as e:
            self.get_logger().error(f"[ALIGN] Service error: {e}")
            self.notified = False   # allow retry on error

    # ── Circle detection ───────────────────────────────────────────────────────
    def _detect_circle(self, frame):
        """
        Hough circle detection. Darkness check disabled (not needed for Station A).
        Returns (cx, cy, r, annotated). cy is None if no circle found.
        Uses full frame — no resize (RPi handles 320x240 fine at tuned params).
        """
        annotated = frame.copy()
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 2)

        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT,
            dp=self.DP, minDist=self.MIN_DIST,
            param1=self.PARAM1, param2=self.PARAM2,
            minRadius=self.MIN_R, maxRadius=self.MAX_R)

        if circles is None:
            return None, None, None, annotated

        circles = np.uint16(np.around(circles))
        cx, cy, r = [int(v) for v in max(circles[0], key=lambda x: x[2])]

        cv2.circle(annotated, (cx, cy), r, (0, 255, 0), 2)
        cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)

        return cx, cy, r, annotated


def main(args=None):
    rclpy.init(args=args)
    node = HoughDetectorStationA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()