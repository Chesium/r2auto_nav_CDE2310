#!/usr/bin/env python3
"""
station_a_aligner.py
====================
Station A: detect tin receptacle circle, align bot via linear P-control.
This node owns /cmd_vel during alignment.
Firing is handled entirely by the mission controller FSM via /fire_launcher service.
This node just publishes /receptacle/aligned and /receptacle/tin_ready.

TOPICS PUBLISHED:
  /receptacle/offset     Int32  — pixel offset from frame centre (FSM monitoring)
  /receptacle/aligned    Bool   — True when centred (FSM uses this to transition to FIRE_AT_A)
  /receptacle/tin_ready  Bool   — same as aligned for Station A (FSM fire gate)
  /receptacle/annotated  Image  — debug feed
  /cmd_vel               Twist  — linear.x only (fwd/bwd); angular.z always 0
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

        # ── Linear P-control ─────────────────────────────────────────────
        # Bot slides forward/backward to centre the tin in frame.
        # offset > 0 (tin right of centre) → forward; offset < 0 → backward.
        self.KP_LINEAR      = 0.002   # px → m/s gain
        self.MAX_LINEAR_VEL = 0.08    # m/s cap
        self.ALIGN_THRESHOLD    = 15  # pixels — within this = aligned
        self.ALIGN_STABLE_FRAMES = 20  # consecutive aligned frames before notifying FSM
        self.align_stable_count  = 0
        self.notified            = False  # fire service call only once per alignment

        # ── Hough params ─────────────────────────────────────────────────
        self.DP       = 1.2
        self.MIN_DIST = 100
        self.PARAM1   = 50
        self.PARAM2   = 25
        self.MIN_R    = 41
        self.MAX_R    = 61

        # ── Temporal filter: require N consecutive hits before trusting ──
        self.CONFIRM_FRAMES   = 4
        self.consecutive_hits = 0
        self.confirmed        = False

        # ── EMA smoother on detected cx ──────────────────────────────────
        self.EMA_ALPHA = 0.35
        self.ema_cx    = None

        # ── Darkness check: reject shiny acrylic false positives ─────────
        # Interior of real tin should be dark. Acrylic walls are bright.
        self.DARKNESS_CHECK       = True
        self.DARKNESS_THRESHOLD   = 100   # mean grey < this → valid circle
        self.DARKNESS_SAMPLE_RATIO = 0.6  # sample inner 60% of radius

        # ── General state ─────────────────────────────────────────────────
        self.bridge      = CvBridge()
        self.frame_count = 0

        # ── QoS (match RPi compressed publisher) ──────────────────────────
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed",
            self.image_callback, cam_qos)

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_offset    = self.create_publisher(Int32,  "/receptacle/offset",    10)
        self.pub_tin_ready = self.create_publisher(Bool,   "/receptacle/tin_ready", 10)
        self.pub_annotated = self.create_publisher(Image,  "/receptacle/annotated", 10)
        # cmd_vel: this node owns it during alignment (linear only, no angular)
        self.pub_cmd_vel   = self.create_publisher(Twist,  "/cmd_vel",              10)

        # Service client: notify FSM once when stably aligned
        self.notify_client = self.create_client(Trigger, "/receptacle/notify_aligned")

        self.get_logger().info("=" * 60)
        self.get_logger().info("STATION A ALIGNER — linear P-control, FSM fires")
        self.get_logger().info(f"Radius: {self.MIN_R}–{self.MAX_R}px  "
                               f"PARAM2={self.PARAM2}  "
                               f"align={self.ALIGN_THRESHOLD}px")
        self.get_logger().info("=" * 60)

    # ── Image callback ────────────────────────────────────────────────────────
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
        center_x = width // 2

        # Detect circle (darkness check filters out shiny acrylic)
        cx_raw, _, _, annotated = self._detect_circle(frame)

        # ── Temporal filter ────────────────────────────────────────────────
        if cx_raw is not None:
            self.consecutive_hits += 1
            if self.consecutive_hits >= self.CONFIRM_FRAMES:
                self.confirmed = True
        else:
            self.consecutive_hits = max(0, self.consecutive_hits - 1)
            if self.consecutive_hits == 0:
                self.confirmed = False
                self.ema_cx    = None

        # ── EMA smoother ───────────────────────────────────────────────────
        if cx_raw is not None:
            self.ema_cx = cx_raw if self.ema_cx is None else (
                self.EMA_ALPHA * cx_raw + (1 - self.EMA_ALPHA) * self.ema_cx)

        # ── Build messages ─────────────────────────────────────────────────
        offset_msg    = Int32(); offset_msg.data    = 9999
        aligned_msg   = Bool();  aligned_msg.data   = False
        tin_ready_msg = Bool();  tin_ready_msg.data = False
        cmd           = Twist()  # default: stop

        if self.confirmed and self.ema_cx is not None:
            cx_smooth  = int(round(self.ema_cx))
            offset_x   = cx_smooth - center_x
            is_aligned = abs(offset_x) < self.ALIGN_THRESHOLD

            # Linear P-control: fwd/bwd only, no rotation
            linear_vel = max(-self.MAX_LINEAR_VEL,
                             min(self.MAX_LINEAR_VEL, self.KP_LINEAR * offset_x))
            cmd.linear.x  = 0.0 if is_aligned else linear_vel
            cmd.angular.z = 0.0

            offset_msg.data    = offset_x
            tin_ready_msg.data = is_aligned

            # Stable frame debounce — call service once when threshold reached
            if is_aligned:
                self.align_stable_count += 1
            else:
                self.align_stable_count = 0

            if self.align_stable_count >= self.ALIGN_STABLE_FRAMES and not self.notified:
                self.notified = True
                self.get_logger().info(
                    f"✓ Stably aligned ({self.ALIGN_STABLE_FRAMES} frames) — notifying FSM")
                self._call_notify_service()

            # Annotate debug frame
            direction = "FWD" if offset_x > 0 else "BWD"
            cv2.line(annotated, (cx_smooth,0), (cx_smooth,height), (255,255,0), 1)
            cv2.line(annotated, (center_x,0),  (center_x,height),  (0,200,255), 1)
            sc = (0,255,0) if is_aligned else (0,100,255)
            cv2.putText(annotated,
                f"off={offset_x:+d} {'ALIGNED' if is_aligned else direction}",
                (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, sc, 2)

            if self.frame_count % 10 == 0:
                self.get_logger().info(
                    f"[A] off={offset_x:+d} {'ALIGNED' if is_aligned else direction} | "
                    f"lin={linear_vel:+.4f} hits={self.consecutive_hits}")
        else:
            if self.frame_count % 30 == 0:
                self.get_logger().warn(
                    f"[A] NO DETECT hits={self.consecutive_hits}/{self.CONFIRM_FRAMES}")

        # Publish — this node owns cmd_vel during alignment
        self.pub_cmd_vel.publish(cmd)
        self.pub_offset.publish(offset_msg)
        self.pub_tin_ready.publish(tin_ready_msg)

        try:
            ann = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            ann.header = msg.header
            self.pub_annotated.publish(ann)
        except Exception:
            pass

    # ── Notify FSM of stable alignment ───────────────────────────────────────
    def _call_notify_service(self):
        if not self.notify_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn("[ALIGN] /receptacle/notify_aligned not available — will retry")
            self.notified = False  # allow retry next frame
            return
        future = self.notify_client.call_async(Trigger.Request())
        future.add_done_callback(self._notify_cb)

    def _notify_cb(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info("[ALIGN] FSM notified successfully")
            else:
                self.get_logger().warn(f"[ALIGN] FSM rejected notify: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"[ALIGN] Notify service error: {e}")

    # ── Circle detection ──────────────────────────────────────────────────────
    def _detect_circle(self, frame):
        """
        Hough circle detection with optional darkness filter.
        Darkness filter: rejects circles whose interior is too bright
        (shiny acrylic walls, not the real tin opening).
        Returns (cx, cy, r, annotated).
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

        # Darkness filter
        if self.DARKNESS_CHECK:
            sr   = max(3, int(r * self.DARKNESS_SAMPLE_RATIO))
            mask = np.zeros(gray.shape, dtype=np.uint8)
            cv2.circle(mask, (cx, cy), sr, 255, -1)
            mean_inside = cv2.mean(gray, mask=mask)[0]
            if mean_inside > self.DARKNESS_THRESHOLD:
                if self.frame_count % 30 == 0:
                    self.get_logger().warn(
                        f"[FILTER] mean={mean_inside:.1f} rejected (too bright = acrylic)")
                return None, None, None, annotated

        cv2.circle(annotated, (cx, cy), r,  (0, 255, 0), 2)
        cv2.circle(annotated, (cx, cy), 4,  (0, 0, 255), -1)

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