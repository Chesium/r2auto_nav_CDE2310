#!/usr/bin/env python3
"""
station_b_aligner.py
====================
Station B: two-phase align + HSV blue-LED reactive fire.

This node owns BOTH alignment AND firing for Station B.
Mission controller FSM does NOT call /fire_launcher for Station B.
FSM only watches /receptacle/b_done (Bool True) to know B is finished.

CAMERA ORIENTATION:
  RPi camera mounted rotated 90° LEFT on the bot.
  Alignment uses Y-axis (cy) instead of X-axis (cx).
  offset_y > 0 (circle below frame centre) → bot drives BACKWARD
  offset_y < 0 (circle above frame centre) → bot drives FORWARD

PHASE 1 — ALIGNING:
  Linear P-control (fwd/bwd) until centred on wooden cutout hole.
  Must hold ALIGN_STABLE_FRAMES consecutive aligned frames → LOCKED → FIRING.

PHASE 2 — FIRING:
  Bot NEVER moves. Watches for blue LED inside the detected circle via HSV.
  Rising edge (LED appears) → calls /fire_launcher.
  Publishes /receptacle/b_done = True when all balls fired.

TOPICS PUBLISHED:
  /receptacle/offset     Int32  — Y-axis pixel offset
  /receptacle/tin_ready  Bool   — True when blue LED detected inside circle
  /receptacle/b_done     Bool   — True when all balls fired
  /receptacle/annotated  Image  — debug feed with alignment + LED overlay
  /cmd_vel               Twist  — linear.x only (fwd/bwd); angular.z always 0

SERVICE CALLED:
  /fire_launcher  Trigger  — called once per blue-LED rising edge
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


class Phase:
    ALIGNING = "ALIGNING"
    LOCKED   = "LOCKED"
    FIRING   = "FIRING"
    DONE     = "DONE"


class StationBAligner(Node):

    def __init__(self):
        super().__init__("station_b_aligner")

        # ── Phase 1: Linear P-control (Y-axis, camera rotated 90°) ────────
        self.KP_LINEAR           = 0.002
        self.MAX_LINEAR_VEL      = 0.08
        self.ALIGN_THRESHOLD     = 15      # pixels on Y-axis
        self.ALIGN_STABLE_FRAMES = 5       # consecutive aligned frames before locking
        self.align_stable_count  = 0

        # ── Hough params ──────────────────────────────────────────────────
        self.DP       = 1.2
        self.MIN_DIST = 100
        self.PARAM1   = 50
        self.PARAM2   = 25
        self.MIN_R    = 40
        self.MAX_R    = 60

        # ── Temporal filter (circle detection) ────────────────────────────
        self.CONFIRM_FRAMES   = 3
        self.consecutive_hits = 0
        self.confirmed        = False
        self.was_confirmed    = False

        # ── EMA smoother on cy (Y-axis) ──────────────────────────────────
        self.EMA_ALPHA = 0.50
        self.ema_cy    = None

        # ── Phase 2: HSV blue-LED detection ──────────────────────────────
        self.LED_H_LOW  = 100;  self.LED_H_HIGH = 130
        self.LED_S_LOW  = 100;  self.LED_S_HIGH = 255
        self.LED_V_LOW  = 100;  self.LED_V_HIGH = 255
        self.LED_PIXEL_RATIO_THRESH = 0.05   # fraction of circle area
        self.LED_CONFIRM_FRAMES     = 10
        self.led_consecutive_hits   = 0
        self.led_detected           = False
        self.prev_led_detected      = False

        # ── Firing ────────────────────────────────────────────────────────
        self.BALLS_TO_FIRE  = 1
        self.balls_fired    = 0
        self.launcher_busy  = False

        # ── Phase state ───────────────────────────────────────────────────
        self.phase         = Phase.ALIGNING
        self.locked_circle = None   # (cx, cy, r) saved at LOCKED transition

        # ── General ───────────────────────────────────────────────────────
        self.bridge      = CvBridge()
        self.frame_count = 0

        # ── QoS ───────────────────────────────────────────────────────────
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

        # ── Service client: fire launcher ─────────────────────────────────
        self.fire_client = self.create_client(Trigger, "/fire_launcher")

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_offset    = self.create_publisher(Int32, "/receptacle/offset",    10)
        self.pub_tin_ready = self.create_publisher(Bool,  "/receptacle/tin_ready", 10)
        self.pub_b_done    = self.create_publisher(Bool,  "/receptacle/b_done",    10)
        self.pub_annotated = self.create_publisher(Image, "/receptacle/annotated", 10)
        self.pub_cmd_vel   = self.create_publisher(Twist, "/cmd_vel",              10)

        self.get_logger().info("=" * 60)
        self.get_logger().info("STATION B ALIGNER — Y-axis align + HSV blue-LED fire")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Radius : {self.MIN_R}–{self.MAX_R}px")
        self.get_logger().info(f"Align  : {self.ALIGN_THRESHOLD}px Y-axis | "
                               f"stable={self.ALIGN_STABLE_FRAMES}f")
        self.get_logger().info(f"LED HSV: H[{self.LED_H_LOW}–{self.LED_H_HIGH}] "
                               f"S[{self.LED_S_LOW}–{self.LED_S_HIGH}] "
                               f"V[{self.LED_V_LOW}–{self.LED_V_HIGH}] "
                               f"ratio>{self.LED_PIXEL_RATIO_THRESH}")
        self.get_logger().info(f"Balls  : {self.BALLS_TO_FIRE}")
        self.get_logger().info("=" * 60)

    # ── Image callback ─────────────────────────────────────────────────────────
    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count == 1:
            self.get_logger().info("✓ First frame — Phase 1: ALIGNING")

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
        cx_raw, cy_raw, r_raw, annotated = self._detect_circle(frame)

        # ── Temporal filter ─────────────────────────────────────────────────
        if cy_raw is not None:
            self.consecutive_hits += 1
            if self.consecutive_hits >= self.CONFIRM_FRAMES:
                self.confirmed = True
        else:
            self.consecutive_hits = max(0, self.consecutive_hits - 1)
            if self.consecutive_hits == 0:
                self.confirmed = False
                self.ema_cy    = None
                if self.phase == Phase.ALIGNING:
                    self.align_stable_count = 0

        # ── EMA smoother on cy ──────────────────────────────────────────────
        if cy_raw is not None:
            self.ema_cy = cy_raw if self.ema_cy is None else (
                self.EMA_ALPHA * cy_raw + (1 - self.EMA_ALPHA) * self.ema_cy)

        # ── Default safe values ─────────────────────────────────────────────
        offset_msg    = Int32(); offset_msg.data    = 9999
        tin_ready_msg = Bool();  tin_ready_msg.data = False
        b_done_msg    = Bool();  b_done_msg.data    = (self.phase == Phase.DONE)
        cmd           = Twist()

        if self.confirmed and self.ema_cy is not None:
            cy_smooth  = int(round(self.ema_cy))
            offset_y   = cy_smooth - center_y
            is_aligned = abs(offset_y) < self.ALIGN_THRESHOLD
            offset_msg.data = offset_y

            # ── PHASE 1: ALIGNING ─────────────────────────────────────────
            if self.phase == Phase.ALIGNING:
                linear_vel = max(-self.MAX_LINEAR_VEL,
                                 min(self.MAX_LINEAR_VEL, self.KP_LINEAR * offset_y))
                cmd.linear.x  = 0.0 if is_aligned else linear_vel
                cmd.angular.z = 0.0

                self.align_stable_count = self.align_stable_count + 1 if is_aligned else 0

                if self.align_stable_count >= self.ALIGN_STABLE_FRAMES:
                    self.phase = Phase.LOCKED
                    cmd.linear.x = 0.0
                    self.locked_circle = (cx_raw, cy_raw, r_raw)
                    self.get_logger().info(
                        f"✓ LOCKED at off={offset_y:+d}px — bot frozen, watching for LED")

                # Annotate: horizontal lines for Y-axis alignment
                direction = "BWD" if offset_y > 0 else "FWD"
                cv2.line(annotated, (0, cy_smooth), (width, cy_smooth), (255, 255, 0), 1)
                cv2.line(annotated, (0, center_y),  (width, center_y),  (0, 200, 255), 1)
                sc = (0, 255, 0) if is_aligned else (0, 100, 255)
                cv2.putText(annotated,
                    f"[ALIGNING] off={offset_y:+d} "
                    f"{'ALIGNED' if is_aligned else direction} "
                    f"s={self.align_stable_count}/{self.ALIGN_STABLE_FRAMES}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, sc, 2)

                if cy_raw is not None and self.frame_count % 15 == 0:
                    self.get_logger().info(
                        f"[ALIGNING] off={offset_y:+d}px "
                        f"{'ALIGNED ✓' if is_aligned else direction} | "
                        f"lin={linear_vel:+.4f} | "
                        f"stable={self.align_stable_count}/{self.ALIGN_STABLE_FRAMES}")

            # ── PHASE 2: LOCKED / FIRING / DONE ───────────────────────────
            else:
                cmd.linear.x = 0.0; cmd.angular.z = 0.0   # frozen

                # LOCKED → FIRING on first frame
                if self.phase == Phase.LOCKED:
                    self.phase = Phase.FIRING
                    self.get_logger().info("Entering FIRING — watching for blue LED")

                if self.phase == Phase.FIRING:
                    # Update locked_circle from live detection when available
                    circle_for_roi = self.locked_circle
                    if cx_raw is not None and cy_raw is not None and r_raw is not None:
                        circle_for_roi = (cx_raw, cy_raw, r_raw)
                        self.locked_circle = circle_for_roi

                    # HSV blue-LED detection inside circle ROI
                    led_on = self._detect_blue_led(frame, circle_for_roi)
                    tin_ready_msg.data = led_on

                    # Temporal filter for LED
                    if led_on:
                        self.led_consecutive_hits = min(
                            self.led_consecutive_hits + 1, self.LED_CONFIRM_FRAMES + 2)
                    else:
                        self.led_consecutive_hits = max(0, self.led_consecutive_hits - 1)
                    self.led_detected = self.led_consecutive_hits >= self.LED_CONFIRM_FRAMES

                    # Rising edge → fire
                    rising_edge = self.led_detected and not self.prev_led_detected
                    if rising_edge and self.balls_fired < self.BALLS_TO_FIRE:
                        self.balls_fired += 1
                        self._call_fire_service()
                        self.get_logger().info(
                            f"✓ LED rising edge — FIRE ball "
                            f"{self.balls_fired}/{self.BALLS_TO_FIRE}")

                    self.prev_led_detected = self.led_detected

                    # Check completion
                    if self.balls_fired >= self.BALLS_TO_FIRE:
                        self.phase = Phase.DONE
                        self.get_logger().info("✓ All balls fired — DONE")

                b_done_msg.data = (self.phase == Phase.DONE)

                # Annotate Phase 2
                self._annotate_phase2(annotated, cy_smooth, center_y, width,
                                      offset_y, is_aligned)

                if self.frame_count % 10 == 0:
                    self.get_logger().info(
                        f"[{self.phase}] off={offset_y:+d}px | "
                        f"LED={'ON' if self.led_detected else 'off'} "
                        f"hits={self.led_consecutive_hits}/{self.LED_CONFIRM_FRAMES} | "
                        f"balls={self.balls_fired}/{self.BALLS_TO_FIRE}")

        else:
            cv2.putText(annotated, "NO CIRCLE DETECTED",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

            just_lost = self.was_confirmed and not self.confirmed
            if just_lost or self.frame_count % 30 == 0:
                self.get_logger().warn(
                    f"[B] NO CIRCLE DETECTED | "
                    f"hits={self.consecutive_hits}/{self.CONFIRM_FRAMES} "
                    f"phase={self.phase}")

        self.was_confirmed = self.confirmed

        # Publish every frame
        self.pub_cmd_vel.publish(cmd)
        self.pub_offset.publish(offset_msg)
        self.pub_tin_ready.publish(tin_ready_msg)
        self.pub_b_done.publish(b_done_msg)

        try:
            ann = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            ann.header = msg.header
            self.pub_annotated.publish(ann)
        except Exception:
            pass

    # ── HSV blue-LED detection inside circle ──────────────────────────────────
    def _detect_blue_led(self, frame, circle):
        """Check if blue LED is visible inside the Hough-detected circle ROI."""
        if circle is None:
            return False

        cx, cy, r = circle
        h, w = frame.shape[:2]

        # Circular mask
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(mask, (cx, cy), r, 255, -1)

        # HSV threshold for blue
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([self.LED_H_LOW, self.LED_S_LOW, self.LED_V_LOW])
        upper = np.array([self.LED_H_HIGH, self.LED_S_HIGH, self.LED_V_HIGH])
        blue_mask = cv2.inRange(hsv, lower, upper)

        # Blue pixels inside circle
        blue_in_circle = cv2.bitwise_and(blue_mask, mask)
        total_pixels = cv2.countNonZero(mask)
        if total_pixels == 0:
            return False
        blue_pixels = cv2.countNonZero(blue_in_circle)
        ratio = blue_pixels / total_pixels

        return ratio > self.LED_PIXEL_RATIO_THRESH

    # ── Fire launcher service (async, non-blocking) ───────────────────────────
    def _call_fire_service(self):
        if self.launcher_busy:
            self.get_logger().warn("[FIRE] Launcher busy — skipping")
            return
        if not self.fire_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn("[FIRE] /fire_launcher service not available")
            return
        self.launcher_busy = True
        future = self.fire_client.call_async(Trigger.Request())
        future.add_done_callback(self._fire_cb)
        self.get_logger().info(
            f"[FIRE] Service called — ball {self.balls_fired}/{self.BALLS_TO_FIRE}")

    def _fire_cb(self, future):
        self.launcher_busy = False
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"[FIRE] Launcher accepted: {resp.message}")
            else:
                self.get_logger().warn(f"[FIRE] Launcher rejected: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"[FIRE] Service error: {e}")

    # ── Circle detection ──────────────────────────────────────────────────────
    def _detect_circle(self, frame):
        """Hough circle detection. Returns (cx, cy, r, annotated).
        cy is None if no circle found."""
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

    # ── Phase 2 annotation ────────────────────────────────────────────────────
    def _annotate_phase2(self, frame, cy, mid_y, width, offset_y, aligned):
        colors = {Phase.FIRING: (0, 200, 200), Phase.DONE: (0, 255, 0)}
        c = colors.get(self.phase, (255, 255, 255))

        # Horizontal alignment lines
        cv2.line(frame, (0, cy),    (width, cy),    (255, 255, 0), 1)
        cv2.line(frame, (0, mid_y), (width, mid_y), (0, 200, 255), 1)

        cv2.putText(frame,
            f"[{self.phase}] off={offset_y:+d} "
            f"{'ALIGNED' if aligned else ('BWD' if offset_y > 0 else 'FWD')}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 2)

        cv2.putText(frame,
            f"LED={'ON' if self.led_detected else 'off'} "
            f"hits={self.led_consecutive_hits}/{self.LED_CONFIRM_FRAMES}",
            (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.50, c, 2)

        fire_text = ">>> FIRE! <<<" if (
            self.led_detected and not self.prev_led_detected) else ""
        cv2.putText(frame,
            f"balls={self.balls_fired}/{self.BALLS_TO_FIRE} {fire_text}",
            (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
            (0, 255, 0) if self.phase == Phase.DONE else (200, 200, 200), 2)


def main(args=None):
    rclpy.init(args=args)
    node = StationBAligner()
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
