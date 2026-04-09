#!/usr/bin/env python3
"""
station_b_aligner.py
====================
Station B: two-phase predictive firing node.

This node owns BOTH alignment AND firing for Station B.
Mission controller FSM does NOT call /fire_launcher for Station B.
FSM only watches /receptacle/b_done (Bool True) to know B is finished.

PHASE 1 — ALIGNING:
  Linear P-control (fwd/bwd) until centred on wooden cutout hole.
  Must hold ALIGN_STABLE_FRAMES consecutive aligned frames → LOCKED.

PHASE 2 — LOCKED → CALIBRATING → READY → DONE:
  Bot NEVER moves. Watches brightness transitions only.
  Measures rail period T from two rising edges.
  Calls /fire_launcher Trigger service at correct predicted times.
  Publishes /receptacle/b_done = True when all balls fired.

FIRE FORMULA:
  Ball N fires at: edge2_time + (N+1)*T - LAUNCHER_DELAY
  Ball exits launcher exactly when tin arrives at cutout for pass N+2.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32, Bool, Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class Phase:
    ALIGNING    = "ALIGNING"
    LOCKED      = "LOCKED"
    CALIBRATING = "CALIBRATING"
    READY       = "READY"
    DONE        = "DONE"


class StationBAligner(Node):

    def __init__(self):
        super().__init__("station_b_aligner")

        # ── Phase 1 alignment ─────────────────────────────────────────────
        self.KP_LINEAR           = 0.002
        self.MAX_LINEAR_VEL      = 0.08
        self.ALIGN_THRESHOLD     = 15
        self.ALIGN_STABLE_FRAMES = 20
        self.align_stable_count  = 0

        # ── Hough params ──────────────────────────────────────────────────
        self.DP = 1.2; self.MIN_DIST = 100
        self.PARAM1 = 50; self.PARAM2 = 25
        self.MIN_R = 40; self.MAX_R = 60

        # ── Temporal filter ───────────────────────────────────────────────
        self.CONFIRM_FRAMES = 3; self.consecutive_hits = 0; self.confirmed = False

        # ── EMA ───────────────────────────────────────────────────────────
        self.EMA_ALPHA = 0.50; self.ema_cx = None

        # ── Darkness detection (Phase 2 only) ─────────────────────────────
        # Calibrate: watch mean= in logs after LOCKED.
        #   Tin behind hole:  mean ~30-65  → dark → tin_ready
        #   Tin sliding past: mean ~120+   → bright → not ready
        self.DARK_THRESH             = 75
        self.DARKNESS_SAMPLE_RATIO   = 0.25
        self.DARKNESS_CONFIRM_FRAMES = 4
        self.darkness_hits           = 0

        # ── Launcher timing ───────────────────────────────────────────────
        self.LAUNCHER_DELAY = 2.2   # seconds: service call → ball exits
        self.BALLS_TO_FIRE  = 3
        self.launcher_busy  = False  # guard: one call at a time

        # ── Period FSM ────────────────────────────────────────────────────
        self.phase             = Phase.ALIGNING
        self.prev_tin_ready    = False
        self.edge1_time        = None
        self.period            = None
        self.fire_schedule     = []
        self.balls_fired       = 0
        self.last_pred_arrival = None

        # ── General ───────────────────────────────────────────────────────
        self.bridge = CvBridge(); self.frame_count = 0

        cam_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST, depth=10,
                              durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(CompressedImage, "/camera/image_raw/compressed",
                                 self.image_callback, cam_qos)

        # Service client to fire launcher (same service the launcher node provides)
        self.fire_client = self.create_client(Trigger, "/fire_launcher")

        self.pub_offset    = self.create_publisher(Int32,   "/receptacle/offset",    10)
        self.pub_aligned   = self.create_publisher(Bool,    "/receptacle/aligned",   10)
        self.pub_tin_ready = self.create_publisher(Bool,    "/receptacle/tin_ready", 10)
        self.pub_fire_now  = self.create_publisher(Bool,    "/receptacle/fire_now",  10)
        self.pub_period    = self.create_publisher(Float32, "/receptacle/period",    10)
        self.pub_b_done    = self.create_publisher(Bool,    "/receptacle/b_done",    10)
        self.pub_annotated = self.create_publisher(Image,   "/receptacle/annotated", 10)
        self.pub_cmd_vel   = self.create_publisher(Twist,   "/cmd_vel",              10)

        self.get_logger().info("=" * 60)
        self.get_logger().info("STATION B — Two-phase align + predictive fire")
        self.get_logger().info(f"LAUNCHER_DELAY={self.LAUNCHER_DELAY}s  BALLS={self.BALLS_TO_FIRE}")
        self.get_logger().info(f"DARK_THRESH={self.DARK_THRESH}  "
                               f"ALIGN_THRESHOLD={self.ALIGN_THRESHOLD}px")
        self.get_logger().info("=" * 60)

    # ── Image callback ────────────────────────────────────────────────────────
    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count == 1:
            self.get_logger().info("✓ First frame — Phase 1: ALIGNING")

        try:
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None: return
        except Exception as e:
            self.get_logger().error(f"Decode: {e}"); return

        height, width = frame.shape[:2]
        center_x = width // 2
        now = time.time()

        cx_raw, _, _, annotated, mean_inside = self._detect_circle(frame)

        # Temporal filter
        if cx_raw is not None:
            self.consecutive_hits += 1
            if self.consecutive_hits >= self.CONFIRM_FRAMES:
                self.confirmed = True
        else:
            self.consecutive_hits = max(0, self.consecutive_hits - 1)
            if self.consecutive_hits == 0:
                self.confirmed = False; self.ema_cx = None
                if self.phase == Phase.ALIGNING:
                    self.align_stable_count = 0

        # EMA
        if cx_raw is not None:
            self.ema_cx = cx_raw if self.ema_cx is None else (
                self.EMA_ALPHA * cx_raw + (1 - self.EMA_ALPHA) * self.ema_cx)

        # Message defaults
        offset_msg    = Int32(); offset_msg.data    = 9999
        aligned_msg   = Bool();  aligned_msg.data   = False
        tin_ready_msg = Bool();  tin_ready_msg.data = False
        fire_now_msg  = Bool();  fire_now_msg.data  = False
        b_done_msg    = Bool();  b_done_msg.data    = (self.phase == Phase.DONE)
        cmd = Twist()

        if self.confirmed and self.ema_cx is not None:
            cx_smooth  = int(round(self.ema_cx))
            offset_x   = cx_smooth - center_x
            is_aligned = abs(offset_x) < self.ALIGN_THRESHOLD
            offset_msg.data = offset_x; aligned_msg.data = is_aligned

            # ── PHASE 1: ALIGNING ─────────────────────────────────────────
            if self.phase == Phase.ALIGNING:
                self.align_stable_count = self.align_stable_count + 1 if is_aligned else 0
                linear_vel = max(-self.MAX_LINEAR_VEL,
                                 min(self.MAX_LINEAR_VEL, self.KP_LINEAR * offset_x))
                cmd.linear.x  = 0.0 if is_aligned else linear_vel
                cmd.angular.z = 0.0

                if self.align_stable_count >= self.ALIGN_STABLE_FRAMES:
                    self.phase = Phase.LOCKED; cmd.linear.x = 0.0
                    self.get_logger().info(
                        f"✓ LOCKED at off={offset_x:+d}px — bot frozen. Watching tin...")

                if self.frame_count % 15 == 0:
                    self.get_logger().info(
                        f"[ALIGNING] off={offset_x:+d} "
                        f"{'AL' if is_aligned else ('FWD' if offset_x > 0 else 'BWD')} "
                        f"stable={self.align_stable_count}/{self.ALIGN_STABLE_FRAMES}")

            # ── PHASE 2: LOCKED / CALIB / READY / DONE ────────────────────
            else:
                cmd.linear.x = 0.0; cmd.angular.z = 0.0  # FROZEN

                # Darkness: tin behind hole?
                is_dark = mean_inside is not None and mean_inside < self.DARK_THRESH
                self.darkness_hits = (
                    min(self.darkness_hits + 1, self.DARKNESS_CONFIRM_FRAMES + 2)
                    if is_dark else max(0, self.darkness_hits - 1))
                tin_ready = self.darkness_hits >= self.DARKNESS_CONFIRM_FRAMES
                tin_ready_msg.data = tin_ready

                # Period FSM — returns True when launcher should fire
                should_fire = self._period_fsm(tin_ready, now)
                if should_fire:
                    fire_now_msg.data = True
                    self._call_fire_service()

                b_done_msg.data = (self.phase == Phase.DONE)

                if self.frame_count % 10 == 0:
                    ms = f"{mean_inside:.1f}" if mean_inside is not None else "?"
                    Ts = f"T={self.period:.2f}s" if self.period else "T=?"
                    self.get_logger().info(
                        f"[{self.phase}] mean={ms} "
                        f"dark={'Y' if is_dark else 'n'} hits={self.darkness_hits} | "
                        f"tin={'Y' if tin_ready else 'n'} | "
                        f"{Ts} balls={self.balls_fired}/{self.BALLS_TO_FIRE}")

            self._annotate(annotated, cx_smooth, center_x, height,
                           offset_x, is_aligned, mean_inside,
                           fire_now_msg.data, tin_ready_msg.data)
        else:
            if self.frame_count % 30 == 0:
                self.get_logger().warn(
                    f"[NO DETECT] hits={self.consecutive_hits}/{self.CONFIRM_FRAMES} "
                    f"phase={self.phase}")

        # Publish
        self.pub_cmd_vel.publish(cmd)
        self.pub_offset.publish(offset_msg)
        self.pub_aligned.publish(aligned_msg)
        self.pub_tin_ready.publish(tin_ready_msg)
        self.pub_fire_now.publish(fire_now_msg)
        self.pub_b_done.publish(b_done_msg)
        if self.period is not None:
            pm = Float32(); pm.data = float(self.period)
            self.pub_period.publish(pm)
        try:
            ann = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            ann.header = msg.header; self.pub_annotated.publish(ann)
        except Exception:
            pass
        self.prev_tin_ready = tin_ready_msg.data

    # ── /fire_launcher service call (async, non-blocking) ────────────────────
    def _call_fire_service(self):
        """Send one async Trigger request to the launcher service."""
        if self.launcher_busy:
            self.get_logger().warn("[FIRE] Launcher busy — skipping this call")
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
        """Service response callback — just log and release the busy flag."""
        self.launcher_busy = False
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"[FIRE] Launcher accepted: {resp.message}")
            else:
                self.get_logger().warn(f"[FIRE] Launcher rejected: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"[FIRE] Service error: {e}")

    # ── Period FSM ────────────────────────────────────────────────────────────
    def _period_fsm(self, tin_ready: bool, now: float) -> bool:
        """Returns True on the frame when launcher should be triggered."""
        fire_cmd    = False
        rising_edge = tin_ready and not self.prev_tin_ready

        if self.phase == Phase.LOCKED:
            if rising_edge:
                self.edge1_time = now; self.phase = Phase.CALIBRATING
                self.get_logger().info(f"[PERIOD] Edge 1 at t={now:.3f}s")

        elif self.phase == Phase.CALIBRATING:
            if rising_edge:
                raw_T = now - self.edge1_time
                self.get_logger().info(f"[PERIOD] Edge 2 — raw T={raw_T:.3f}s")
                if not (0.5 < raw_T < 30.0):
                    self.get_logger().warn(f"[PERIOD] T={raw_T:.2f}s out of range — reset")
                    self.edge1_time = now; return False
                if raw_T < self.LAUNCHER_DELAY + 0.5:
                    self.get_logger().warn(
                        f"[PERIOD] T={raw_T:.2f}s close to LAUNCHER_DELAY — timing tight!")
                self.period = raw_T
                self.get_logger().info(f"[PERIOD] T={self.period:.3f}s locked")

                # Schedule: fire at edge2 + (N+1)*T - LAUNCHER_DELAY
                self.fire_schedule = []
                for i in range(self.BALLS_TO_FIRE):
                    ft = now + (i + 1) * self.period - self.LAUNCHER_DELAY
                    self.fire_schedule.append(ft)
                    self.get_logger().info(
                        f"[SCHEDULE] Ball {i+1}: fire t={ft:.3f}s "
                        f"→ arrives t={ft + self.LAUNCHER_DELAY:.3f}s")
                self.balls_fired = 0
                self.last_pred_arrival = now + self.period
                self.phase = Phase.READY

        elif self.phase == Phase.READY:
            if self.balls_fired >= self.BALLS_TO_FIRE:
                self.phase = Phase.DONE
                self.get_logger().info("[FIRE] All balls fired — DONE"); return False

            # Gentle T correction on each observed rising edge
            if rising_edge and self.last_pred_arrival is not None:
                err = now - self.last_pred_arrival
                if abs(err) < 1.0:
                    old_T = self.period
                    self.period = 0.9 * self.period + 0.1 * (self.period + err)
                    for i in range(self.balls_fired, self.BALLS_TO_FIRE):
                        passes = i - self.balls_fired + 1
                        self.fire_schedule[i] = now + passes * self.period - self.LAUNCHER_DELAY
                    self.last_pred_arrival = now + self.period
                    if abs(old_T - self.period) > 0.03:
                        self.get_logger().info(
                            f"[T-CORRECT] {old_T:.3f}→{self.period:.3f}s err={err:+.3f}s")
                else:
                    self.get_logger().warn(f"[T-CORRECT] err={err:+.3f}s too large — ignored")

            next_fire = self.fire_schedule[self.balls_fired]
            if next_fire < now - 0.5:
                self.get_logger().warn(
                    f"[SCHEDULE] Ball {self.balls_fired+1} overdue — firing now")
                next_fire = now
            if now >= next_fire:
                self.balls_fired += 1; fire_cmd = True
                self.get_logger().info(
                    f"[FIRE] Ball {self.balls_fired}/{self.BALLS_TO_FIRE} t={now:.3f}s")
            elif self.frame_count % 15 == 0:
                self.get_logger().info(
                    f"[READY] Ball {self.balls_fired+1} in {next_fire - now:.2f}s")

        return fire_cmd

    # ── Circle detection ──────────────────────────────────────────────────────
    def _detect_circle(self, frame):
        annotated = frame.copy()
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 2)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT,
            dp=self.DP, minDist=self.MIN_DIST, param1=self.PARAM1, param2=self.PARAM2,
            minRadius=self.MIN_R, maxRadius=self.MAX_R)
        if circles is None:
            return None, None, None, annotated, None
        circles = np.uint16(np.around(circles))
        cx, cy, r = [int(v) for v in max(circles[0], key=lambda x: x[2])]
        sr   = max(3, int(r * self.DARKNESS_SAMPLE_RATIO))
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.circle(mask, (cx, cy), sr, 255, -1)
        mean_inside = float(np.mean(gray[mask == 255]))
        color = (0, 255, 0) if mean_inside < self.DARK_THRESH else (0, 165, 255)
        cv2.circle(annotated, (cx, cy), r, color, 2)
        cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)
        cv2.circle(annotated, (cx, cy), sr, (255, 150, 0), 1)
        return cx, cy, r, annotated, mean_inside

    def _annotate(self, frame, cx, mid, h, offset_x, aligned, mean_inside, fire, tin):
        colors = {Phase.ALIGNING:(0,165,255), Phase.LOCKED:(255,255,0),
                  Phase.CALIBRATING:(255,200,0), Phase.READY:(0,200,200), Phase.DONE:(0,255,0)}
        c = (0,255,0) if fire else colors.get(self.phase, (255,255,255))
        cv2.line(frame, (cx,0), (cx,h), (255,255,0), 1)
        cv2.line(frame, (mid,0), (mid,h), (0,200,255), 1)
        cv2.putText(frame,
            f"[{self.phase}] off={offset_x:+d} "
            f"{'AL' if aligned else ('FWD' if offset_x>0 else 'BWD')}"
            + (f" s={self.align_stable_count}/{self.ALIGN_STABLE_FRAMES}"
               if self.phase == Phase.ALIGNING else ""),
            (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, c, 2)
        if self.phase != Phase.ALIGNING:
            ms = f"{mean_inside:.0f}" if mean_inside is not None else "?"
            Ts = f"T={self.period:.2f}s" if self.period else "T=?"
            cv2.putText(frame, f"mean={ms} tin={'Y' if tin else 'n'} {Ts}",
                (10,56), cv2.FONT_HERSHEY_SIMPLEX, 0.54, c, 2)
            cv2.putText(frame,
                f"balls={self.balls_fired}/{self.BALLS_TO_FIRE} "
                f"{'>>> FIRE! <<<' if fire else ''}",
                (10,82), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                (0,255,0) if fire else (200,200,200), 2)


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