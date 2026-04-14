#!/usr/bin/env python3
"""
Warehouse Mission Controller — Merged FSM
==========================================
Merged from mission_controller.py (v1: exploration) and
mission_controller_2.py (v2: ArUco docking + aligner service + autonomous B).

Uses simple_aruco_dock for visual-servo approach (replaces Nav2 navigation).

FSM:
  INIT → EXPLORE → DOCK_AT_A → ALIGN_AT_A → FIRE_AT_A ─┐
                 → DOCK_AT_B → ALIGN_AT_B → FIRE_AT_B ─┼→ COMPLETE
                                                        └→ FAILED

Topic/Service Contracts:
  From station_a_aligner:
    /receptacle/offset          Int32   — logged only
    /receptacle/notify_aligned  Trigger — service called once when stably aligned
  From station_b_aligner:
    /receptacle/b_done          Bool    — all balls fired at Station B
  From simple_aruco_dock:
    /station_a_pose             PoseStamped — Station A detected
    /station_b_pose             PoseStamped — Station B detected
    /aruco_dock/done            Bool    — docking approach complete
  From ball_launcher_node:
    /launcher_status            String  — "idle"|"firing"|"complete"|"error"
  From exploration:
    /exploration_complete       Bool    — map closure signal

  Services called:
    /fire_launcher              Trigger — fire one ball (Station A only)
    /aruco_dock/dock_to_a       Trigger — activate docking to Station A
    /aruco_dock/dock_to_b       Trigger — activate docking to Station B
    /aruco_dock/scan            Trigger — resume ArUco scanning
    /exploration/set_enabled    SetBool — enable/disable frontier exploration

  Service served:
    /receptacle/notify_aligned  Trigger — station_a_aligner calls once when aligned
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String, Int32
from std_srvs.srv import Trigger, SetBool
import time


class MissionState:
    INIT          = "INIT"
    EXPLORE       = "EXPLORE"
    DOCK_AT_A     = "DOCK_AT_A"
    ALIGN_AT_A    = "ALIGN_AT_A"
    FIRE_AT_A     = "FIRE_AT_A"
    DOCK_AT_B     = "DOCK_AT_B"
    ALIGN_AT_B    = "ALIGN_AT_B"
    FIRE_AT_B     = "FIRE_AT_B"
    COMPLETE      = "COMPLETE"
    FAILED        = "FAILED"


class WarehouseMissionController(Node):

    def __init__(self):
        super().__init__("mission_controller")

        # ── FSM state ───────────────────────────────────────────────────
        self.declare_parameter("initial_state", "INIT")
        initial_state_str = self.get_parameter("initial_state").value
        try:
            self.state = getattr(MissionState, initial_state_str)
        except AttributeError:
            self.get_logger().warn(
                f"Unknown initial_state '{initial_state_str}', defaulting to INIT"
            )
            self.state = MissionState.INIT
        self.previous_state = None
        self._state_entry_time = time.time()
        self._mission_start_time = time.time()

        # ── Station tracking ────────────────────────────────────────────
        self.stations = {
            "A": {"pose": None, "found": False, "delivered": False, "retry_count": 0},
            "B": {"pose": None, "found": False, "delivered": False, "retry_count": 0},
        }
        self.max_retries       = 3
        self.exploration_complete = False

        # ── Alignment state ─────────────────────────────────────────────
        self.receptacle_not_detected = 9999
        self.receptacle_offset       = self.receptacle_not_detected
        self.alignment_timeout       = 15.0
        self.alignment_start_time    = None
        self.notify_aligned_received = False

        # ── Station B completion flag ───────────────────────────────────
        self.station_b_done = False

        # ── ArUco dock state ───────────────────────────────────────────
        self.aruco_dock_done    = False
        self._dock_started      = False
        self._dock_start_time   = None

        # ── Launcher state (Station A only) ─────────────────────────────
        self.launcher_ready           = True
        self.launcher_status          = "idle"
        self.launcher_request_pending = False
        self.balls_fired              = 0
        self.balls_per_station        = 3
        self.current_station_firing   = None

        # Station A timing delays
        self.declare_parameter("station_a_delay_after_ball_1", 4.7)
        self.declare_parameter("station_a_delay_after_ball_2", 0.7)
        self.declare_parameter("station_a_delay_after_ball_3", 0.0)
        self.station_a_delays = {
            0: float(self.get_parameter("station_a_delay_after_ball_1").value),
            1: float(self.get_parameter("station_a_delay_after_ball_2").value),
            2: float(self.get_parameter("station_a_delay_after_ball_3").value),
        }
        self.waiting_after_fire   = False
        self.fire_wait_start_time = None

        # ── Subscribers ─────────────────────────────────────────────────
        self.create_subscription(PoseStamped, "/station_a_pose", self.station_a_callback, 10)
        self.create_subscription(PoseStamped, "/station_b_pose", self.station_b_callback, 10)
        self.create_subscription(Int32, "/receptacle/offset", self.offset_callback, 10)
        self.create_service(Trigger, "/receptacle/notify_aligned", self.notify_aligned_handler)
        self.create_subscription(Bool, "/receptacle/b_done", self.b_done_callback, 10)
        self.create_subscription(String, "/launcher_status", self.launcher_callback, 10)
        self.create_subscription(Bool, "/aruco_dock/done", self.aruco_dock_done_callback, 10)
        self.create_subscription(Bool, "/exploration_complete", self.exploration_callback, 10)

        # ── Publishers ──────────────────────────────────────────────────
        self.state_pub   = self.create_publisher(String, "/mission_state", 10)
        self.cmd_vel_pub = self.create_publisher(Twist,  "/cmd_vel",       10)

        # ── Service clients ─────────────────────────────────────────────
        self.fire_launcher_client = self.create_client(Trigger, "/fire_launcher")
        self.dock_to_a_client     = self.create_client(Trigger, "/aruco_dock/dock_to_a")
        self.dock_to_b_client     = self.create_client(Trigger, "/aruco_dock/dock_to_b")
        self.dock_scan_client     = self.create_client(Trigger, "/aruco_dock/scan")
        self._exploration_client  = self.create_client(SetBool, "/exploration/set_enabled")

        # ── FSM timer: 10 Hz ────────────────────────────────────────────
        self.create_timer(0.1, self.state_machine_tick)
        # ── Periodic status dump: every 5 seconds ───────────────────────
        self.create_timer(5.0, self._periodic_status)

        self.get_logger().info("=" * 60)
        self.get_logger().info("MISSION CONTROLLER INITIALISED")
        self.get_logger().info("  Docking: simple_aruco_dock visual servo")
        self.get_logger().info("  Station A: FSM fires via /fire_launcher")
        self.get_logger().info("  Station B: station_b_aligner fires autonomously")
        self.get_logger().info(f"  Delays A: ball1={self.station_a_delays[0]}s "
                               f"ball2={self.station_a_delays[1]}s "
                               f"ball3={self.station_a_delays[2]}s")
        self.get_logger().info(f"  Max retries: {self.max_retries}")
        self.get_logger().info(f"  Alignment timeout: {self.alignment_timeout}s")
        self.get_logger().info(f"  Initial state: {self.state}")
        self.get_logger().info("=" * 60)

    # =========================================================================
    # PERIODIC STATUS
    # =========================================================================

    def _periodic_status(self):
        """Dump full FSM state every 5 seconds for debugging."""
        elapsed_state = time.time() - self._state_entry_time
        elapsed_mission = time.time() - self._mission_start_time
        a = self.stations["A"]
        b = self.stations["B"]
        self.get_logger().info(
            f"[STATUS] state={self.state} ({elapsed_state:.0f}s in state, "
            f"{elapsed_mission:.0f}s total) | "
            f"A: found={a['found']} delivered={a['delivered']} retries={a['retry_count']} | "
            f"B: found={b['found']} delivered={b['delivered']} retries={b['retry_count']} | "
            f"dock: started={self._dock_started} done={self.aruco_dock_done} | "
            f"align: offset={self.receptacle_offset} notify={self.notify_aligned_received} | "
            f"launcher: status={self.launcher_status} ready={self.launcher_ready} "
            f"balls={self.balls_fired}/{self.balls_per_station} pending={self.launcher_request_pending} | "
            f"b_done={self.station_b_done} explore_done={self.exploration_complete}"
        )

    # =========================================================================
    # CALLBACKS
    # =========================================================================

    def station_a_callback(self, msg):
        if not self.stations["A"]["found"]:
            self.stations["A"]["found"] = True
            p = msg.pose.position
            self.get_logger().info(
                f"[CB] Station A FIRST DETECTED | dist={p.z:.2f}m "
                f"| current_state={self.state}"
            )
        self.stations["A"]["pose"] = msg

    def station_b_callback(self, msg):
        if not self.stations["B"]["found"]:
            self.stations["B"]["found"] = True
            p = msg.pose.position
            self.get_logger().info(
                f"[CB] Station B FIRST DETECTED | dist={p.z:.2f}m "
                f"| current_state={self.state}"
            )
        self.stations["B"]["pose"] = msg

    def offset_callback(self, msg):
        old = self.receptacle_offset
        self.receptacle_offset = msg.data
        # Log significant changes or first detection
        if old == self.receptacle_not_detected and msg.data != self.receptacle_not_detected:
            self.get_logger().info(
                f"[CB] Receptacle FIRST DETECTED | offset={msg.data}px | state={self.state}")
        elif msg.data == self.receptacle_not_detected and old != self.receptacle_not_detected:
            self.get_logger().warn(
                f"[CB] Receptacle LOST | was offset={old}px | state={self.state}")

    def notify_aligned_handler(self, _request, response):
        self.get_logger().info(
            f"[SRV] /receptacle/notify_aligned called | current_state={self.state}")
        if self.state == MissionState.ALIGN_AT_A:
            self.notify_aligned_received = True
            elapsed = time.time() - (self.alignment_start_time or time.time())
            self.get_logger().info(
                f"[SRV] Alignment ACCEPTED after {elapsed:.1f}s — will transition to FIRE_AT_A")
            response.success = True
            response.message = "Alignment accepted"
        else:
            self.get_logger().warn(
                f"[SRV] Alignment REJECTED — wrong state {self.state} "
                f"(expected ALIGN_AT_A)")
            response.success = False
            response.message = f"Wrong state: {self.state}"
        return response

    def aruco_dock_done_callback(self, msg):
        """Only accept when FSM is actually in a DOCK state."""
        if self.state not in [MissionState.DOCK_AT_A, MissionState.DOCK_AT_B]:
            self.get_logger().debug(
                f"[CB] /aruco_dock/done received (data={msg.data}) but "
                f"state={self.state} — IGNORING stale message")
            return
        if msg.data:
            if not self.aruco_dock_done:
                elapsed = time.time() - (self._dock_start_time or time.time())
                self.aruco_dock_done = True
                self.get_logger().info(
                    f"[CB] ArUco docking SUCCEEDED after {elapsed:.1f}s | state={self.state}")
            else:
                self.get_logger().debug(
                    "[CB] /aruco_dock/done=True received but already flagged")
        else:
            self.get_logger().warn(
                f"[CB] /aruco_dock/done=False received | state={self.state} — dock FAILED")
            self.aruco_dock_done = True  # let handle_dock pick up the failure

    def b_done_callback(self, msg):
        self.get_logger().info(
            f"[CB] /receptacle/b_done received data={msg.data} | "
            f"current b_done={self.station_b_done} | state={self.state}")
        if msg.data and not self.station_b_done:
            self.station_b_done = True
            self.get_logger().info(
                "[CB] Station B: ALL BALLS FIRED — b_done flag set")

    def launcher_callback(self, msg):
        old_status = self.launcher_status
        self.launcher_status = msg.data
        if old_status != msg.data:
            self.get_logger().info(
                f"[CB] Launcher status: {old_status} -> {msg.data} | "
                f"balls_fired={self.balls_fired} ready={self.launcher_ready} "
                f"pending={self.launcher_request_pending}")

        if self.launcher_status == "complete":
            self.launcher_ready = True
            self.balls_fired   += 1
            self.get_logger().info(
                f"[CB] Ball {self.balls_fired}/{self.balls_per_station} CONFIRMED FIRED | "
                f"station={self.current_station_firing}")

            if self.current_station_firing == "A":
                delay = self.get_delay_for_ball(self.balls_fired - 1)
                if delay > 0:
                    self.waiting_after_fire   = True
                    self.fire_wait_start_time = time.time()
                    self.get_logger().info(
                        f"[CB] Starting {delay}s delay before next ball "
                        f"(ball {self.balls_fired} just fired)")
                else:
                    self.get_logger().info(
                        f"[CB] No delay after ball {self.balls_fired} — ready immediately")
        elif self.launcher_status == "error":
            self.launcher_ready = True
            self.get_logger().error(
                f"[CB] Launcher ERROR — ready={self.launcher_ready} "
                f"balls_fired={self.balls_fired} station={self.current_station_firing}")
        elif self.launcher_status == "idle" and not self.launcher_request_pending:
            self.launcher_ready = True

    def exploration_callback(self, msg):
        self.exploration_complete = msg.data
        if msg.data:
            self.get_logger().info(
                f"[CB] Exploration COMPLETE | state={self.state} | "
                f"A_found={self.stations['A']['found']} B_found={self.stations['B']['found']}")

    # =========================================================================
    # FSM TICK (10 Hz)
    # =========================================================================

    def state_machine_tick(self):
        s = String(); s.data = self.state
        self.state_pub.publish(s)

        if   self.state == MissionState.INIT:          self.handle_init()
        elif self.state == MissionState.EXPLORE:        self.handle_explore()
        elif self.state == MissionState.DOCK_AT_A:      self.handle_dock("A")
        elif self.state == MissionState.ALIGN_AT_A:     self.handle_alignment()
        elif self.state == MissionState.FIRE_AT_A:      self.handle_firing_a()
        elif self.state == MissionState.DOCK_AT_B:      self.handle_dock("B")
        elif self.state == MissionState.ALIGN_AT_B:     self.handle_alignment_b()
        elif self.state == MissionState.FIRE_AT_B:      self.handle_firing_b()
        elif self.state == MissionState.COMPLETE:       self.handle_complete()

    # =========================================================================
    # STATE HANDLERS
    # =========================================================================

    def handle_init(self):
        """Wait for dock services to be available."""
        dock_a = self.dock_to_a_client.service_is_ready()
        dock_b = self.dock_to_b_client.service_is_ready()
        scan   = self.dock_scan_client.service_is_ready()
        explore = self._exploration_client.service_is_ready()
        launcher = self.fire_launcher_client.service_is_ready()

        if dock_a:
            self.get_logger().info(
                f"[INIT] Services ready: dock_a={dock_a} dock_b={dock_b} "
                f"scan={scan} explore={explore} launcher={launcher} — starting exploration")
            self._set_exploration(True)
            self.transition_to(MissionState.EXPLORE)
        else:
            self.get_logger().warn(
                f"[INIT] Waiting for services: dock_a={dock_a} dock_b={dock_b} "
                f"scan={scan} explore={explore} launcher={launcher}",
                throttle_duration_sec=2.0,
            )

    def handle_explore(self):
        elapsed = time.time() - self._state_entry_time
        # Periodic exploration status
        if int(elapsed * 10) % 100 == 0 and elapsed > 0:  # every 10s
            self.get_logger().info(
                f"[EXPLORE] {elapsed:.0f}s | A_found={self.stations['A']['found']} "
                f"B_found={self.stations['B']['found']} | "
                f"A_delivered={self.stations['A']['delivered']} "
                f"B_delivered={self.stations['B']['delivered']} | "
                f"explore_complete={self.exploration_complete}")

        if self.stations["A"]["found"] and not self.stations["A"]["delivered"]:
            self.get_logger().info(
                f"[EXPLORE] Station A found after {elapsed:.1f}s exploring — "
                f"disabling exploration, transitioning to DOCK_AT_A")
            self._set_exploration(False)
            self.transition_to(MissionState.DOCK_AT_A)
            return
        if self.stations["B"]["found"] and not self.stations["B"]["delivered"]:
            self.get_logger().info(
                f"[EXPLORE] Station B found after {elapsed:.1f}s exploring — "
                f"disabling exploration, transitioning to DOCK_AT_B")
            self._set_exploration(False)
            self.transition_to(MissionState.DOCK_AT_B)
            return
        if self.stations["A"]["delivered"] and self.stations["B"]["delivered"]:
            self.get_logger().info(
                f"[EXPLORE] Both stations delivered — mission complete after {elapsed:.1f}s")
            self._set_exploration(False)
            self.transition_to(MissionState.COMPLETE)

    def handle_dock(self, sid):
        """Trigger dock service on entry, then wait for /aruco_dock/done."""
        if not self._dock_started:
            self.aruco_dock_done = False
            self._dock_start_time = time.time()
            client = self.dock_to_a_client if sid == "A" else self.dock_to_b_client
            if client.service_is_ready():
                self.get_logger().info(
                    f"[DOCK_{sid}] Calling /aruco_dock/dock_to_{sid.lower()} — "
                    f"visual servo approach starting")
                future = client.call_async(Trigger.Request())
                future.add_done_callback(
                    lambda f, s=sid: self._dock_service_response(f, s)
                )
                self._dock_started = True
            else:
                self.get_logger().warn(
                    f"[DOCK_{sid}] Service /aruco_dock/dock_to_{sid.lower()} "
                    f"NOT READY — will retry next tick",
                    throttle_duration_sec=2.0,
                )
            return

        # Periodic waiting log
        if self._dock_start_time:
            elapsed = time.time() - self._dock_start_time
            if int(elapsed * 10) % 100 == 0 and elapsed > 0:  # every 10s
                self.get_logger().info(
                    f"[DOCK_{sid}] Waiting for visual servo... {elapsed:.0f}s elapsed | "
                    f"dock_done={self.aruco_dock_done}")

        if self.aruco_dock_done:
            elapsed = time.time() - (self._dock_start_time or time.time())
            self.aruco_dock_done = False
            self._dock_started = False
            self.get_logger().info(
                f"[DOCK_{sid}] ArUco dock + post-dock COMPLETE after {elapsed:.1f}s — "
                f"transitioning to ALIGN_AT_{sid}")
            self.transition_to(self.get_align_state(sid))

    def _dock_service_response(self, future, sid):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(
                    f"[DOCK_{sid}] Service ACCEPTED: {resp.message}")
            else:
                self.get_logger().error(
                    f"[DOCK_{sid}] Service REJECTED: {resp.message} — "
                    f"will trigger failure path")
                self.aruco_dock_done = True
        except Exception as e:
            self.get_logger().error(
                f"[DOCK_{sid}] Service call EXCEPTION: {e} — will trigger failure path")
            self.aruco_dock_done = True

    def handle_alignment(self):
        """Station A: wait for /receptacle/notify_aligned service call."""
        if self.alignment_start_time is None:
            self.alignment_start_time = time.time()
            self.get_logger().info(
                f"[ALIGN_A] Starting alignment | timeout={self.alignment_timeout}s | "
                f"offset={self.receptacle_offset} | "
                f"waiting for /receptacle/notify_aligned service call...")

        elapsed = time.time() - self.alignment_start_time
        remaining = self.alignment_timeout - elapsed

        if elapsed > self.alignment_timeout:
            self.get_logger().error(
                f"[ALIGN_A] TIMEOUT after {elapsed:.1f}s | "
                f"last_offset={self.receptacle_offset} | "
                f"notify_received={self.notify_aligned_received} — triggering failure")
            self.alignment_start_time = None
            self.handle_failure("A")
            return

        # Periodic alignment status
        if int(elapsed * 10) % 20 == 0 and elapsed > 0:  # every 2s
            detected = self.receptacle_offset != self.receptacle_not_detected
            self.get_logger().info(
                f"[ALIGN_A] {elapsed:.1f}s/{self.alignment_timeout}s "
                f"({remaining:.1f}s left) | "
                f"circle_detected={detected} offset={self.receptacle_offset}px | "
                f"notify_aligned={self.notify_aligned_received}")

        if self.receptacle_offset == self.receptacle_not_detected and elapsed > 3.0:
            self.get_logger().warn(
                f"[ALIGN_A] No circle detected after {elapsed:.1f}s — "
                f"{remaining:.1f}s until timeout",
                throttle_duration_sec=3.0,
            )

        if self.notify_aligned_received:
            self.get_logger().info(
                f"[ALIGN_A] ALIGNED after {elapsed:.1f}s | "
                f"final_offset={self.receptacle_offset}px — transitioning to FIRE_AT_A")
            self.notify_aligned_received = False
            self.alignment_start_time = None
            self.transition_to(MissionState.FIRE_AT_A)

    def handle_alignment_b(self):
        """Station B: aligner takes over alignment AND firing autonomously."""
        self.get_logger().info(
            "[ALIGN_B] Handing off to station_b_aligner for alignment + firing | "
            "FSM will wait for /receptacle/b_done")
        self.station_b_done = False
        self.transition_to(MissionState.FIRE_AT_B)

    def handle_firing_a(self):
        """Station A: FSM calls /fire_launcher with fixed timing delays."""
        if self.current_station_firing != "A":
            self.current_station_firing = "A"
            self.balls_fired            = 0
            self.get_logger().info(
                f"[FIRE_A] Starting firing sequence | "
                f"balls_per_station={self.balls_per_station} | "
                f"delays={self.station_a_delays}")

        if self.balls_fired >= self.balls_per_station:
            self.get_logger().info(
                f"[FIRE_A] All {self.balls_per_station} balls fired — "
                f"completing Station A delivery")
            self._complete_station("A")
            return

        # Wait for post-fire delay if needed
        if self.waiting_after_fire:
            delay   = self.get_delay_for_ball(self.balls_fired - 1)
            elapsed = time.time() - self.fire_wait_start_time
            remaining = delay - elapsed
            if elapsed < delay:
                if int(elapsed * 10) % 10 == 0 and elapsed > 0:  # every 1s
                    self.get_logger().info(
                        f"[FIRE_A] Delay after ball {self.balls_fired}: "
                        f"{remaining:.1f}s remaining (of {delay}s)")
                return
            self.waiting_after_fire = False
            self.fire_wait_start_time = None
            self.get_logger().info(
                f"[FIRE_A] Delay complete ({delay}s) — "
                f"ready for ball {self.balls_fired+1}")

        if not (self.launcher_ready and not self.launcher_request_pending
                and self.launcher_status == "idle"):
            self.get_logger().debug(
                f"[FIRE_A] Waiting for launcher: ready={self.launcher_ready} "
                f"pending={self.launcher_request_pending} status={self.launcher_status}")
            return

        if not self.fire_launcher_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn(
                "[FIRE_A] /fire_launcher service NOT AVAILABLE — waiting",
                throttle_duration_sec=2.0)
            return

        self.get_logger().info(
            f"[FIRE_A] >>> FIRING ball {self.balls_fired+1}/{self.balls_per_station} <<<")
        self.launcher_request_pending = True
        future = self.fire_launcher_client.call_async(Trigger.Request())
        future.add_done_callback(self._fire_response_cb)

    def _fire_response_cb(self, future):
        self.launcher_request_pending = False
        try:
            resp = future.result()
            if resp.success:
                self.launcher_ready = False
                self.get_logger().info(
                    f"[FIRE_A] Launcher ACCEPTED fire request: {resp.message}")
            else:
                self.get_logger().error(
                    f"[FIRE_A] Launcher REJECTED fire request: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"[FIRE_A] Launcher service EXCEPTION: {e}")

    def handle_firing_b(self):
        """Station B: aligner handles everything. FSM waits for b_done."""
        elapsed = time.time() - self._state_entry_time
        if int(elapsed * 10) % 100 == 0 and elapsed > 0:  # every 10s
            self.get_logger().info(
                f"[FIRE_B] Waiting for station_b_aligner... {elapsed:.0f}s | "
                f"b_done={self.station_b_done}")

        if self.station_b_done:
            self.get_logger().info(
                f"[FIRE_B] Station B autonomous firing COMPLETE after {elapsed:.1f}s")
            self._complete_station("B")

    def handle_complete(self):
        elapsed_mission = time.time() - self._mission_start_time
        self._set_exploration(False)
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("MISSION COMPLETE")
        self.get_logger().info(f"  Total mission time: {elapsed_mission:.1f}s")
        self.get_logger().info(
            f"  Station A: {'SUCCESS' if self.stations['A']['delivered'] else 'SKIPPED'}"
            f" (retries: {self.stations['A']['retry_count']})")
        self.get_logger().info(
            f"  Station B: {'SUCCESS' if self.stations['B']['delivered'] else 'SKIPPED'}"
            f" (retries: {self.stations['B']['retry_count']})")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")

    # =========================================================================
    # HELPERS
    # =========================================================================

    def transition_to(self, new_state):
        elapsed_in_old = time.time() - self._state_entry_time
        self.get_logger().info(
            f"[FSM] {self.state} -> {new_state} "
            f"(was in {self.state} for {elapsed_in_old:.1f}s)")
        self.previous_state = self.state
        self.state = new_state
        self._state_entry_time = time.time()

    def _set_exploration(self, enabled: bool):
        label = "ENABLE" if enabled else "DISABLE"
        if not self._exploration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"[EXPLORE] Cannot {label} exploration — service not available")
            return
        self.get_logger().info(f"[EXPLORE] {label} exploration requested")
        req = SetBool.Request()
        req.data = enabled
        future = self._exploration_client.call_async(req)
        future.add_done_callback(
            lambda f: self._exploration_response(f, enabled)
        )

    def _exploration_response(self, future, enabled):
        label = "enable" if enabled else "disable"
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(
                    f"[EXPLORE] {label} -> OK: {resp.message}")
            else:
                self.get_logger().warn(
                    f"[EXPLORE] {label} -> FAILED: {resp.message}")
        except Exception as e:
            self.get_logger().error(
                f"[EXPLORE] {label} service error: {e}")

    def _call_dock_scan(self):
        """Tell simple_aruco_dock to stop and return to scan mode."""
        if self.dock_scan_client.service_is_ready():
            self.get_logger().info(
                "[DOCK] Calling /aruco_dock/scan — cancelling post-dock maneuvers")
            future = self.dock_scan_client.call_async(Trigger.Request())
            future.add_done_callback(self._dock_scan_response)
        else:
            self.get_logger().warn(
                "[DOCK] /aruco_dock/scan service not ready — "
                "post-dock maneuvers may still be running!")

    def _dock_scan_response(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f"[DOCK] scan -> {resp.message}")
        except Exception as e:
            self.get_logger().error(f"[DOCK] scan service error: {e}")

    def _complete_station(self, sid):
        elapsed_mission = time.time() - self._mission_start_time
        self.get_logger().info(
            f"[COMPLETE] Station {sid} DELIVERED at {elapsed_mission:.1f}s into mission | "
            f"retries_used={self.stations[sid]['retry_count']}")
        self.stations[sid]["delivered"] = True
        self.balls_fired = 0
        self.waiting_after_fire = False
        self.current_station_firing = None
        self._dock_started = False

        other = "B" if sid == "A" else "A"
        if self.stations[other]["found"] and not self.stations[other]["delivered"]:
            self.get_logger().info(
                f"[COMPLETE] Station {other} already found — proceeding to dock")
            self.transition_to(self.get_dock_state(other))
        elif self.stations[other]["delivered"]:
            self.get_logger().info(
                f"[COMPLETE] Station {other} also delivered — MISSION COMPLETE")
            self.transition_to(MissionState.COMPLETE)
        else:
            self.get_logger().info(
                f"[COMPLETE] Station {other} not found yet — "
                f"resuming scan + exploration")
            self._call_dock_scan()
            self._set_exploration(True)
            self.transition_to(MissionState.EXPLORE)

    def handle_failure(self, sid):
        self.stations[sid]["retry_count"] += 1
        self._dock_started = False
        retries = self.stations[sid]["retry_count"]
        elapsed_state = time.time() - self._state_entry_time

        self.get_logger().error(
            f"[FAILURE] Station {sid} FAILED | "
            f"retry {retries}/{self.max_retries} | "
            f"was in {self.state} for {elapsed_state:.1f}s | "
            f"previous_state={self.previous_state}")

        if retries >= self.max_retries:
            self.get_logger().error(
                f"[FAILURE] Station {sid} EXHAUSTED all {self.max_retries} retries — SKIPPING")
            other = "B" if sid == "A" else "A"
            if self.stations[other]["found"] and not self.stations[other]["delivered"]:
                self.get_logger().info(
                    f"[FAILURE] Falling back to Station {other}")
                self.transition_to(self.get_dock_state(other))
            else:
                self.get_logger().info(
                    f"[FAILURE] Station {other} not found — resuming exploration")
                self._call_dock_scan()
                self._set_exploration(True)
                self.transition_to(MissionState.EXPLORE)
        else:
            self.get_logger().warn(
                f"[FAILURE] Retrying Station {sid} "
                f"(attempt {retries+1}/{self.max_retries})")
            self.transition_to(self.get_dock_state(sid))

    def get_delay_for_ball(self, ball_index):
        return self.station_a_delays.get(ball_index, 0.0)

    def get_dock_state(self, sid):
        return MissionState.DOCK_AT_A if sid == "A" else MissionState.DOCK_AT_B

    def get_align_state(self, sid):
        return MissionState.ALIGN_AT_A if sid == "A" else MissionState.ALIGN_AT_B

    def get_fire_state(self, sid):
        return MissionState.FIRE_AT_A if sid == "A" else MissionState.FIRE_AT_B


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseMissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[SHUTDOWN] KeyboardInterrupt — disabling exploration")
        node._set_exploration(False)
    finally:
        node.get_logger().info("[SHUTDOWN] Destroying node")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
