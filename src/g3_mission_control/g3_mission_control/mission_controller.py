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

        # ── Station tracking ────────────────────────────────────────────
        self.stations = {
            "A": {"pose": None, "found": False, "delivered": False, "retry_count": 0},
            "B": {"pose": None, "found": False, "delivered": False, "retry_count": 0},
        }
        self.max_retries       = 3
        self.exploration_complete = False

        # ── Alignment state ─────────────────────────────────────────────
        # FSM reads offset for logging only. Transition driven by service call.
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

        # From station_a_aligner: offset for logging only
        self.create_subscription(Int32, "/receptacle/offset", self.offset_callback, 10)

        # Service server: station_a_aligner calls this once when stably aligned
        self.create_service(Trigger, "/receptacle/notify_aligned", self.notify_aligned_handler)

        # From station_b_aligner: all-done signal
        self.create_subscription(Bool, "/receptacle/b_done", self.b_done_callback, 10)

        # From launcher node: status string
        self.create_subscription(String, "/launcher_status", self.launcher_callback, 10)

        # From simple_aruco_dock: docking complete signal
        self.create_subscription(Bool, "/aruco_dock/done", self.aruco_dock_done_callback, 10)

        # From exploration node
        self.create_subscription(Bool, "/exploration_complete", self.exploration_callback, 10)

        # ── Publishers ──────────────────────────────────────────────────
        self.state_pub   = self.create_publisher(String, "/mission_state", 10)
        # Emergency stop only — aligners own /cmd_vel during alignment
        self.cmd_vel_pub = self.create_publisher(Twist,  "/cmd_vel",       10)

        # ── Service clients ─────────────────────────────────────────────
        self.fire_launcher_client = self.create_client(Trigger, "/fire_launcher")
        self.dock_to_a_client     = self.create_client(Trigger, "/aruco_dock/dock_to_a")
        self.dock_to_b_client     = self.create_client(Trigger, "/aruco_dock/dock_to_b")
        self.dock_scan_client     = self.create_client(Trigger, "/aruco_dock/scan")
        self._exploration_client  = self.create_client(SetBool, "/exploration/set_enabled")

        # ── FSM timer: 10 Hz ────────────────────────────────────────────
        self.create_timer(0.1, self.state_machine_tick)

        self.get_logger().info("Mission Controller initialised (simple_aruco_dock)")
        self.get_logger().info("Station A: FSM fires via /fire_launcher service")
        self.get_logger().info("Station B: station_b_aligner fires autonomously")
        self.get_logger().info(f"Initial state: {self.state}")

    # =========================================================================
    # CALLBACKS
    # =========================================================================

    def station_a_callback(self, msg):
        if not self.stations["A"]["found"]:
            self.stations["A"]["pose"]  = msg
            self.stations["A"]["found"] = True
            self.get_logger().info("Station A detected")

    def station_b_callback(self, msg):
        if not self.stations["B"]["found"]:
            self.stations["B"]["pose"]  = msg
            self.stations["B"]["found"] = True
            self.get_logger().info("Station B detected")

    def offset_callback(self, msg):
        self.receptacle_offset = msg.data

    def notify_aligned_handler(self, _request, response):
        if self.state == MissionState.ALIGN_AT_A:
            self.notify_aligned_received = True
            self.get_logger().info("Alignment notification received — transitioning to FIRE_AT_A")
            response.success = True
            response.message = "Alignment accepted"
        else:
            self.get_logger().warn(
                f"Alignment notify received in wrong state: {self.state}")
            response.success = False
            response.message = f"Wrong state: {self.state}"
        return response

    def aruco_dock_done_callback(self, msg):
        """Only accept when FSM is actually in a DOCK state."""
        if self.state not in [MissionState.DOCK_AT_A, MissionState.DOCK_AT_B]:
            return
        if msg.data and not self.aruco_dock_done:
            self.aruco_dock_done = True
            self.get_logger().info("ArUco docking complete")

    def b_done_callback(self, msg):
        if msg.data and not self.station_b_done:
            self.station_b_done = True
            self.get_logger().info("Station B: all balls fired — b_done received")

    def launcher_callback(self, msg):
        self.launcher_status = msg.data
        if self.launcher_status == "complete":
            self.launcher_ready = True
            self.balls_fired   += 1
            self.get_logger().info(
                f"Ball {self.balls_fired}/{self.balls_per_station} confirmed fired")

            if self.current_station_firing == "A":
                delay = self.get_delay_for_ball(self.balls_fired - 1)
                if delay > 0:
                    self.waiting_after_fire   = True
                    self.fire_wait_start_time = time.time()
                    self.get_logger().info(f"Waiting {delay}s before next ball...")
        elif self.launcher_status == "error":
            self.launcher_ready = True
            self.get_logger().warn("Launcher error — will retry on next fire attempt")
        elif self.launcher_status == "idle" and not self.launcher_request_pending:
            self.launcher_ready = True

    def exploration_callback(self, msg):
        self.exploration_complete = msg.data
        if msg.data:
            self.get_logger().info("Exploration complete")

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
        dock_ready = self.dock_to_a_client.service_is_ready()
        if dock_ready:
            self.get_logger().info("simple_aruco_dock ready — exploring")
            self._set_exploration(True)
            self.transition_to(MissionState.EXPLORE)
        else:
            self.get_logger().warn(
                "Waiting for /aruco_dock/dock_to_a service...",
                throttle_duration_sec=2.0,
            )

    def handle_explore(self):
        # If station found and not delivered, start docking
        if self.stations["A"]["found"] and not self.stations["A"]["delivered"]:
            self.get_logger().info("Station A found — docking")
            self._set_exploration(False)
            self.transition_to(MissionState.DOCK_AT_A)
            return
        if self.stations["B"]["found"] and not self.stations["B"]["delivered"]:
            self.get_logger().info("Station B found — docking")
            self._set_exploration(False)
            self.transition_to(MissionState.DOCK_AT_B)
            return
        if self.stations["A"]["delivered"] and self.stations["B"]["delivered"]:
            self._set_exploration(False)
            self.transition_to(MissionState.COMPLETE)

    def handle_dock(self, sid):
        """Trigger dock service on entry, then wait for /aruco_dock/done."""
        if not self._dock_started:
            self.aruco_dock_done = False
            client = self.dock_to_a_client if sid == "A" else self.dock_to_b_client
            if client.service_is_ready():
                self.get_logger().info(f"Calling /aruco_dock/dock_to_{sid.lower()}")
                future = client.call_async(Trigger.Request())
                future.add_done_callback(
                    lambda f, s=sid: self._dock_service_response(f, s)
                )
                self._dock_started = True
            else:
                self.get_logger().warn(
                    f"Dock service for Station {sid} not ready — retrying",
                    throttle_duration_sec=2.0,
                )
            return

        if self.aruco_dock_done:
            self.aruco_dock_done = False
            self._dock_started = False
            # Stop any post-dock maneuvers before handing off to aligner
            self._call_dock_scan()
            self.get_logger().info(
                f"ArUco dock complete — transitioning to ALIGN at Station {sid}")
            self.transition_to(self.get_align_state(sid))

    def _dock_service_response(self, future, sid):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"Dock accepted for Station {sid}: {resp.message}")
            else:
                self.get_logger().warn(f"Dock rejected for Station {sid}: {resp.message}")
                self.aruco_dock_done = True  # force failure path
        except Exception as e:
            self.get_logger().error(f"Dock service error for Station {sid}: {e}")
            self.aruco_dock_done = True

    def handle_alignment(self):
        """Station A: wait for /receptacle/notify_aligned service call."""
        if self.alignment_start_time is None:
            self.alignment_start_time = time.time()
            self.get_logger().info("Waiting for alignment at Station A...")

        elapsed = time.time() - self.alignment_start_time
        if elapsed > self.alignment_timeout:
            self.get_logger().warn("Alignment timeout at Station A")
            self.alignment_start_time = None
            self.handle_failure("A")
            return

        if self.receptacle_offset == self.receptacle_not_detected and elapsed > 3.0 and elapsed % 5.0 < 0.1:
            self.get_logger().warn("No circle detected at Station A — still waiting...")

        if self.notify_aligned_received:
            self.notify_aligned_received = False
            self.alignment_start_time = None
            self.transition_to(MissionState.FIRE_AT_A)

    def handle_alignment_b(self):
        """Station B: aligner takes over alignment AND firing autonomously."""
        self.get_logger().info(
            "Station B: handing off to station_b_aligner for alignment + firing")
        self.station_b_done = False
        self.transition_to(MissionState.FIRE_AT_B)

    def handle_firing_a(self):
        """Station A: FSM calls /fire_launcher with fixed timing delays."""
        if self.current_station_firing != "A":
            self.current_station_firing = "A"
            self.balls_fired            = 0
            self.get_logger().info("Starting Station A firing sequence")

        if self.balls_fired >= self.balls_per_station:
            self._complete_station("A")
            return

        # Wait for post-fire delay if needed
        if self.waiting_after_fire:
            delay   = self.get_delay_for_ball(self.balls_fired - 1)
            elapsed = time.time() - self.fire_wait_start_time
            if elapsed < delay:
                return
            self.waiting_after_fire = False
            self.fire_wait_start_time = None
            self.get_logger().info(f"Delay complete — ready for ball {self.balls_fired+1}")

        if not (self.launcher_ready and not self.launcher_request_pending
                and self.launcher_status == "idle"):
            return

        # Call fire service for next ball
        if not self.fire_launcher_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn("Launcher service not available")
            return

        self.get_logger().info(
            f"Firing ball {self.balls_fired+1}/{self.balls_per_station} at Station A")
        self.launcher_request_pending = True
        future = self.fire_launcher_client.call_async(Trigger.Request())
        future.add_done_callback(self._fire_response_cb)

    def _fire_response_cb(self, future):
        self.launcher_request_pending = False
        try:
            resp = future.result()
            if resp.success:
                self.launcher_ready = False
                self.get_logger().info(f"Launcher accepted: {resp.message}")
            else:
                self.get_logger().warn(f"Launcher rejected: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"Launcher service error: {e}")

    def handle_firing_b(self):
        """Station B: aligner handles everything. FSM waits for b_done."""
        if self.station_b_done:
            self.get_logger().info("Station B firing complete (b_done received)")
            self._complete_station("B")

    def handle_complete(self):
        self._set_exploration(False)
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("=" * 60)
        self.get_logger().info("MISSION COMPLETE")
        self.get_logger().info(
            f"  Station A: {'SUCCESS' if self.stations['A']['delivered'] else 'SKIPPED'}")
        self.get_logger().info(
            f"  Station B: {'SUCCESS' if self.stations['B']['delivered'] else 'SKIPPED'}")
        self.get_logger().info("=" * 60)

    # =========================================================================
    # HELPERS
    # =========================================================================

    def transition_to(self, new_state):
        self.get_logger().info(f"FSM: {self.state} -> {new_state}")
        self.previous_state = self.state
        self.state = new_state

    def _set_exploration(self, enabled: bool):
        if not self._exploration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Exploration service not available")
            return
        req = SetBool.Request()
        req.data = enabled
        self._exploration_client.call_async(req)

    def _call_dock_scan(self):
        """Tell simple_aruco_dock to stop and return to scan mode."""
        if self.dock_scan_client.service_is_ready():
            self.dock_scan_client.call_async(Trigger.Request())

    def _complete_station(self, sid):
        self.get_logger().info(f"Station {sid} delivered!")
        self.stations[sid]["delivered"] = True
        self.balls_fired = 0
        self.waiting_after_fire = False
        self.current_station_firing = None
        self._dock_started = False

        other = "B" if sid == "A" else "A"
        if self.stations[other]["found"] and not self.stations[other]["delivered"]:
            self.transition_to(self.get_dock_state(other))
        elif self.stations[other]["delivered"]:
            self.transition_to(MissionState.COMPLETE)
        else:
            # Other station not found yet — resume scanning and exploration
            self._call_dock_scan()
            self._set_exploration(True)
            self.transition_to(MissionState.EXPLORE)

    def handle_failure(self, sid):
        self.stations[sid]["retry_count"] += 1
        self._dock_started = False
        retries = self.stations[sid]["retry_count"]
        if retries >= self.max_retries:
            self.get_logger().error(
                f"Station {sid} failed {self.max_retries}x — skipping")
            other = "B" if sid == "A" else "A"
            if self.stations[other]["found"] and not self.stations[other]["delivered"]:
                self.transition_to(self.get_dock_state(other))
            else:
                self._call_dock_scan()
                self._set_exploration(True)
                self.transition_to(MissionState.EXPLORE)
        else:
            self.get_logger().warn(
                f"Retry {retries}/{self.max_retries} for Station {sid}")
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
        node._set_exploration(False)
    finally:
        node.get_logger().info("Shutting down")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
