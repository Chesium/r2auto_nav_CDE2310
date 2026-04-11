#!/usr/bin/env python3
"""
High-level FSM.

Key Changes from Previous Version:
  1. REMOVED angular P-control from offset_callback.
     Station A aligner owns /cmd_vel during alignment — FSM must not fight it.
  2. FIRE_AT_B: FSM does NOT call /fire_launcher for Station B.
     Station B aligner handles its own predictive firing via service.
     FSM waits for /receptacle/b_done = True then marks B delivered.
  3. FIRE_AT_A: FSM still calls /fire_launcher service (fixed timing delays).
     This is correct — Station A has fixed timing the TA specifies.
  4. alignment_callback: no longer publishes cmd_vel (aligner owns it).
  5. Added /receptacle/b_done subscriber for Station B completion signal.

Topic/Service Contracts:
  From station_a_aligner.py:
    /receptacle/offset          Int32   — logged only
    /receptacle/notify_aligned  Trigger — service called once when stably aligned
  From station_b_aligner.py:
    /receptacle/b_done     Bool   — FSM uses to complete Station B delivery
  To launcher (std_srvs Trigger):
    /fire_launcher                — called by FSM for Station A only
  From launcher:
    /launcher_status       String — "idle"|"firing"|"complete"|"error"
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String, Int32
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import time


class MissionState:
    INIT          = "INIT"
    EXPLORE       = "EXPLORE"
    NAVIGATE_TO_A = "NAVIGATE_TO_A"
    DOCK_AT_A     = "DOCK_AT_A"
    ALIGN_AT_A    = "ALIGN_AT_A"
    FIRE_AT_A     = "FIRE_AT_A"
    NAVIGATE_TO_B = "NAVIGATE_TO_B"
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

        # ── Station tracking ────────────────────────────────────────────
        self.stations = {
            "A": {"pose": None, "found": False, "delivered": False, "retry_count": 0},
            "B": {"pose": None, "found": False, "delivered": False, "retry_count": 0},
        }
        self.max_retries       = 3
        self.exploration_complete = False

        # ── Alignment state (from station_a_aligner) ────────────────────
        # FSM reads offset for logging only. Alignment transition driven by service call.
        self.receptacle_not_detected = 9999
        self.receptacle_offset       = self.receptacle_not_detected
        self.alignment_timeout       = 15.0
        self.alignment_start_time    = None
        self.notify_aligned_received = False

        # ── Station B completion flag (from station_b_aligner) ──────────
        self.station_b_done = False

        # ── ArUco dock completion flag ───────────────────────────────────
        self.aruco_dock_done = False

        # ── Launcher state (Station A only) ─────────────────────────────
        self.launcher_ready           = True
        self.launcher_status          = "idle"
        self.launcher_request_pending = False
        self.balls_fired              = 0
        self.balls_per_station        = 3
        self.current_station_firing   = None

        # Station A timing delays (seconds after each ball)
        # TA will specify exact delays in week 7 — update these values.
        self.declare_parameter("station_a_delay_after_ball_1", 4.8)
        self.declare_parameter("station_a_delay_after_ball_2", 0.8)
        self.declare_parameter("station_a_delay_after_ball_3", 0.0)
        self.station_a_delays = {
            0: float(self.get_parameter("station_a_delay_after_ball_1").value),
            1: float(self.get_parameter("station_a_delay_after_ball_2").value),
            2: float(self.get_parameter("station_a_delay_after_ball_3").value),
        }
        self.waiting_after_fire   = False
        self.fire_wait_start_time = None

        # ── Nav2 async state ────────────────────────────────────────────
        self.nav_in_progress = False
        self.nav_succeeded   = None

        # ── Subscribers ─────────────────────────────────────────────────
        # From ArUco node: station poses
        self.create_subscription(PoseStamped, "/station_a_pose", self.station_a_callback, 10)
        self.create_subscription(PoseStamped, "/station_b_pose", self.station_b_callback, 10)

        # From station_a_aligner: offset for logging only
        self.create_subscription(Int32, "/receptacle/offset", self.offset_callback, 10)

        # Service server: station_a_aligner calls this once when stably aligned
        self.create_service(Trigger, "/receptacle/notify_aligned", self.notify_aligned_handler)

        # From station_b_aligner: all-done signal (used to complete Station B delivery)
        self.create_subscription(Bool, "/receptacle/b_done", self.b_done_callback, 10)

        # From launcher node: status string
        self.create_subscription(String, "/launcher_status", self.launcher_callback, 10)

        # From aruco_dock_node: docking complete signal
        self.create_subscription(Bool, "/aruco_dock/done", self.aruco_dock_done_callback, 10)

        # From exploration node: map closure signal
        self.create_subscription(Bool, "/exploration_complete", self.exploration_callback, 10)

        # ── Publishers ──────────────────────────────────────────────────
        # FSM does NOT publish /cmd_vel — aligners own it.
        self.state_pub   = self.create_publisher(String, "/mission_state", 10)
        # Emergency stop only
        self.cmd_vel_pub = self.create_publisher(Twist,  "/cmd_vel",       10)

        # ── Service clients ──────────────────────────────────────────────
        self.fire_launcher_client = self.create_client(Trigger, "/fire_launcher")
        self.dock_to_a_client     = self.create_client(Trigger, "/aruco_dock/dock_to_a")
        self.dock_to_b_client     = self.create_client(Trigger, "/aruco_dock/dock_to_b")
        self.dock_scan_client     = self.create_client(Trigger, "/aruco_dock/scan")

        # ── Nav2 ────────────────────────────────────────────────────────
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── FSM timer: 10 Hz ────────────────────────────────────────────
        self.create_timer(0.1, self.state_machine_tick)

        self.get_logger().info("Mission Controller initialised")
        self.get_logger().info("Station A: FSM fires via /fire_launcher service")
        self.get_logger().info("Station B: station_b_aligner fires autonomously")
        self.get_logger().info(f"Initial state: {self.state}")

    # =========================================================================
    # CALLBACKS
    # =========================================================================

    def station_a_callback(self, msg):
        """aruco_dock_node detected Station A — store pose and begin docking."""
        if not self.stations["A"]["found"]:
            self.stations["A"]["pose"]  = msg
            self.stations["A"]["found"] = True
            self.get_logger().info("✓ Station A detected — transitioning to DOCK_AT_A")
            if self.state == MissionState.EXPLORE:
                self.aruco_dock_done = False
                self.transition_to(MissionState.DOCK_AT_A)

    def station_b_callback(self, msg):
        """aruco_dock_node detected Station B — store pose and begin docking."""
        if not self.stations["B"]["found"]:
            self.stations["B"]["pose"]  = msg
            self.stations["B"]["found"] = True
            self.get_logger().info("✓ Station B detected — transitioning to DOCK_AT_B")
            if self.state == MissionState.EXPLORE:
                self.aruco_dock_done = False
                self.transition_to(MissionState.DOCK_AT_B)

    def offset_callback(self, msg):
        """Pixel offset from station_a_aligner — store for logging only.
        FSM does NOT publish cmd_vel here. Aligner owns cmd_vel."""
        self.receptacle_offset = msg.data

    def notify_aligned_handler(self, _request, response):
        """station_a_aligner calls this once when stably aligned."""
        if self.state == MissionState.ALIGN_AT_A:
            self.notify_aligned_received = True
            self.get_logger().info("✓ Alignment notification received — transitioning to FIRE_AT_A")
            response.success = True
            response.message = "Alignment accepted"
        else:
            self.get_logger().warn(
                f"Alignment notify received in wrong state: {self.state}")
            response.success = False
            response.message = f"Wrong state: {self.state}"
        return response

    def aruco_dock_done_callback(self, msg):
        """aruco_dock_node signals docking approach complete."""
        if msg.data and not self.aruco_dock_done:
            self.aruco_dock_done = True
            self.get_logger().info("✓ ArUco docking complete")

    def b_done_callback(self, msg):
        """Station B aligner signals all 3 balls fired."""
        if msg.data and not self.station_b_done:
            self.station_b_done = True
            self.get_logger().info("✓ Station B: all balls fired — b_done received")

    def launcher_callback(self, msg):
        """Launcher status: idle | firing | complete | error.
        Used for Station A firing sequence only."""
        prev_status = self.launcher_status
        self.launcher_status = msg.data
        if self.launcher_status == "complete" and prev_status != "complete":
            self.launcher_ready = True
            self.balls_fired   += 1
            self.get_logger().info(
                f"Ball {self.balls_fired}/{self.balls_per_station} confirmed fired")

            if self.current_station_firing == "A":
                delay = self.station_a_delays.get(self.balls_fired - 1, 0.0)
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
        """Exploration node signals map closure."""
        self.exploration_complete = msg.data
        if msg.data:
            self.get_logger().info("✓ Exploration complete")

    # =========================================================================
    # FSM TICK (10 Hz)
    # =========================================================================

    def state_machine_tick(self):
        s = String(); s.data = self.state
        self.state_pub.publish(s)

        if   self.state == MissionState.INIT:          self.handle_init()
        elif self.state == MissionState.EXPLORE:        self.handle_explore()
        elif self.state == MissionState.NAVIGATE_TO_A:  self.handle_navigate("A")
        elif self.state == MissionState.DOCK_AT_A:      self.handle_dock("A")
        elif self.state == MissionState.ALIGN_AT_A:     self.handle_alignment("A")
        elif self.state == MissionState.FIRE_AT_A:      self.handle_firing_a()
        elif self.state == MissionState.NAVIGATE_TO_B:  self.handle_navigate("B")
        elif self.state == MissionState.DOCK_AT_B:      self.handle_dock("B")
        elif self.state == MissionState.ALIGN_AT_B:     self.handle_alignment_b()
        elif self.state == MissionState.FIRE_AT_B:      self.handle_firing_b()
        elif self.state == MissionState.COMPLETE:       self.handle_complete()

    # =========================================================================
    # STATE HANDLERS
    # =========================================================================

    def handle_init(self):
        """Wait for Nav2 then start exploring."""
        if self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("Nav2 ready — exploring")
            self.transition_to(MissionState.EXPLORE)
        else:
            self.get_logger().warn("Nav2 not ready yet...")

    def handle_explore(self):
        """Navigate to whichever station is found first."""
        if self.stations["A"]["found"] and not self.stations["A"]["delivered"]:
            self.get_logger().info("Station A found — navigating")
            self.transition_to(MissionState.NAVIGATE_TO_A); return
        if self.stations["B"]["found"] and not self.stations["B"]["delivered"]:
            self.get_logger().info("Station B found — navigating")
            self.transition_to(MissionState.NAVIGATE_TO_B); return
        if self.stations["A"]["delivered"] and self.stations["B"]["delivered"]:
            self.transition_to(MissionState.COMPLETE)

    def handle_navigate(self, sid):
        """Send Nav2 goal and wait for result."""
        station = self.stations[sid]
        if station["pose"] is None:
            self.get_logger().error(f"No pose for Station {sid}!")
            self.handle_failure(sid); return

        if not self.nav_in_progress and self.nav_succeeded is None:
            self.get_logger().info(f"Navigating to Station {sid}...")
            self._send_nav_goal(station["pose"]); self.nav_in_progress = True; return

        if self.nav_in_progress: return

        success = self.nav_succeeded; self.nav_succeeded = None
        if success:
            self.get_logger().info(f"Arrived at Station {sid} — starting ArUco dock")
            self.transition_to(MissionState.DOCK_AT_A if sid == "A"
                               else MissionState.DOCK_AT_B)
        else:
            self.get_logger().error(f"Nav2 failed for Station {sid}")
            self.handle_failure(sid)

    def handle_dock(self, sid):
        """
        aruco_dock_node already started docking when it published the station pose.
        FSM just waits for /aruco_dock/done = True then hands off to the aligner.
        """
        if self.aruco_dock_done:
            self.aruco_dock_done = False
            self.get_logger().info(f"ArUco dock complete — transitioning to ALIGN at Station {sid}")
            self.transition_to(MissionState.ALIGN_AT_A if sid == "A"
                               else MissionState.ALIGN_AT_B)

    def handle_alignment(self, sid):
        """
        Station A alignment: wait for /receptacle/aligned from station_a_aligner.
        FSM does NOT publish cmd_vel — aligner owns it.
        """
        if self.alignment_start_time is None:
            self.alignment_start_time = time.time()
            self.get_logger().info(f"Waiting for alignment at Station {sid}...")

        elapsed = time.time() - self.alignment_start_time
        if elapsed > self.alignment_timeout:
            self.get_logger().warn(f"Alignment timeout at Station {sid}")
            self.alignment_start_time = None
            self.handle_failure(sid); return

        if self.receptacle_offset == self.receptacle_not_detected and elapsed > 3.0 and elapsed % 5.0 < 0.1:
            self.get_logger().warn(f"No circle detected at Station {sid} — still waiting...")

        if self.notify_aligned_received:
            self.notify_aligned_received = False
            self.alignment_start_time = None
            self.transition_to(MissionState.FIRE_AT_A if sid == "A"
                               else MissionState.FIRE_AT_B)

    def handle_alignment_b(self):
        """
        Station B: after navigation, station_b_aligner takes over alignment AND firing.
        FSM just transitions to FIRE_AT_B immediately — the aligner starts automatically.
        """
        self.get_logger().info(
            "Station B: handing off to station_b_aligner for alignment + firing")
        self.station_b_done = False  # reset done flag for this run
        self.transition_to(MissionState.FIRE_AT_B)

    def handle_firing_a(self):
        """
        Station A firing: FSM calls /fire_launcher service with fixed timing delays.
        Waits for launcher_status='complete' between balls.
        """
        if self.current_station_firing != "A":
            self.current_station_firing = "A"
            self.balls_fired            = 0
            self.get_logger().info("Starting Station A firing sequence")

        if self.balls_fired >= self.balls_per_station:
            self._complete_station("A"); return

        # Wait for post-fire delay if needed
        if self.waiting_after_fire:
            delay   = self.station_a_delays.get(self.balls_fired - 1, 0.0)
            elapsed = time.time() - self.fire_wait_start_time
            if elapsed < delay:
                return
            self.waiting_after_fire = False; self.fire_wait_start_time = None
            self.get_logger().info(f"Delay complete — ready for ball {self.balls_fired+1}")

        if not (self.launcher_ready and not self.launcher_request_pending
                and self.launcher_status == "idle"):
            return

        # Call fire service for next ball
        if not self.fire_launcher_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn("Launcher service not available"); return

        self.get_logger().info(
            f"Firing ball {self.balls_fired+1}/{self.balls_per_station} at Station A")
        self.launcher_request_pending = True
        future = self.fire_launcher_client.call_async(Trigger.Request())
        future.add_done_callback(self._fire_response_cb)

    def _fire_response_cb(self, future):
        """Launcher service response for Station A."""
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
        """
        Station B firing: station_b_aligner handles EVERYTHING autonomously.
        FSM just waits for /receptacle/b_done = True then marks delivery done.
        """
        if self.station_b_done:
            self.get_logger().info("Station B firing complete (b_done received)")
            self._complete_station("B")

    def handle_complete(self):
        """Mission done."""
        self.cmd_vel_pub.publish(Twist())  # ensure bot stopped
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
        self.get_logger().info(f"FSM: {self.state} → {new_state}")
        self.state = new_state

    def _complete_station(self, sid):
        self.get_logger().info(f"Station {sid} delivered!")
        self.stations[sid]["delivered"] = True
        self.balls_fired = 0; self.waiting_after_fire = False
        self.current_station_firing = None

        other = "B" if sid == "A" else "A"
        if self.stations[other]["found"] and not self.stations[other]["delivered"]:
            # Activate aruco_dock for the other station (it is idle in DONE)
            self.aruco_dock_done = False
            client = self.dock_to_a_client if other == "A" else self.dock_to_b_client
            if client.wait_for_service(timeout_sec=0.0):
                future = client.call_async(Trigger.Request())
                future.add_done_callback(lambda f: self.get_logger().info(
                    f"Dock activated for Station {other}: {f.result().message}"))
            else:
                self.get_logger().warn(f"aruco_dock service unavailable for Station {other}")
            self.transition_to(MissionState.DOCK_AT_A if other == "A"
                               else MissionState.DOCK_AT_B)
        elif self.stations[other]["delivered"]:
            self.transition_to(MissionState.COMPLETE)
        else:
            # Other station not found yet — wake aruco to scan for it
            if self.dock_scan_client.wait_for_service(timeout_sec=0.0):
                self.dock_scan_client.call_async(Trigger.Request())
            self.transition_to(MissionState.EXPLORE)

    def handle_failure(self, sid):
        self.stations[sid]["retry_count"] += 1
        retries = self.stations[sid]["retry_count"]
        if retries >= self.max_retries:
            self.get_logger().error(f"Station {sid} failed {self.max_retries}x — skipping")
            other = "B" if sid == "A" else "A"
            self.transition_to(MissionState.NAVIGATE_TO_A if (
                self.stations[other]["found"] and other == "A")
                else MissionState.NAVIGATE_TO_B if (
                self.stations[other]["found"] and other == "B")
                else MissionState.EXPLORE)
        else:
            self.get_logger().warn(f"Retry {retries}/{self.max_retries} for Station {sid}")
            self.transition_to(MissionState.NAVIGATE_TO_A if sid == "A"
                               else MissionState.NAVIGATE_TO_B)

    def _send_nav_goal(self, pose):
        goal = NavigateToPose.Goal(); goal.pose = pose
        self.nav_succeeded = None
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_accepted)

    def _nav_goal_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Nav2 rejected goal")
            self.nav_in_progress = False; self.nav_succeeded = False; return
        self.get_logger().info("Nav2 goal accepted")
        handle.get_result_async().add_done_callback(self._nav_result)

    def _nav_result(self, future):
        try:
            r = future.result()
            self.nav_succeeded = r.status == GoalStatus.STATUS_SUCCEEDED
            if not self.nav_succeeded:
                self.get_logger().warn(f"Nav2 status: {r.status}")
        except Exception as e:
            self.get_logger().error(f"Nav2 result error: {e}")
            self.nav_succeeded = False
        finally:
            self.nav_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseMissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()