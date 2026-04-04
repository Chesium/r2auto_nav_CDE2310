#!/usr/bin/env python3
"""
Warehouse Mission Controller - Python Finite State Machine

Navigate to stations immediately when found during exploration

Station A (Static): Ball 1 → Wait 7s → Ball 2 → Wait 3s → Ball 3
Station B (Moving): Ball 1 → Ball 2 → Ball 3 (no delays)

written by Daphne
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String, Int32
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import time


class MissionState:
    """Mission states enumeration"""

    INIT = "INIT"
    EXPLORE = "EXPLORE"
    NAVIGATE_TO_A = "NAVIGATE_TO_A"
    ALIGN_AT_A = "ALIGN_AT_A"
    FIRE_AT_A = "FIRE_AT_A"
    NAVIGATE_TO_B = "NAVIGATE_TO_B"
    ALIGN_AT_B = "ALIGN_AT_B"
    FIRE_AT_B = "FIRE_AT_B"
    BONUS = "BONUS"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


class WarehouseMissionController(Node):
    """
    Main mission controller using Finite State Machine
    STRATEGY: Deliver to stations as soon as they're found

    Subscribes to:
        /station_a_pose (PoseStamped) - ArUco detected pose of Station A
        /station_b_pose (PoseStamped) - ArUco detected pose of Station B
        /receptacle/offset (Int32) - Hough circle offset from center
        /receptacle/aligned (Bool) - Hough alignment status
        /launcher_status (String) - Launcher firing status
        /exploration_complete (Bool) - Both stations found flag

    Publishes to:
        /cmd_vel (Twist) - Velocity commands for alignment
        /fire_launcher (Bool) - Trigger launcher
        /mission_state (String) - Current state for monitoring

    Action Clients:
        /navigate_to_pose - Nav2 navigation action
    """

    def __init__(self):
        super().__init__("mission_controller")

        # IMPORTANT INTEGRATION NOTE:
        # This node is a high-level executor, not a mapper/explorer.
        # Exploration/localization/perception nodes must provide station poses.

        # ========== STATE MACHINE ==========
        self.state = MissionState.INIT
        self.previous_state = None

        # ========== MISSION DATA ==========
        self.stations = {
            "A": {"pose": None, "found": False, "delivered": False, "retry_count": 0},
            "B": {"pose": None, "found": False, "delivered": False, "retry_count": 0},
        }
        self.max_retries = 3
        self.exploration_complete = False

        # ========== ALIGNMENT DATA (HOUGH) ==========
        self.receptacle_not_detected = 9999
        self.receptacle_offset = self.receptacle_not_detected
        self.is_aligned = False
        self.alignment_timeout = 10.0  # seconds
        self.alignment_start_time = None
  
        # P-controller parameters for alignment
        self.declare_parameter("alignment_kp", 0.003)
        self.declare_parameter("max_angular_vel", 0.5)
        self.kp = self.get_parameter("alignment_kp").value
        self.max_angular_vel = self.get_parameter("max_angular_vel").value

        # ========== LAUNCHER DATA ==========
        self.launcher_ready = True
        self.launcher_status = "idle"
        self.balls_fired = 0
        self.balls_per_station = 3

        # ========== TIMING DELAYS ==========
        # 2.3s to shoot ball
        self.declare_parameter("station_a_delay_after_ball_1", 4.7)
        self.declare_parameter("station_a_delay_after_ball_2", 0.7)
        self.declare_parameter("station_a_delay_after_ball_3", 0.0)
        self.declare_parameter("station_b_delay_after_ball_1", 0.0)
        self.declare_parameter("station_b_delay_after_ball_2", 0.0)
        self.declare_parameter("station_b_delay_after_ball_3", 0.0)

        self.station_a_delays = {
            0: float(self.get_parameter("station_a_delay_after_ball_1").value),
            1: float(self.get_parameter("station_a_delay_after_ball_2").value),
            2: float(self.get_parameter("station_a_delay_after_ball_3").value),
        }
        self.station_b_delays = {
            0: float(self.get_parameter("station_b_delay_after_ball_1").value),
            1: float(self.get_parameter("station_b_delay_after_ball_2").value),
            2: float(self.get_parameter("station_b_delay_after_ball_3").value),
        }
        self.waiting_after_fire = False
        self.fire_wait_start_time = None
        self.current_station_firing = None  # Track which station we're firing at

        # ========== SUBSCRIBERS ==========
        self.create_subscription(
            PoseStamped, "/station_a_pose", self.station_a_callback, 10
        )
        self.create_subscription(
            PoseStamped, "/station_b_pose", self.station_b_callback, 10
        )
        self.create_subscription(Int32, "/receptacle/offset", self.offset_callback, 10)
        self.create_subscription(
            Bool, "/receptacle/aligned", self.alignment_callback, 10
        )
        self.create_subscription(String, "/launcher_status", self.launcher_callback, 10)
        self.create_subscription(
            Bool, "/exploration_complete", self.exploration_callback, 10
        )

        # Contract to exploration teammate:
        # 1) Publish /station_a_pose and /station_b_pose when detected.
        # 2) Optional: publish /exploration_complete when map/search phase is done.

        # ========== PUBLISHERS ==========
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.fire_launcher_pub = self.create_publisher(Bool, "/fire_launcher", 10)
        self.state_pub = self.create_publisher(String, "/mission_state", 10)

        # ========== ACTION CLIENTS ==========
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_in_progress = False
        self.nav_succeeded = None

        # ========== STATE MACHINE TIMER ==========
        self.create_timer(0.1, self.state_machine_tick)  # 10Hz

        self.get_logger().info("Mission Controller Initialized")
        self.get_logger().info("Strategy: DELIVER AS FOUND (Time-Optimized)")
        self.get_logger().info("Station A: 7s + 3s delays | Station B: No delays")
        self.get_logger().info(f"Initial State: {self.state}")

    # ========================================
    # CALLBACK FUNCTIONS
    # ========================================

    def station_a_callback(self, msg):
        """Receive Station A pose from ArUco detection"""
        if not self.stations["A"]["found"]:
            self.stations["A"]["pose"] = msg
            self.stations["A"]["found"] = True
            self.get_logger().info("✓ Station A detected!")
            self.get_logger().info("→ Will navigate to A on next state tick")

    def station_b_callback(self, msg):
        """Receive Station B pose from ArUco detection"""
        if not self.stations["B"]["found"]:
            self.stations["B"]["pose"] = msg
            self.stations["B"]["found"] = True
            self.get_logger().info("✓ Station B detected!")
            self.get_logger().info("→ Will navigate to B on next state tick")

    def offset_callback(self, msg):
        """
        Receive Hough circle offset and apply P-controller
        Runs at camera frame rate (~30Hz) for smooth alignment
        """
        self.receptacle_offset = msg.data

        # Apply P-controller for alignment (if offset is valid)
        if self.receptacle_offset != self.receptacle_not_detected:
            # Calculate angular velocity using P-controller
            angular_vel = -self.kp * self.receptacle_offset

            # Clamp to max velocity
            if abs(angular_vel) > self.max_angular_vel:
                angular_vel = (
                    self.max_angular_vel if angular_vel > 0 else -self.max_angular_vel
                )

            # Publish velocity command ONLY during alignment states
            if self.state in [MissionState.ALIGN_AT_A, MissionState.ALIGN_AT_B]:
                cmd = Twist()
                cmd.angular.z = angular_vel
                self.cmd_vel_pub.publish(cmd)

    def alignment_callback(self, msg):
        """Receive Hough alignment status"""
        self.is_aligned = msg.data

        # Stop robot when aligned
        if self.is_aligned and self.state in [
            MissionState.ALIGN_AT_A,
            MissionState.ALIGN_AT_B,
        ]:
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

    def launcher_callback(self, msg):
        """
        Receive launcher status and start timing delays
        """
        self.launcher_status = msg.data

        if self.launcher_status == "complete":
            self.launcher_ready = True
            self.balls_fired += 1

            # Start waiting timer after fire
            if self.current_station_firing is not None:
                station_id = self.current_station_firing
                delay = self.get_delay_for_ball(station_id, self.balls_fired - 1)

                if delay > 0:
                    self.waiting_after_fire = True
                    self.fire_wait_start_time = time.time()
                    self.get_logger().info(
                        f" Ball {self.balls_fired} fired. Waiting {delay}s before next ball..."
                    )
                else:
                    self.get_logger().info(
                        f"Ball {self.balls_fired} fired. No delay required."
                    )

    def exploration_callback(self, msg):
        """Receive exploration completion status"""
        self.exploration_complete = msg.data
        if self.exploration_complete:
            self.get_logger().info("Exploration complete! Both stations found.")

    # ========================================
    # MAIN STATE MACHINE
    # ========================================

    def state_machine_tick(self):
        """
        Main state machine loop - runs at 10Hz
        """
        # Single-threaded FSM tick: read current data, then run one state handler.
        # Publish current state for monitoring
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        # State transition logic
        if self.state == MissionState.INIT:
            self.handle_init()

        elif self.state == MissionState.EXPLORE:
            self.handle_explore()

        elif self.state == MissionState.NAVIGATE_TO_A:
            self.handle_navigate_to_station("A")

        elif self.state == MissionState.ALIGN_AT_A:
            self.handle_alignment("A")

        elif self.state == MissionState.FIRE_AT_A:
            self.handle_firing("A")

        elif self.state == MissionState.NAVIGATE_TO_B:
            self.handle_navigate_to_station("B")

        elif self.state == MissionState.ALIGN_AT_B:
            self.handle_alignment("B")

        elif self.state == MissionState.FIRE_AT_B:
            self.handle_firing("B")

        elif self.state == MissionState.COMPLETE:
            self.handle_complete()

    # ========================================
    # STATE HANDLERS
    # ========================================

    def handle_init(self):
        """Initialize mission"""
        self.get_logger().info("Initializing mission...")
        self.get_logger().info("Waiting for Nav2 action server...")

        # Wait for Nav2 to be ready
        if self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("Nav2 ready!")
            self.transition_to(MissionState.EXPLORE)
        else:
            self.get_logger().warn("Nav2 not available, retrying...")

    def handle_explore(self):
        """
        Exploration phase - deliver to stations as soon as they're found
        """
        # If Station A found and not delivered, go deliver immediately
        if self.stations["A"]["found"] and not self.stations["A"]["delivered"]:
            self.get_logger().info(
                "Station A found! Navigating immediately to secure points."
            )
            self.transition_to(MissionState.NAVIGATE_TO_A)
            return

        # If Station B found and not delivered, go deliver immediately
        if self.stations["B"]["found"] and not self.stations["B"]["delivered"]:
            self.get_logger().info(
                "Station B found! Navigating immediately to secure points."
            )
            self.transition_to(MissionState.NAVIGATE_TO_B)
            return

        # If both delivered, mission complete
        if self.stations["A"]["delivered"] and self.stations["B"]["delivered"]:
            self.get_logger().info("All deliveries complete!")
            self.transition_to(MissionState.COMPLETE)

        # Still exploring for unfound stations
        # Exploration is handled by separate exploration_node.py
        # This FSM just monitors stations and transitions when they're found

    def handle_navigate_to_station(self, station_id):
        """Navigate to a station using Nav2"""
        station = self.stations[station_id]

        if station["pose"] is None:
            self.get_logger().error(f"Station {station_id} pose not available!")
            self.handle_station_failure(station_id)
            return

        # Send navigation goal (only once)
        if not self.nav_in_progress:
            self.get_logger().info(f"Navigating to Station {station_id}...")
            self.send_nav_goal(station["pose"])
            self.nav_in_progress = True
            return

        # Wait here until async Nav2 result callback sets self.nav_succeeded.
        if self.nav_in_progress or self.nav_succeeded is None:
            return

        nav_succeeded = self.nav_succeeded
        self.nav_succeeded = None

        if nav_succeeded:
            self.get_logger().info(f"Reached Station {station_id}")
            self.transition_to(self.get_align_state(station_id))
        else:
            self.get_logger().error(f"Navigation to Station {station_id} failed!")
            self.handle_station_failure(station_id)

    def handle_alignment(self, station_id):
        """
        Align robot to receptacle using Hough circle detection

        NOTE: Alignment control is automatic via offset_callback()
        This function just monitors alignment status and handles timeout
        """
        # offset_callback performs continuous steering; this handler manages
        # timing/failure transitions so robot does not get stuck forever.
        self.start_alignment_if_needed(station_id)

        if self.handle_missing_receptacle(station_id):
            return

        elapsed = time.time() - self.alignment_start_time
        if self.is_alignment_timed_out(elapsed):
            self.get_logger().warn(f"Alignment timeout at Station {station_id}")
            self.alignment_start_time = None
            self.handle_station_failure(station_id)
            return

        if self.is_aligned:
            self.get_logger().info(
                f"Aligned at Station {station_id}! (offset={self.receptacle_offset}px)"
            )
            self.alignment_start_time = None
            self.transition_to(self.get_fire_state(station_id))
            return

        self.log_alignment_progress(elapsed)

    def handle_firing(self, station_id):
        """
        Fire launcher at station with timing delays

        Station A (Static):
            Ball 1 → Wait 7s → Ball 2 → Wait 3s → Ball 3

        Station B (Moving):
            Ball 1 → Ball 2 → Ball 3 (no extra delays)
        """
        # Firing is event-driven by launcher_callback("complete").
        # This handler decides whether to wait, fire next, or finish station.
        self.begin_station_firing_if_needed(station_id)

        if self.balls_fired >= self.balls_per_station:
            self.complete_station_delivery(station_id)
            return

        if self.handle_fire_wait_phase(station_id):
            return

        if not self.is_launcher_ready_to_fire():
            return

        self.fire_next_ball(station_id)

    def handle_complete(self):
        """Mission complete!"""
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("MISSION COMPLETE!")
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f'Station A: {"SUCCESS" if self.stations["A"]["delivered"] else "FAILED"}'
        )
        self.get_logger().info(
            f'Station B: {"SUCCESS" if self.stations["B"]["delivered"] else "FAILED"}'
        )
        self.get_logger().info("=" * 60)
        self.get_logger().info("")

        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        # Keep publishing state, don't shutdown (let operator review)

    def handle_station_failure(self, station_id):
        """Handle delivery failure at a station"""
        self.stations[station_id]["retry_count"] += 1

        retry_count = self.stations[station_id]["retry_count"]
        if retry_count >= self.max_retries:
            self.get_logger().error(
                f"Station {station_id} failed after {self.max_retries} retries. Skipping."
            )
            self.transition_to(self.get_post_failure_state(station_id))
            return

        self.get_logger().warn(
            f"⚠ Retrying Station {station_id} ({retry_count}/{self.max_retries})"
        )
        self.transition_to(self.get_navigate_state(station_id))

    # ========================================
    # HELPER FUNCTIONS
    # ========================================

    def transition_to(self, new_state):
        """Transition to a new state"""
        self.get_logger().info(f"State: {self.state} → {new_state}")
        self.previous_state = self.state
        self.state = new_state

    def send_nav_goal(self, pose_stamped):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.nav_succeeded = None
        self.nav_goal_handle = None
        self.nav_result_future = None

        self.get_logger().info("Sending navigation goal to Nav2...")

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """Handle Nav2 goal acceptance"""
        # Runs asynchronously when Nav2 accepts/rejects the goal.
        goal_handle = future.result()
        self.nav_goal_handle = goal_handle

        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected!")
            self.nav_in_progress = False
            self.nav_succeeded = False
            return

        self.get_logger().info("Nav2 goal accepted!")
        self.nav_result_future = goal_handle.get_result_async()
        self.nav_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """Handle Nav2 goal result"""
        # Normal success path sets nav_succeeded=True and unblocks FSM.
        try:
            result_wrapper = future.result()
            self.nav_succeeded = result_wrapper.status == GoalStatus.STATUS_SUCCEEDED
            if not self.nav_succeeded:
                self.get_logger().warn(
                    f"Nav2 completed with status {result_wrapper.status}"
                )
        except Exception as exc:
            self.get_logger().error(f"Nav2 result callback failed: {exc}")
            self.nav_succeeded = False
        finally:
            self.nav_in_progress = False
            self.nav_goal_handle = None
            self.nav_result_future = None

    def start_alignment_if_needed(self, station_id):
        if self.alignment_start_time is None:
            self.alignment_start_time = time.time()
            self.get_logger().info(f"Aligning at Station {station_id}...")

    def handle_missing_receptacle(self, station_id):
        # Sentinel means detector currently sees no valid receptacle target.
        if self.receptacle_offset != self.receptacle_not_detected:
            return False

        elapsed = time.time() - self.alignment_start_time
        if elapsed > 3.0:
            self.get_logger().warn(f"No circle detected at Station {station_id}")
            self.alignment_start_time = None
            self.handle_station_failure(station_id)
            return True

        if int(elapsed * 10) % 10 == 0:
            self.get_logger().info(f"Waiting for circle detection... ({elapsed:.1f}s)")
        return True

    def is_alignment_timed_out(self, elapsed):
        return elapsed > self.alignment_timeout

    def log_alignment_progress(self, elapsed):
        if int(elapsed * 5) % 5 == 0:
            direction = "LEFT" if self.receptacle_offset > 0 else "RIGHT"
            self.get_logger().info(
                f"Aligning: offset={self.receptacle_offset}px, turning {direction}"
            )

    def begin_station_firing_if_needed(self, station_id):
        if self.current_station_firing != station_id:
            self.current_station_firing = station_id
            self.get_logger().info(f"Starting firing sequence at Station {station_id}")

    def complete_station_delivery(self, station_id):
        # Reset per-station firing variables before switching objective.
        self.get_logger().info(f"Station {station_id} delivery complete!")
        self.stations[station_id]["delivered"] = True
        self.balls_fired = 0
        self.waiting_after_fire = False
        self.current_station_firing = None

        next_state = self.get_next_state_after_delivery(station_id)
        self.transition_to(next_state)

    def get_next_state_after_delivery(self, station_id):
        # If other station is already known, go there immediately.
        # Otherwise return to exploration and wait for detection callbacks.
        other_station = "B" if station_id == "A" else "A"
        if self.stations[other_station]["found"]:
            self.get_logger().info(
                f"Station {other_station} already found. Navigating to {other_station}."
            )
            return self.get_navigate_state(other_station)

        self.get_logger().info(
            f"Resuming exploration to find Station {other_station}..."
        )
        return MissionState.EXPLORE

    def handle_fire_wait_phase(self, station_id):
        if not self.waiting_after_fire:
            return False

        delay_time = self.get_delay_for_ball(station_id, self.balls_fired - 1)
        elapsed = time.time() - self.fire_wait_start_time
        if elapsed >= delay_time:
            self.waiting_after_fire = False
            self.fire_wait_start_time = None
            self.get_logger().info(
                f"Delay complete ({delay_time}s). Ready for next ball."
            )
            return False

        remaining = delay_time - elapsed
        if int(elapsed * 10) % 10 == 0:
            self.get_logger().info(f" Waiting... {remaining:.1f}s remaining")
        return True

    def is_launcher_ready_to_fire(self):
        return self.launcher_ready and self.launcher_status == "idle"

    def fire_next_ball(self, station_id):
        self.get_logger().info(
            f"Firing ball {self.balls_fired + 1}/{self.balls_per_station} at Station {station_id}"
        )
        fire_msg = Bool()
        fire_msg.data = True
        self.fire_launcher_pub.publish(fire_msg)
        self.launcher_ready = False

    def get_delay_for_ball(self, station_id, ball_index):
        station_delays = (
            self.station_a_delays if station_id == "A" else self.station_b_delays
        )
        return station_delays.get(ball_index, 0.0)

    def get_navigate_state(self, station_id):
        return (
            MissionState.NAVIGATE_TO_A
            if station_id == "A"
            else MissionState.NAVIGATE_TO_B
        )

    def get_align_state(self, station_id):
        return MissionState.ALIGN_AT_A if station_id == "A" else MissionState.ALIGN_AT_B

    def get_fire_state(self, station_id):
        return MissionState.FIRE_AT_A if station_id == "A" else MissionState.FIRE_AT_B

    def get_post_failure_state(self, station_id):
        # After max retries, skip failed station and keep mission moving.
        other_station = "B" if station_id == "A" else "A"
        if self.stations[other_station]["found"]:
            return self.get_navigate_state(other_station)
        return MissionState.EXPLORE


def main(args=None):
    rclpy.init(args=args)
    controller = WarehouseMissionController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.get_logger().info("Shutting down mission controller")
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
