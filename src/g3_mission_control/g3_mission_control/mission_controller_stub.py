#!/usr/bin/env python3
"""
Stub mission controller for simulation testing.

Tests aruco_dock_node + frontier exploration integration:
  EXPLORE → (frontier explorer drives robot) → marker detected → DOCK_AT_X
          → (1s stub delay) → stop exploration → back to EXPLORE → COMPLETE

No alignment, no firing.  Run alongside:
  - aruco_dock_node         (g3_visual_servo)
  - frontier_explorer node  (g3g_frontier_exploration)
  - Gazebo warehouse_world with markers 42 and 67
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool


class StubState:
    EXPLORE   = "EXPLORE"
    DOCK_AT_A = "DOCK_AT_A"
    DOCK_AT_B = "DOCK_AT_B"
    COMPLETE  = "COMPLETE"


class MissionControllerStub(Node):

    def __init__(self):
        super().__init__("mission_controller_stub")

        self.state = StubState.EXPLORE

        self.stations = {
            "A": {"found": False, "done": False},
            "B": {"found": False, "done": False},
        }

        self.aruco_dock_done  = False
        self._dock_start_time = None

        # Latched QoS — must match aruco_dock_node's publisher to receive poses published before startup
        _latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Subscribers
        self.create_subscription(PoseStamped, "/station_a_pose", self._station_a_cb, _latched)
        self.create_subscription(PoseStamped, "/station_b_pose", self._station_b_cb, _latched)
        self.create_subscription(Bool,        "/aruco_dock/done", self._dock_done_cb, 10)

        # Publisher
        self._state_pub = self.create_publisher(String, "/mission_state", 10)

        # Service clients
        self._dock_to_a      = self.create_client(Trigger, "/aruco_dock/dock_to_a")
        self._dock_to_b      = self.create_client(Trigger, "/aruco_dock/dock_to_b")
        self._dock_scan      = self.create_client(Trigger, "/aruco_dock/scan")
        self._exploration_en = self.create_client(SetBool, "/exploration/set_enabled")

        self.create_timer(0.1, self._tick)

        # Start exploration once the service is available (checked every second)
        self._startup_timer = self.create_timer(1.0, self._try_start_exploration)

        self.get_logger().info("Stub mission controller running — waiting for markers 42 / 67")

    # ── exploration control ───────────────────────────────────────────────────

    def _try_start_exploration(self):
        """Called every second until exploration is successfully enabled."""
        if self.state != StubState.EXPLORE:
            self._startup_timer.cancel()
            return
        if not self._exploration_en.wait_for_service(timeout_sec=0.0):
            self.get_logger().info("Waiting for /exploration/set_enabled ...")
            return
        self._set_exploration(True)
        self._startup_timer.cancel()

    def _set_exploration(self, enable: bool):
        if not self._exploration_en.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn("Exploration service not available")
            return
        req = SetBool.Request()
        req.data = enable
        future = self._exploration_en.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Exploration {'enabled' if enable else 'disabled'}: {f.result().message}"
            )
        )

    # ── callbacks ────────────────────────────────────────────────────────────

    def _station_a_cb(self, _msg):
        if not self.stations["A"]["found"]:
            self.stations["A"]["found"] = True
            self.get_logger().info("Station A (marker 42) detected")
            if self.state == StubState.EXPLORE:
                self._set_exploration(False)
                self.aruco_dock_done = False
                if self._dock_to_a.wait_for_service(timeout_sec=0.0):
                    self._dock_to_a.call_async(Trigger.Request())
                self._transition(StubState.DOCK_AT_A)

    def _station_b_cb(self, _msg):
        if not self.stations["B"]["found"]:
            self.stations["B"]["found"] = True
            self.get_logger().info("Station B (marker 67) detected")
            if self.state == StubState.EXPLORE:
                self._set_exploration(False)
                self.aruco_dock_done = False
                if self._dock_to_b.wait_for_service(timeout_sec=0.0):
                    self._dock_to_b.call_async(Trigger.Request())
                self._transition(StubState.DOCK_AT_B)

    def _dock_done_cb(self, msg):
        if msg.data:
            self.aruco_dock_done = True

    # ── FSM tick ─────────────────────────────────────────────────────────────

    def _tick(self):
        s = String(); s.data = self.state
        self._state_pub.publish(s)

        if   self.state == StubState.EXPLORE:  self._handle_explore()
        elif self.state == StubState.DOCK_AT_A: self._handle_dock("A")
        elif self.state == StubState.DOCK_AT_B: self._handle_dock("B")
        elif self.state == StubState.COMPLETE:  self._handle_complete()

    def _handle_explore(self):
        for sid, data in self.stations.items():
            if data["found"] and not data["done"]:
                self._set_exploration(False)
                self.aruco_dock_done = False
                self._transition(StubState.DOCK_AT_A if sid == "A" else StubState.DOCK_AT_B)
                return

        if all(d["done"] for d in self.stations.values()):
            self._set_exploration(False)
            self._transition(StubState.COMPLETE)

    def _handle_dock(self, sid):
        if not self.aruco_dock_done:
            return

        if self._dock_start_time is None:
            self._dock_start_time = time.time()
            self.get_logger().info(f"Dock at Station {sid} complete — stub 1s delay...")
            return

        if time.time() - self._dock_start_time < 1.0:
            return

        self._dock_start_time      = None
        self.aruco_dock_done       = False
        self.stations[sid]["done"] = True
        self.get_logger().info(f"Station {sid} finished")

        other = "B" if sid == "A" else "A"
        if self.stations[other]["done"]:
            self._transition(StubState.COMPLETE)
        elif self.stations[other]["found"]:
            client = self._dock_to_a if other == "A" else self._dock_to_b
            if client.wait_for_service(timeout_sec=0.0):
                client.call_async(Trigger.Request())
            self._transition(StubState.DOCK_AT_A if other == "A" else StubState.DOCK_AT_B)
        else:
            if self._dock_scan.wait_for_service(timeout_sec=0.0):
                self._dock_scan.call_async(Trigger.Request())
            self._set_exploration(True)
            self._transition(StubState.EXPLORE)

    def _handle_complete(self):
        self.get_logger().info("=" * 50)
        self.get_logger().info("STUB MISSION COMPLETE — both stations docked")
        self.get_logger().info("=" * 50)

    def _transition(self, new_state):
        self.get_logger().info(f"  {self.state} → {new_state}")
        self.state = new_state


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
