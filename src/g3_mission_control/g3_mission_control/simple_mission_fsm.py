#!/usr/bin/env python3
"""
simple_mission_fsm.py
=====================
A minimal teaching FSM that coordinates frontier exploration and ArUco docking.

Mission:
    1. Explore the environment using the frontier_explorer node.
    2. As soon as the aruco_dock_node reports a marker detection,
       tell the frontier_explorer to stop and ask the docker to approach.
    3. When the docker reports "done," the mission is complete.

States:
    INIT     - wait until both required services are reachable
    EXPLORE  - enable frontier exploration; watch for a marker detection
    DOCK     - disable exploration; request docking; wait for done
    DONE     - terminal, nothing left to do

External nodes this FSM talks to (unchanged from the existing codebase):
    frontier_explorer:
        Service /exploration/set_enabled  (SetBool)   on/off switch
    aruco_dock_node:
        Service /aruco_dock/dock_to_a     (Trigger)   start approaching marker 42
        Topic   /station_a_pose           (PoseStamped) "I saw marker 42"
        Topic   /aruco_dock/done          (Bool)       "I finished docking"

Design rules enforced here (the whole point of the example):
    - Every subscription callback only WRITES variables. Never transitions.
    - Every transition happens inside a state handler during the 10 Hz tick.
    - Every "on entry" action (calling services) is guarded by a bool so it
      runs exactly once when the state is entered.
    - Service calls are fire-and-forget (call_async) so the tick never blocks.
"""

from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class State(Enum):
    INIT = auto()
    EXPLORE = auto()
    DOCK = auto()
    DONE = auto()


class SimpleMissionFSM(Node):
    """Minimal frontier-explore → ArUco-dock FSM."""

    TICK_PERIOD_SEC = 0.1  # 10 Hz

    def __init__(self) -> None:
        super().__init__("simple_mission_fsm")

        # ─── Current state ────────────────────────────────────────────────
        self._state: State = State.INIT

        # ─── Event flags (written by subscription callbacks only) ────────
        self._marker_seen: bool = False
        self._dock_done: bool = False

        # ─── Entry guards (so "on enter" work runs exactly once) ─────────
        self._explore_entered: bool = False
        self._dock_entered: bool = False

        # ─── Subscribers: callbacks ONLY write flags, never transition ───
        self.create_subscription(
            PoseStamped, "/station_a_pose", self._marker_cb, 10
        )
        self.create_subscription(
            Bool, "/aruco_dock/done", self._dock_done_cb, 10
        )

        # ─── Publisher: debug-only broadcast of current state ────────────
        self._state_pub = self.create_publisher(String, "/mission_state", 10)

        # ─── Service clients for coordinating the other two nodes ───────
        self._explore_client = self.create_client(
            SetBool, "/exploration/set_enabled"
        )
        self._dock_client = self.create_client(
            Trigger, "/aruco_dock/dock_to_a"
        )

        # ─── The single heartbeat that drives everything ────────────────
        self.create_timer(self.TICK_PERIOD_SEC, self._tick)

        self.get_logger().info("simple_mission_fsm started — state: INIT")

    # ═══════════════════════════ CALLBACKS ════════════════════════════════
    # Rule: callbacks ONLY write flags. They never call transition_to.
    # This keeps transitions in exactly one place (the 10 Hz tick).

    def _marker_cb(self, msg: PoseStamped) -> None:
        if not self._marker_seen:
            self.get_logger().info("✓ Marker detected — setting marker_seen flag")
        self._marker_seen = True

    def _dock_done_cb(self, msg: Bool) -> None:
        if msg.data and not self._dock_done:
            self.get_logger().info("✓ Docking complete — setting dock_done flag")
        if msg.data:
            self._dock_done = True

    # ═══════════════════════════ TICK (10 Hz) ═════════════════════════════

    def _tick(self) -> None:
        # Always broadcast current state so you can `ros2 topic echo /mission_state`.
        self._state_pub.publish(String(data=self._state.name))

        if self._state is State.INIT:
            self._handle_init()
        elif self._state is State.EXPLORE:
            self._handle_explore()
        elif self._state is State.DOCK:
            self._handle_dock()
        elif self._state is State.DONE:
            self._handle_done()

    # ═══════════════════════════ STATE HANDLERS ═══════════════════════════

    def _handle_init(self) -> None:
        """Wait for both services to be reachable, then move to EXPLORE."""
        explore_ready = self._explore_client.service_is_ready()
        dock_ready = self._dock_client.service_is_ready()

        if explore_ready and dock_ready:
            self._transition_to(State.EXPLORE)
            return

        missing = []
        if not explore_ready:
            missing.append("/exploration/set_enabled")
        if not dock_ready:
            missing.append("/aruco_dock/dock_to_a")
        self.get_logger().warn(
            f"Waiting for services: {', '.join(missing)}",
            throttle_duration_sec=2.0,
        )

    def _handle_explore(self) -> None:
        """Turn the explorer on (once), then wait for a marker detection."""
        if not self._explore_entered:
            self._set_exploration(True)
            self._explore_entered = True
            self.get_logger().info("EXPLORE: exploration enabled, watching for marker")

        if self._marker_seen:
            self._transition_to(State.DOCK)

    def _handle_dock(self) -> None:
        """Stop the explorer, kick off docking (once), then wait for done."""
        if not self._dock_entered:
            self._set_exploration(False)
            self._request_docking()
            self._dock_entered = True
            self.get_logger().info("DOCK: exploration disabled, dock requested")

        if self._dock_done:
            self._transition_to(State.DONE)

    def _handle_done(self) -> None:
        """Terminal state — nothing to do. Mission complete."""
        # Intentionally empty. The tick still runs but this does nothing.
        return

    # ═══════════════════════════ HELPERS ══════════════════════════════════

    def _transition_to(self, new_state: State) -> None:
        self.get_logger().info(f"FSM: {self._state.name} → {new_state.name}")
        self._state = new_state

    def _set_exploration(self, enabled: bool) -> None:
        """Fire-and-forget toggle of the frontier explorer."""
        request = SetBool.Request()
        request.data = enabled
        future = self._explore_client.call_async(request)
        future.add_done_callback(
            lambda f: self._log_response(f, f"set_exploration({enabled})")
        )

    def _request_docking(self) -> None:
        """Fire-and-forget Trigger call to start docking toward marker 42."""
        future = self._dock_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self._log_response(f, "dock_to_a")
        )

    def _log_response(self, future, label: str) -> None:
        try:
            response = future.result()
            status = "OK" if response.success else "FAIL"
            self.get_logger().info(f"{label} → {status}: {response.message}")
        except Exception as exc:
            self.get_logger().error(f"{label} service error: {exc}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimpleMissionFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
