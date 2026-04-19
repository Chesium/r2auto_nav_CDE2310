# High-Level Design

[Home](../README.md)

## Overall System Architecture

### Hardware and Electrical System Architecture

![schematic](assets/g2-report/schematic.png)

<p align="center">Fig: System schematic</p><br>

The hardware platform is based on a TurtleBot3 mobile base extended with onboard compute, camera sensing, launcher actuation, and supporting electrical subsystems. Power and signal routing are distributed across the Raspberry Pi, the OpenCR motor controller, and the attached peripheral devices. This modular arrangement allowed individual subsystems to be developed, tested, and replaced with minimal impact on the rest of the platform.

### Software Architecture

The software stack follows a modular ROS 2 node-based architecture. A central mission controller coordinates overall task sequencing, while specialised worker nodes handle docking, receptacle alignment, exploration, perception, and ball launching. The nodes communicate through ROS 2 topics and services, allowing each subsystem to be developed independently while maintaining clear runtime interfaces.

## Mission Logic Architecture Selection

An initial Behaviour Tree implementation using Groot2 and BehaviorTree.CPP was evaluated during early development. However, the final mission requirements were largely sequential and required limited interruption or dynamic replanning. In practice, the Behaviour Tree approach introduced additional implementation overhead, debugging complexity, and tooling instability relative to the project timeline.

The team therefore migrated to a Python-based finite state machine (FSM). This choice provided deterministic execution, faster implementation, and a simpler debugging workflow, while still matching the fixed sequence of tasks required by the competition.

## Mission Controller Design

### Overview

The mission controller (`mission_controller.py`) is the sole decision-making authority in the system. All other nodes act as specialised workers that publish observations or respond to service requests. The FSM runs at 10 Hz and coordinates the full delivery sequence across both stations. Long-range navigation is handled through exploration and Nav2, while close-range docking is delegated to the visual-servo subsystem based on ArUco pose estimation.

### FSM Graph

State flow: `INIT → EXPLORE → DOCK_AT_X → ALIGN_AT_X → FIRE_AT_X → COMPLETE`, with fallback transitions into `FAILED` for retry handling.

![missionfsm](assets/g2-report/missionfsm.png)

<p align="center">Fig: High-level mission controller FSM</p><br>

### Design Decisions and Rationale

| Decision | Rationale |
| --- | --- |
| Use visual servoing rather than Nav2 for final docking | Final approach is a short-range perception problem. ArUco-based visual servoing is less dependent on global localisation quality and avoids compounding map and pose error close to the target. |
| Run the FSM at 10 Hz rather than as a purely event-driven system | A fixed control tick simplifies timeout handling, keeps state transitions predictable, and reduces the likelihood of callback-order race conditions. |
| Allow Station B to trigger firing autonomously | The moving target at Station B requires a reactive trigger based on LED visibility. This response is more reliable when handled directly by the aligner than by a slower supervisory loop. |
| Expose timing values as ROS 2 parameters | Timing constraints changed during development. Parameterisation allowed the team to update mission behaviour without editing code or rebuilding the software stack. |

### Fault Handling and Robustness Strategy

The mission controller incorporates timeout detection, retry limits, and fallback transitions. If docking or alignment fails repeatedly at a given station, the controller can skip that station and continue with the remaining objectives where possible. This design reduces the risk that a single subsystem failure will terminate the entire mission.
