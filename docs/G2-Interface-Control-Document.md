# Interface Control Document

[Home](../README.md)

## Mechanical Interfaces

The mechanical subsystem combines the TurtleBot3 platform with custom launcher, camera, and support structures. The figures below show the assembled upper and lower layers of the robot, while the table summarises the principal mechanical connections.

![assembly1](assets/g2-report/assembly1.png)

<p align="center">Fig: TurtleBot assembly, Layer 4</p><br>

![assembly2](assets/g2-report/assembly2.png)

<p align="center">Fig: TurtleBot assembly, Layers 1 and 2</p><br>

| Component 1 | Component 2 | Connection via |
| --- | --- | --- |
| TurtleBot Layer 4 | USB Camera Mount | 2x M2 nuts and bolts |
| TurtleBot Layer 4 | RPi Camera Mount | 2x M4 nuts and bolts |
| TurtleBot Layer 4 | Pipe Mounts | 4x M4 nuts and bolts |
| TurtleBot Layer 2 | UART Servo Mount | 2x M2 nuts and bolts |
| TurtleBot Layer 3 | Servo Encoder | 2x M3 nuts and bolts |
| TurtleBot Layer 1 | Launcher Mount | 4x M4 nuts |
| Launcher Mount | Launcher Housing | 4x M3 nuts |
| Launcher Housing | Launcher Striker | 2x dowels via friction fit |
| Launcher Housing | Launcher-Pipe Pins | Friction fit |
| Launcher-Pipe Pins | Pipe | 2x M2 self-tapping screws |
| Pipe | Pipe Cover | Superglue |
| UART Servo Mount | UART Servo Motor | 4x M3 nuts and bolts |
| UART Servo Motor | Cam | Servo disc fastened with 8x approximately M1.8 screws |
| Cam | Shaft and Secondary Cam | Friction fit |
| USB Camera Mount | USB Camera | 2x M2 nuts, bolts, and washers |
| RPi Camera Mount | RPi Camera | 2x M2 nuts, bolts, and washers |

## Electrical Interfaces

The electrical subsystem connects the launcher actuator, cameras, and LiDAR to the Raspberry Pi and supporting power rails. The table below summarises the primary electrical interfaces.

| Component | Physical Connection | Connector on RPi | Power Source | Voltage | Communication Protocol | Data Rate | Device Node | Purpose |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| JOHO UART Servo (Ball Launcher) | USB-to-UART adapter into Raspberry Pi USB-A | USB 2.0 | RPi 5V GPIO (`Pin 2/4`) + GND (`Pin 6`) | 5V DC | UART 8N1 via USB-to-TTL adapter | `115200` bps | `/dev/servo` | Ball launcher motor control in DC mode |
| IMX219 Camera Module | USB cable | USB 3.0 | USB bus power | 5V DC | USB 2.0 UVC | `480 Mbps` | `/dev/video1` | ArUco detection for visual-servo docking |
| RPi Camera (CSI) | 15-pin FFC ribbon cable | CSI camera port | CSI port | 3.3V DC | MIPI CSI-2, 2 lanes | `320x240` BGR888 | `libcamera` | Receptacle alignment using Hough circle detection |
| LDS-02 LiDAR | 4-pin JST to USB2LDS to USB-A | USB 2.0 | USB bus power | 5V DC | UART 8N1 over USB (CDC-ACM) | `230400` bps | `/dev/ttyUSB0` | 2D scan input for SLAM and navigation |

## Software Interfaces

### Mission Controller ↔ Frontier Explorer

The frontier explorer is a standalone node built on `BasicNavigator` from `nav2_simple_commander`. It manages its own navigation goal lifecycle internally, while the mission controller enables or disables exploration as required by the high-level mission sequence.

| Interface | Type | Nodes / Direction | Trigger | Data Type | Description |
| --- | --- | --- | --- | --- | --- |
| `/exploration/set_enabled` | Service | `mission_controller → frontier_explorer` | `INIT → EXPLORE`, `EXPLORE → DOCK`, station completion | `SetBool` | Enables or disables the planning loop. Enabling clears cluster blacklists and failure counters; disabling cancels the active Nav2 goal and halts exploration. |
| `/exploration_complete` | Topic | `frontier_explorer → mission_controller` | No valid frontiers for repeated planning cycles, or encapsulation completion | `Bool` | Signals that exploration is complete. Published with transient-local QoS. |
| `/exploration/frontiers` | Topic | `frontier_explorer → RViz` | Every planning tick | `MarkerArray` | Visualisation of current frontier cells, blacklisted regions, and active goal. |
| `/exploration/current_goal` | Topic | `frontier_explorer → RViz` | On goal dispatch | `PoseStamped` | Visualisation of the active exploration goal. |
| `/navigate_to_pose` | Action | `frontier_explorer → Nav2 BT Navigator` | Best frontier selected | `NavigateToPose` | Executes the active exploration goal with timeout and retry handling. |
| `/compute_path_to_pose` | Action | `frontier_explorer → Nav2 Planner Server` | Candidate validation | `ComputePathToPose` | Verifies path feasibility before goal commitment. |

### Mission Controller ↔ Visual Servo

`simple_aruco_dock` operates in two modes: passive scanning during exploration and active docking during the final approach to a selected station.

| Interface | Type | Nodes / Direction | Trigger | Data Type | Description |
| --- | --- | --- | --- | --- | --- |
| `/station_a_pose` | Topic | `simple_aruco_dock → mission_controller` | Station A marker detected during passive scan | `PoseStamped` | Publishes the detected station pose so the FSM can mark Station A as found. |
| `/station_b_pose` | Topic | `simple_aruco_dock → mission_controller` | Station B marker detected during passive scan | `PoseStamped` | Same as above for Station B. |
| `/aruco_dock/dock_to_a` | Service | `mission_controller → simple_aruco_dock` | Entering `DOCK_AT_A` | `Trigger` | Switches the docking node into active approach mode for marker 42. |
| `/aruco_dock/dock_to_b` | Service | `mission_controller → simple_aruco_dock` | Entering `DOCK_AT_B` | `Trigger` | Switches the docking node into active approach mode for marker 67. |
| `/cmd_vel` | Topic | `simple_aruco_dock → motor driver` | Every camera frame during active docking | `TwistStamped` | PI-control output for final approach. |
| `/aruco_dock/done` | Topic | `simple_aruco_dock → mission_controller` | Docking criteria satisfied | `Bool` | Signals completion of the docking manoeuvre. |
| `/aruco_dock/scan` | Service | `mission_controller → simple_aruco_dock` | Leaving docking mode | `Trigger` | Returns the docking node to passive scanning and resets internal reporting flags. |
| `/aruco_dock/marker_visible` | Topic | `simple_aruco_dock → mission_controller` | Every camera frame | `Bool` | Indicates whether a known marker is visible in the current image. |
| `/aruco_dock/marker_distance` | Topic | `simple_aruco_dock → mission_controller` | Every detected frame | `Float32` | Publishes the raw `solvePnP` distance estimate to the target marker. |

### Mission Controller ↔ Station A Aligner

`station_a_aligner` subscribes to the compressed CSI camera stream, performs Hough-based circle detection, and commands linear alignment motion while the robot is in `ALIGN_AT_A`.

| Interface | Type | Nodes / Direction | Trigger | Data Type | Description |
| --- | --- | --- | --- | --- | --- |
| `/aligner_a/set_enabled` | Service | `mission_controller → station_a_aligner` | Entering or leaving `ALIGN_AT_A` | `SetBool` | Enables or disables image processing and command output. |
| `/receptacle/offset` | Topic | `station_a_aligner → mission_controller` | Every camera frame | `Int32` | Publishes `Y`-axis pixel offset; `9999` indicates no circle detected. |
| `/receptacle/tin_ready` | Topic | `station_a_aligner → mission_controller` | Every camera frame | `Bool` | Continuous alignment status signal. |
| `/receptacle/notify_aligned` | Service | `station_a_aligner → mission_controller` | Required number of consecutive aligned frames | `Trigger` | One-shot alignment notification to the FSM. |
| `/cmd_vel` | Topic | `station_a_aligner → motor driver` | Every enabled frame | `Twist` | Linear-only alignment command. |
| `/receptacle/annotated` | Topic | `station_a_aligner → Foxglove` | Every camera frame | `Image` | Annotated debug view with Hough detections and alignment overlay. |
| `/camera/image_raw/compressed` | Topic | `csi_cam → station_a_aligner` | Camera publish rate | `CompressedImage` | Input image stream from the rotated CSI camera. |

### Mission Controller ↔ Station B Aligner

`station_b_aligner` handles both geometric alignment and autonomous firing for the moving receptacle at Station B.

| Interface | Type | Nodes / Direction | Trigger | Data Type | Description |
| --- | --- | --- | --- | --- | --- |
| `/receptacle/b_done` | Topic | `station_b_aligner → mission_controller` | All required shots completed | `Bool` | Signals completion of Station B firing. |
| `/fire_launcher` | Service | `station_b_aligner → ball_launcher_node` | Blue LED rising edge detected | `Trigger` | Direct firing call from the Station B aligner. |
| `/cmd_vel` | Topic | `station_b_aligner → motor driver` | Every camera frame during geometric alignment | `Twist` | Linear alignment command; held at zero during the LED-wait phase. |
| `/receptacle/offset` | Topic | `station_b_aligner → mission_controller` | Every camera frame | `Int32` | Publishes the same offset contract used by Station A. |
| `/receptacle/tin_ready` | Topic | `station_b_aligner → mission_controller` | LED-detection phase | `Bool` | Indicates that the blue LED is currently detected within the target region. |
| `/receptacle/annotated` | Topic | `station_b_aligner → Foxglove` | Every camera frame | `Image` | Debug overlay showing phase, LED detection state, and ball count. |
| `/camera/image_raw/compressed` | Topic | `csi_cam → station_b_aligner` | Camera publish rate | `CompressedImage` | Shared input stream from the CSI camera. |

### Mission Controller ↔ Ball Launcher

The launcher node runs on the Raspberry Pi and exposes a simple service-and-status interface for firing control.

| Interface | Type | Nodes / Direction | Trigger | Data Type | Description |
| --- | --- | --- | --- | --- | --- |
| `/fire_launcher` | Service | `mission_controller → ball_launcher_node` | Station A firing sequence | `Trigger` | Starts a timed launcher cycle if the launcher is idle and available. |
| `/launcher_status` | Topic | `ball_launcher_node → mission_controller` | Continuous timer | `String` | Reports `idle`, `firing`, `complete`, or `error`. |
| `/stop_launcher` | Service | `ball_launcher_node` | Manual emergency stop | `Trigger` | Interrupts the firing thread and returns the launcher to a safe idle state. |

## Nodes Launch Location

The final deployment distributed nodes between the Raspberry Pi and the operator laptop as follows:

| Platform | Nodes | Peripherals |
| --- | --- | --- |
| Raspberry Pi | `turtlebot3_bringup` | LDS-02 LiDAR, Dynamixel motors |
| Raspberry Pi | `usb_cam` | IMX219 USB camera |
| Raspberry Pi | `csi_cam` | RPi CSI camera |
| Raspberry Pi | `ball_launcher_node` | JOHO servo in launcher mechanism |
| Laptop (Docker) | Cartographer, Nav2, `frontier_explorer`, `post_exploration_traverser`, `simple_aruco_dock`, `station_a_aligner`, `station_b_aligner`, `mission_controller`, `foxglove_bridge` | None |
