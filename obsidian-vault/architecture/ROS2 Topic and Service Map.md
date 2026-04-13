# ROS2 Topic and Service Map

## Topics

### Perception Topics

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `/station_a_pose` | PoseStamped | aruco_dock_node | mission_controller, simple_mission_fsm | Robot's map-frame pose when marker 42 seen |
| `/station_b_pose` | PoseStamped | aruco_dock_node | mission_controller | Robot's map-frame pose when marker 67 seen |
| `/detected_dock_pose` | PoseStamped | aruco_dock_pose_publisher | Nav2 docking_server | Marker pose in camera frame for Nav2 docking |
| `/aruco_dock/done` | Bool | aruco_dock_node | simple_mission_fsm | Docking approach complete signal |
| `/aruco_debug/image_raw` | Image | aruco_dock_node | Foxglove | Debug overlay with marker detection |
| `/receptacle/offset` | Int32 | station_a_aligner | mission_controller | Pixel offset from center (9999 = not detected) |
| `/receptacle/aligned` | Bool | station_a_aligner | mission_controller | True when Hough circle centered |
| `/receptacle/tin_ready` | Bool | station_a_aligner | mission_controller | Same as aligned for Station A |
| `/receptacle/annotated` | Image | station_a_aligner | Foxglove | Debug overlay with Hough detection |

### Navigation Topics

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `/cmd_vel` | TwistStamped or Twist | Nav2/aruco_dock/aligner | motor driver | Velocity commands |
| `/map` | OccupancyGrid | SLAM (Cartographer) | frontier_explorer, Nav2 | Occupancy grid map |
| `/scan` | LaserScan | LiDAR driver | SLAM, Nav2 costmaps | 2D laser scans |
| `/odom` | Odometry | motor driver | Nav2, SLAM | Wheel odometry |
| `/exploration/frontiers` | MarkerArray | frontier_explorer | RViz | Visualization of frontier cells |
| `/exploration/current_goal` | PoseStamped | frontier_explorer | RViz | Current exploration target |

### State/Status Topics

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `/mission_state` | String | mission_controller / simple_mission_fsm | monitoring | Current FSM state name |
| `/launcher_status` | String | ball_launcher_node | mission_controller | "idle"/"firing"/"complete"/"error" |
| `/exploration_complete` | Bool | frontier_explorer | mission_controller | All stations found signal |

### Camera Topics

| Topic | Type | Source | Notes |
|-------|------|--------|-------|
| `/usb_cam/image_raw` | Image | USB camera (hardware) | Used by aruco_dock_node |
| `/usb_cam/camera_info` | CameraInfo | USB camera (hardware) | Camera intrinsics |
| `/camera/image_raw` | Image | Gazebo camera (sim) | Used by aruco_dock_pose_publisher |
| `/camera/camera_info` | CameraInfo | Gazebo camera (sim) | Camera intrinsics |
| `/camera/image_raw/compressed` | CompressedImage | RPi camera | Used by station_a_aligner |

---

## Services

| Service | Type | Server | Client | Purpose |
|---------|------|--------|--------|---------|
| `/fire_launcher` | Trigger | ball_launcher_node | mission_controller | Request one ball shot |
| `/stop_launcher` | Trigger | ball_launcher_node | operator | Emergency stop motor |
| `/aruco_dock/dock_to_a` | Trigger | aruco_dock_node | simple_mission_fsm | Start docking to marker 42 |
| `/aruco_dock/dock_to_b` | Trigger | aruco_dock_node | mission_controller | Start docking to marker 67 |
| `/aruco_dock/scan` | Trigger | aruco_dock_node | operator | Return to passive scanning |
| `/exploration/set_enabled` | SetBool | frontier_explorer | simple_mission_fsm | Enable/disable exploration |

---

## Actions

| Action | Type | Server | Client | Purpose |
|--------|------|--------|--------|---------|
| `/navigate_to_pose` | NavigateToPose | Nav2 BT navigator | mission_controller, frontier_explorer | Point-to-point navigation |
| `/compute_path_to_pose` | ComputePathToPose | Nav2 planner | frontier_explorer | Path feasibility check |
| `/dock_robot` | DockRobot | Nav2 docking_server | operator (manual test) | Nav2 docking action |

---

## QoS Notes

- `/station_a_pose` and `/station_b_pose` use **TRANSIENT_LOCAL** (latched) QoS
  - Subscribers MUST match this or messages silently drop
  - `simple_mission_fsm` correctly uses latched QoS for `/station_a_pose`
- `/map` uses TRANSIENT_LOCAL from SLAM
- Camera topics from Gazebo bridge use default RELIABLE
- RPi compressed camera uses BEST_EFFORT (matched in station_a_aligner)
