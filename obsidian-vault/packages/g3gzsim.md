# g3gzsim

> **Type:** `ament_cmake` package
> **Purpose:** Gazebo simulation worlds, robot models, launch files, and Nav2 parameter configs

## Directory Structure

```
g3gzsim/
├── launch/
│   ├── sim_only_launch.py              # Gazebo only, no Nav2
│   ├── nav2full_test_launch.py         # Full sim: Gazebo + Nav2 + SLAM + exploration
│   └── nav2_docking_sim_test.launch.py # Docking test: Gazebo + Nav2 + ArUco detector
├── worlds/
│   ├── square_room.sdf     # Simple 4-wall room with ArUco marker on east wall
│   ├── warehouse_world.sdf # More complex warehouse layout
│   └── tutorial_world.sdf  # Gazebo tutorial world
├── models/
│   ├── aruco_marker_42/    # ArUco marker ID 42 (Station A)
│   ├── aruco_marker_67/    # ArUco marker ID 67 (Station B)
│   └── turtlebot3_world/   # TurtleBot3 world model
├── urdf/
│   └── tb.sdf.xacro        # TurtleBot3 robot model with camera
├── nav2/
│   └── nav2_params.yaml    # Nav2 parameters for simulation
├── rviz/
│   └── nav2_default_view.rviz # RViz display config
└── CMakeLists.txt
```

## Launch Files

### `sim_only_launch.py`
Minimal launch: just Gazebo + robot + camera bridges. No Nav2, no SLAM.
Good for: Testing camera/sensor output, world design.

### `nav2full_test_launch.py`  
Full simulation stack:
- Gazebo with square_room world
- TurtleBot3 with camera
- Nav2 (SLAM mode, no static map)
- Camera bridges (image + camera_info)
- Frontier exploration node
- Simple mission FSM

This is the main simulation launch for full-stack testing.

### `nav2_docking_sim_test.launch.py`
Docking-specific test:
- Gazebo with square_room (includes ArUco marker 42 on east wall)
- TurtleBot3 spawned at (-2, 0) facing east toward marker
- Nav2 with SLAM and **docking_server** enabled
- Camera bridges
- `aruco_dock_pose_publisher` node

Manual test command after launch:
```bash
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot "{
  use_dock_id: false,
  dock_pose: {header: {frame_id: 'map'}, 
    pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
  dock_type: 'aruco_dock',
  navigate_to_staging_pose: true
}" --feedback
```

## ArUco Marker Models

### aruco_marker_42
- A 0.18m square SDF model with marker 42 texture
- Placed on the east wall of `square_room.sdf`
- Used for docking tests and Station A simulation

### aruco_marker_67  
- Same structure, marker 67 texture
- Station B marker

## Nav2 Parameters (`nav2_params.yaml`)

Key customizations vs default TurtleBot3 params:
- Includes `docking_server` and `loopback_simulator` configuration
- `SimpleNonChargingDock` plugin for ArUco docking
- `use_external_detection_pose: true` - relies on external pose publisher
- Docking approach parameters: `staging_x_offset: -0.5`, `external_detection_timeout: 3.0`

---

**See also:** [[How to Run Everything]], [[Architecture Overview]]
