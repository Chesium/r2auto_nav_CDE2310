# g3nav2

> **Type:** `ament_cmake` package
> **Purpose:** Nav2 configuration and launch files for real hardware

## Files

```
g3nav2/
├── config/
│   ├── g3_nav2_params.yaml   # Nav2 params tuned for real TurtleBot3
│   └── g3_nav2.rviz          # RViz config for hardware nav
├── launch/
│   └── g3nav2_bringup_launch.py  # Launches Nav2 stack for real robot
├── CMakeLists.txt
└── package.xml
```

## What's Different from g3gzsim?

| Aspect | g3gzsim (sim) | g3nav2 (hardware) |
|--------|---------------|-------------------|
| World | Gazebo simulation | Real environment |
| Sensors | Simulated LiDAR + camera | Physical LiDAR + USB camera |
| Nav2 params | `g3gzsim/nav2/nav2_params.yaml` | `g3nav2/config/g3_nav2_params.yaml` |
| Includes docking? | Yes (docking_server) | Yes (with docking_server) |
| SLAM | Cartographer (built-in) | Cartographer (separate launch) |

## Key Parameter Differences

The hardware params include:
- `collision_monitor > scan > source_timeout: 1.0` (increased from 0.2 for real LiDAR)
- `docking_server` configuration matching simulation
- Controller tuned for physical TurtleBot3 dynamics

## Launch Procedure (Real Robot)

See [[How to Run Everything]] for the full hardware setup procedure.

```bash
# 1. On RPi: start robot drivers
rosbu

# 2. On Laptop: start SLAM
slam use_rviz:=false

# 3. On Laptop: start Nav2
ros2 launch g3nav2 g3nav2_bringup_launch.py

# 4. On Laptop: start exploration
ros2 service call /exploration/set_enabled std_srvs/srv/SetBool "{data: true}"
```

---

**See also:** [[g3gzsim]], [[How to Run Everything]]
