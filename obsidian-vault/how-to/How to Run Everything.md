# How to Run Everything

## 1. Docker Development Setup

### First Time
```bash
git clone git@github.com:Chesium/r2auto_nav_CDE2310.git -b jazzystack ~/nav_ws
cd ~/nav_ws
docker compose build
cp docker/ros_network.env.example docker/ros_network.env
```

### Start Container (Ubuntu)
```bash
xhost +local:docker
docker compose -f docker-compose.linux-host.yml up -d
docker compose -f docker-compose.linux-host.yml exec dev bash
```

### Start Container (WSL2)
```bash
docker compose -f docker-compose.yml -f docker-compose.wslg.yml up -d
docker compose -f docker-compose.yml -f docker-compose.wslg.yml exec dev bash
```

### Build Inside Container
```bash
cd ~/nav_ws
colcon build --symlink-install
source install/setup.bash   # or: roset
```

---

## 2. Simulation

### Full Stack (Exploration + Navigation)
```bash
colcon build && roset && ros2 launch g3gzsim nav2full_test_launch.py
```

Then in another terminal, start exploration:
```bash
ros2 service call /exploration/set_enabled std_srvs/srv/SetBool "{data: true}"
```

### Docking Test
```bash
ros2 launch g3gzsim nav2_docking_sim_test.launch.py headless:=False
```

Trigger docking manually:
```bash
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot "{
  use_dock_id: false,
  dock_pose: {header: {frame_id: 'map'}, 
    pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
  dock_type: 'aruco_dock',
  navigate_to_staging_pose: true
}" --feedback
```

### Headless (No GUI)
```bash
ros2 launch g3gzsim nav2_docking_sim_test.launch.py headless:=True use_rviz:=False
```

### Teleop in Simulation
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## 3. Real Hardware

### Step 1: Network Setup
1. Activate phone hotspot
2. Connect RPi and laptop to same hotspot
3. Edit `docker/ros_network.env` with laptop's IP
4. Edit `~/.bashrc` on BOTH RPi and laptop:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_SUPER_CLIENT=TRUE
export ROS_DISCOVERY_SERVER=<LAPTOP_IP>:11811
export ROS_DOMAIN_ID=32
```
5. Source bashrc on both: `source ~/.bashrc`
6. Restart ROS daemon on both: `ros2 daemon stop && ros2 daemon start`

### Step 2: Start Robot
```bash
# On RPi:
rosbu
```

### Step 3: Verify Connectivity
```bash
# On RPi:
ros2 run demo_nodes_cpp talker

# On Laptop (should see /chatter):
ros2 topic list
```

### Step 4: Start SLAM
```bash
# On Laptop:
slam use_rviz:=false
# (alias for: ros2 launch turtlebot3_cartographer cartographer.launch.py)
```

### Step 5: Start Nav2
```bash
ros2 launch g3nav2 g3nav2_bringup_launch.py
```

### Step 6: Start Camera
```bash
# On Laptop (USB camera plugged in):
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:="/dev/video1" \
  -p pixel_format:="mjpeg2rgb" \
  -p image_width:=640 \
  -p image_height:=480 \
  -p camera_name:="usb_cam" \
  -p camera_info_url:="file:///home/g3/camera_ws/src/calibration/usb_cam_calibration.yaml" \
  -r image_raw:=/usb_cam/image_raw \
  -r camera_info:=/usb_cam/camera_info
```

### Step 7: Start Exploration
```bash
ros2 service call /exploration/set_enabled std_srvs/srv/SetBool "{data: true}"
```

---

## 4. Monitoring with Foxglove

```bash
# Start bridge:
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# (or alias: foxnode)
```

Open Foxglove Studio -> Connect `ws://localhost:8765`

Useful panels:
- **Image** on `/aruco_debug/image_raw` - ArUco detection overlay
- **Image** on `/receptacle/annotated` - Hough circle overlay
- **Map** on `/map` - SLAM map
- **Text** on `/mission_state` - Current FSM state
- **Text** on `/launcher_status` - Ball launcher state

---

## 5. Useful Debug Commands

```bash
# See all topics
ros2 topic list

# Check camera is publishing
ros2 topic hz /camera/image_raw

# Check SLAM is working
ros2 topic hz /map

# See current mission state
ros2 topic echo /mission_state

# See launcher status
ros2 topic echo /launcher_status

# Check TF tree
ros2 run tf2_ros tf2_echo map base_link

# Check Nav2 lifecycle states
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server

# List all nodes
ros2 node list

# Manual teleop
ros2 run turtlebot3_teleop teleop_keyboard
```

---

**See also:** [[Package Index]], [[Architecture Overview]]
