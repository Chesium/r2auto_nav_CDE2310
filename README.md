## Portable Docker Development Environment

This repo now includes a portable Docker setup built on top of the official `osrf/ros:jazzy-desktop` image (Ubuntu 24.04 / Noble) with:

- `ros-jazzy-desktop`
- `ros-dev-tools`
- Gazebo Harmonic (`gz-harmonic`)
- Nav2 + Cartographer dependencies
- Foxglove bridge
- TurtleBot3 built from source in `~/turtlebot3_ws`

### 1. Clone the repo to `~/nav_ws`

```bash
git clone git@github.com:Chesium/r2auto_nav_CDE2310.git -b jazzystack ~/nav_ws
cd ~/nav_ws
```

### 2. Build the image

Default Linux user mapping is `1000:1000`, which is usually correct for Ubuntu and WSL2. If your user/group IDs differ, export them before building:

```bash
export DEV_UID=$(id -u)
export DEV_GID=$(id -g)
docker compose build
```

### 3. Create the editable ROS network config

The container will auto-create `docker/ros_network.env` from the example on first start, or you can do it yourself:

```bash
cp docker/ros_network.env.example docker/ros_network.env
```

Edit [docker/ros_network.env.example](docker/ros_network.env.example) as a reference, and keep your machine-specific values in `docker/ros_network.env`.

The values intended for quick changes are:

- `ROS_DISCOVERY_SERVER`
- `ROS_DOMAIN_ID`
- `ROS_DISCOVERY_INTERFACE`
- `ROS_DISCOVERY_PORT`

By default, `docker/ros_network.env` now leaves `ROS_DISCOVERY_SERVER` unset so terminals inside the same dev container use normal ROS 2 peer discovery immediately. Only set `ROS_DISCOVERY_SERVER` when you are intentionally using a Fast DDS discovery server for laptop/robot communication.

### 4. Start the dev container

Native Ubuntu 22.04 / 24.04:

```bash
xhost +local:docker
docker compose -f docker-compose.linux-host.yml up -d
docker compose -f docker-compose.linux-host.yml exec dev bash
```

WSL2 with Ubuntu 22.04 / 24.04 and WSLg:

```bash
docker compose -f docker-compose.yml -f docker-compose.wslg.yml up -d
docker compose -f docker-compose.yml -f docker-compose.wslg.yml exec dev bash
```

Inside the container, the shell auto-sources:

- `/opt/ros/jazzy/setup.bash`
- `~/turtlebot3_ws/install/setup.bash`
- `~/nav_ws/install/setup.bash` when it exists

and defines these commands:

- `roset`
- `talker`
- `topics`
- `foxnode`
- `slam`
- `rteleop`
- `discovery`

For a quick local sanity check inside the container, open two shells and run:

```bash
talker
```

```bash
ros2 node list
```

You should see `/talker` without needing `discovery`.

### 5. Build this workspace inside the container

```bash
cd ~/nav_ws
colcon build --symlink-install
roset
```

### VS Code Dev Container

This repo also includes [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json), so in VS Code you can:

```bash
code ~/nav_ws
```

then run `Dev Containers: Reopen in Container`.

By default the devcontainer uses [docker-compose.yml](docker-compose.yml), which is the more portable choice across Ubuntu and WSL2. If you are on native Ubuntu and want host networking from inside VS Code for robot communication, change the `dockerComposeFile` entry in [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json) to `../docker-compose.linux-host.yml`.

### Notes on portability

- Ubuntu 22.04 and WSL2 Ubuntu 22.04 are fine as *hosts* because Docker isolates the userland; the container itself still runs Ubuntu 24.04, which is the supported binary platform for ROS 2 Jazzy.
- Native Ubuntu is the best option when you need ROS 2 discovery and real robot networking to behave like a normal Linux machine.
- WSL2 is excellent for editing and local builds, and usually fine for Gazebo/RViz when WSLg is working well, but real LAN robotics workflows are a little more sensitive to Docker/WSL networking.
- If you mainly want simulation, WSL2 is usually convenient enough. If you need to talk to the robot over the same network as the laptop, native Ubuntu will be more predictable.
- If Foxglove is all you need from outside the container, the base compose file publishes port `8765`.

## Run Simulation

```bash
colcon build
source install/setup.bash
ros2 launch g3gzsim nav2full_test_launch.py
```

in one line: (assume you have `alias roset="source install/setup.bash"` in `~/.bashrc`)

```bash
colcon build && roset && ros2 launch g3gzsim nav2full_test_launch.py | tee test.log
```

---

then, to activate the explorer, in a new terminal, run:

```bash
ros2 service call /exploration/set_enabled std_srvs/srv/SetBool "{data: true}"
```

## Update/Connect to Hot-spot and establish ROS 2 Connection

- `sudo nmtui`: Activate the connection > Find the hot-spot, connect it (this time is without password and the program will stuck)
- wait a few seconds, restart the RPi, `ssh g3@rpi` (via Tailscale), then run `sudo nmtui` again : Edit a Connection > edit the password and disable automatic connect.
- use `sudo nmcli c up [NAME]` to activate the connection (replace `[NAME]` with the hot-spot name), you may have to wait a bit, you can also run `ssh g3@rpi` in a new laptop terminal to monitor its progress
- after connection, use `ip a` to find its IP address under the hot-spot connection:

```
3: wlan0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UP group default qlen 1000
    link/ether d8:3a:dd:23:92:0f brd ff:ff:ff:ff:ff:ff
    inet 172.20.10.9/28 brd 172.20.10.15 scope global dynamic noprefixroute wlan0
       valid_lft 2627sec preferred_lft 2627sec
    inet6 fe80::7126:5b15:305f:2030/64 scope link noprefixroute 
       valid_lft forever preferred_lft forever
```

- example output above: ip should be `172.20.10.9`
- connect the same hot-spot on your laptop (if no ethernet and we are using phone hot-spot) and verify we can ssh into the rpi via the ip we just obtained `ssh g3@172.20.10.9` (type `yes` if necessary)
- **Important for ROS 2 Connection:** On Laptop, first edit `docker/ros_network.env` so `ROS_DISCOVERY_SERVER` and `ROS_DISCOVERY_INTERFACE` both use the laptop's actual reachable IP, then run `discovery` in a terminal **before doing anything about ROS 2** after reboot or connect to a new hot-spot
- **Important for ROS 2 Connection:** edit the `~/.bashrc` on **Both RPI and the laptop**: add or modify the `ROS_DISCOVERY_SERVER` IP to be the same as the **laptop** IP (obtain also by running `ip a`)

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_SUPER_CLIENT=TRUE
export ROS_DISCOVERY_SERVER=10.42.0.1:11811 # <- change this to *LAPTOP* IP
export ROS_DOMAIN_ID=32
```

- `source ~/.bashrc` on both rpi and laptop
- restart ROS daemon **on both RPi and laptop** by doing
  - `ros2 daemon stop`
  - `ros2 daemon start`
- verify the ros 2 connection by
  - on rpi, running `ros2 run demo_nodes_cpp talker`
  - on laptop, running `ros2 topic list` (maybe twice for the change to be updated) and verify you see the `/chatter` topic being published by the RPI talker node

## Setup Foxglove Dashboard

- ensure you have downloaded it on your laptop
- follow [this](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2#foxglove-websocket) to install the packages and activate the foxglove node by running:
  - `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
    - you can set an alias for this ("foxnode")
- open foxglove, connect via `ws://localhost:8765`

## Setup Camera

- after verifying that the ROS 2 Connection is fine, run

```bash
sudo bash -ic 'source /home/g3/.bashrc && source /home/g3/camera_ws/install/setup.bash && ros2 run camera_ros camera_node --ros-args -p format:=BGR888 -p camera:=0 -p role:=viewfinder -p height:=240 -p width:=320'
```

- it's fine if you see `unable to open camera calibration file` Error.
- in Foxglove, add a *image* panel and open the `/camera/image_raw/compressed` topic to monitor the image flow
- force kill the node by `sudo pkill -9 camera_node`

## Setup Nav 2

- after verifying that the ROS 2 Connection is fine, run `rosbu` on RPi to activate the robot_state / lidar / motor publisher stuff.
  - verify on laptop by `ros2 topic list`
  - verify you can teleoperate it by `ros2 run turtlebot3_teleop teleop_keyboard` ([reference](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#basic-operation))
- build `g3nav2` and run `ros2 launch g3nav2 g3nav2_bringup_launch.py`
  - this line is equal to the following which are packaged into our launch file
  - currently the parameter is the same as the default `burger.yaml` except for:
  - `collision_monitor > scan > source_timeout`: changed from `0.2` to `1.0` (line 414)

- run slam on your laptop **without RViz UI** (since we will open an another one when activating nav 2) by `slam use_rviz:=false`
  - assuming you have set `alias slam="ros2 launch turtlebot3_cartographer cartographer.launch.py"`
- run the nav2 launch file: `ros2 launch turtlebot3_navigation2 navigation2.launch.py`
  - after you see the costmap overlay (blue/purple/red stuff), do a round of `2D pose Estimate` (not sure if this is compulsory)

## Misc stuff

ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

use slam instead of fixed map server

ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False slam:=True map:='""'

ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False slam:=True map:='""' x_pose:=0 y_pose:=0

empirical suggestion: close rviz2 first, then gazebo

ref: https://robotics.stackexchange.com/questions/87696/provide-empty-string-as-roslaunch-argument-substitution

package flow of the parameter `map`:
- `nav2_bringup/launch/tb3_simulation_launch.py` : `"map" - map_yaml_file`
- `nav2_bringup/launch/bringup_launch.py` : `"map" - map_yaml_file`
- `nav2_bringup/launch/localization_launch.py` : `"map" - map_yaml_file`
  - `nav2_map_server@map_server` : `"yaml_filename"`
  - `[Composable] nav2_map_server::MapServer` : `"yaml_filename"`

next step: use customized world file


there's also a `pose` parameter in the top launch file

```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 g3description
ros2 pkg create --build-type ament_python --license Apache-2.0 g3gzsim
ros2 pkg create --build-type ament_python --license Apache-2.0 g3navigation
ros2 pkg create --build-type ament_python --license Apache-2.0 g3bringup
ros2 pkg create --build-type ament_python --license Apache-2.0 g3exploration
```

recommended file tree by Chat-GPT:

```
your_ws/
└── src/
    ├── your_robot_description/
    │   ├── package.xml
    │   ├── CMakeLists.txt / setup.py
    │   ├── urdf/
    │   │   ├── robot.urdf.xacro
    │   │   ├── robot_core.xacro
    │   │   ├── sensors.xacro
    │   │   └── ros2_control.xacro
    │   ├── meshes/
    │   │   ├── base.dae
    │   │   ├── wheel.dae
    │   │   └── ...
    │   ├── materials/              # optional
    │   ├── rviz/                   # optional if model-only RViz config
    │   │   └── view_robot.rviz
    │   └── launch/
    │       └── display.launch.py   # robot_state_publisher + joint_state_publisher_gui
    │
    ├── your_robot_gazebo/
    │   ├── package.xml
    │   ├── CMakeLists.txt / setup.py
    │   ├── worlds/
    │   │   ├── lab.sdf
    │   │   ├── empty.sdf
    │   │   └── ...
    │   ├── sdf/
    │   │   ├── robot_gz.sdf.xacro
    │   │   └── sensors_gz.sdf.xacro
    │   ├── models/                 # optional for standalone Gazebo models
    │   ├── launch/
    │   │   ├── sim.launch.py
    │   │   ├── spawn_robot.launch.py
    │   │   └── world_only.launch.py
    │   └── config/
    │       └── gazebo_bridge.yaml  # if using ros_gz bridge
    │
    ├── your_robot_navigation/
    │   ├── package.xml
    │   ├── CMakeLists.txt / setup.py
    │   ├── launch/
    │   │   ├── nav_sim.launch.py
    │   │   ├── nav_real.launch.py
    │   │   ├── localization.launch.py
    │   │   ├── slam.launch.py
    │   │   └── rviz.launch.py
    │   ├── params/
    │   │   ├── nav2_common.yaml
    │   │   ├── nav2_sim.yaml
    │   │   ├── nav2_real.yaml
    │   │   ├── localization.yaml
    │   │   ├── slam_toolbox.yaml
    │   │   └── planner_tuning.yaml
    │   ├── rviz/
    │   │   ├── nav_default.rviz
    │   │   ├── nav_debug.rviz
    │   │   └── localization.rviz
    │   ├── maps/
    │   │   ├── office.yaml
    │   │   ├── office.pgm
    │   │   └── ...
    │   ├── behavior_trees/         # if you customize Nav2 BTs
    │   │   └── navigate_w_recovery.xml
    │   └── config/                 # non-Nav2 YAMLs
    │       ├── costmap_filters.yaml
    │       └── route_graph.geojson
    │
    ├── your_robot_bringup/
    │   ├── package.xml
    │   ├── CMakeLists.txt / setup.py
    │   ├── launch/
    │   │   ├── bringup_sim.launch.py
    │   │   ├── bringup_real.launch.py
    │   │   ├── sensors.launch.py
    │   │   ├── drivers.launch.py
    │   │   └── teleop.launch.py
    │   └── config/
    │       ├── ekf.yaml
    │       ├── lidar.yaml
    │       ├── camera.yaml
    │       └── controllers.yaml
    │
    └── your_custom_pkg1/
        └── ...
```
