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
