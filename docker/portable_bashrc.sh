#!/usr/bin/env bash

export NAV_WS="${NAV_WS:-$HOME/nav_ws}"

if [[ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]]; then
  source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
fi

if [[ -f "${NAV_WS}/docker/ros_network.env" ]]; then
  source "${NAV_WS}/docker/ros_network.env"
fi

if [[ -n "${ROS_DISCOVERY_SERVER:-}" ]]; then
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  export ROS_SUPER_CLIENT="${ROS_SUPER_CLIENT:-TRUE}"
else
  unset ROS_SUPER_CLIENT
fi

export ROS_DISCOVERY_INTERFACE="${ROS_DISCOVERY_INTERFACE:-127.0.0.1}"
export ROS_DISCOVERY_PORT="${ROS_DISCOVERY_PORT:-11811}"

nb() { nano ~/.bashrc; }
sb() { source ~/.bashrc; }

roset() { source "${NAV_WS}/install/setup.bash"; }
talker() { ros2 run demo_nodes_cpp talker; }
topics() { ros2 topic list; }
foxnode() { ros2 launch foxglove_bridge foxglove_bridge_launch.xml; }
discovery() { fastdds discovery -i 0 -l "${ROS_DISCOVERY_INTERFACE}" -p "${ROS_DISCOVERY_PORT}"; }

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"

if [[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]]; then
  source "$HOME/turtlebot3_ws/install/setup.bash"
fi

if [[ -f "${NAV_WS}/install/setup.bash" ]]; then
  source "${NAV_WS}/install/setup.bash"
fi

slam() { ros2 launch turtlebot3_cartographer cartographer.launch.py; }
rteleop() { ros2 run turtlebot3_teleop teleop_keyboard; }
