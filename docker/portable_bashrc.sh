#!/usr/bin/env bash

export NAV_WS="${NAV_WS:-$HOME/nav_ws}"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi

if [[ -f "${NAV_WS}/docker/ros_network.env" ]]; then
  source "${NAV_WS}/docker/ros_network.env"
fi

alias nb="nano ~/.bashrc"
alias sb="source ~/.bashrc"

alias roset="source ${NAV_WS}/install/setup.bash"
alias talker="ros2 run demo_nodes_cpp talker"
alias topics="ros2 topic list"
alias foxnode="ros2 launch foxglove_bridge foxglove_bridge_launch.xml"

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

if [[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]]; then
  source "$HOME/turtlebot3_ws/install/setup.bash"
fi

if [[ -f "${NAV_WS}/install/setup.bash" ]]; then
  source "${NAV_WS}/install/setup.bash"
fi

alias slam="ros2 launch turtlebot3_cartographer cartographer.launch.py"
alias rteleop="ros2 run turtlebot3_teleop teleop_keyboard"
