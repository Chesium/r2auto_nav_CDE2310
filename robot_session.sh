#!/usr/bin/env bash
# Launch a tmux session with camera, foxglove, and aruco in one window.
# Usage: bash robot_session.sh
# Kill session: tmux kill-session -t robot

SESSION="robot"

tmux kill-session -t $SESSION 2>/dev/null

# Create session — single window split into 3 vertical panes
tmux new-session -d -s $SESSION -n "main" -x 220 -y 50

# Pane 0 (left): USB Camera
tmux send-keys -t $SESSION:main "ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:='/dev/video1' \
  -p pixel_format:='mjpeg2rgb' \
  -p image_width:=640 \
  -p image_height:=480 \
  -p camera_name:='usb_cam' \
  -p camera_info_url:='file:///home/g3/camera_ws/src/calibration/usb_cam_calibration.yaml' \
  -r image_raw:=/usb_cam/image_raw \
  -r camera_info:=/usb_cam/camera_info" Enter

# Pane 1 (middle): Foxglove bridge
tmux split-window -h -t $SESSION:main
tmux send-keys -t $SESSION:main "ros2 launch foxglove_bridge foxglove_bridge_launch.xml" Enter

# Pane 2 (right): ArUco dock — pre-sourced, manual launch
tmux split-window -h -t $SESSION:main
tmux send-keys -t $SESSION:main "cd ~/nav_ws && source install/setup.bash" Enter
tmux send-keys -t $SESSION:main "# Run: ros2 launch g3_visual_servo aruco_dock.launch.py"

# Even out pane widths
tmux select-layout -t $SESSION:main even-horizontal

# Focus left pane
tmux select-pane -t $SESSION:main.0

tmux attach-session -t $SESSION
