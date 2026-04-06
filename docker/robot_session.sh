#!/usr/bin/env bash
# Launch a tmux session with all robot nodes in named panes.
# Usage: ./scripts/robot_session.sh
# Kill session: tmux kill-session -t robot

SESSION="robot"

# Kill existing session if running
tmux kill-session -t $SESSION 2>/dev/null

# Create session with first window: bringup
tmux new-session -d -s $SESSION -n "bringup" -x 220 -y 50

# Send bringup command
tmux send-keys -t $SESSION:bringup "export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_bringup robot.launch.py" Enter

# Pane 2: USB Camera
tmux new-window -t $SESSION -n "camera"
tmux send-keys -t $SESSION:camera "ros2 run usb_cam usb_cam_node_exe --ros-args \\
  -p video_device:='/dev/video1' \\
  -p pixel_format:='mjpeg2rgb' \\
  -p image_width:=640 \\
  -p image_height:=480 \\
  -p camera_name:='usb_cam' \\
  -p camera_info_url:='file:///home/g3/camera_ws/src/calibration/usb_cam_calibration.yaml' \\
  -r image_raw:=/usb_cam/image_raw \\
  -r camera_info:=/usb_cam/camera_info" Enter

# Pane 3: Foxglove bridge
tmux new-window -t $SESSION -n "foxglove"
tmux send-keys -t $SESSION:foxglove "ros2 launch foxglove_bridge foxglove_bridge_launch.xml" Enter

# Pane 4: ArUco dock (don't auto-launch — wait for bringup to settle)
tmux new-window -t $SESSION -n "aruco"
tmux send-keys -t $SESSION:aruco "cd ~/nav_ws && source install/setup.bash" Enter
tmux send-keys -t $SESSION:aruco "# Run when ready: ros2 launch g3_visual_servo aruco_dock.launch.py"

# Pane 5: Free shell for debugging
tmux new-window -t $SESSION -n "shell"

# Go back to bringup window
tmux select-window -t $SESSION:bringup

# Attach
tmux attach-session -t $SESSION
