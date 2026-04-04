#!/usr/bin/env bash

# make the script exit immediately
# if any command within it returns a non-zero exit status.
# This is particularly useful for debugging and ensuring that errors are caught early.
set -e

SESSION="g3nav2"

# --- edit these for your project ---
SELF_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
WS="$SELF_DIR/.."
SETUP_CMD="source $WS/install/setup.bash"

CMD_A="cd $WS && $SETUP_CMD && ros2 launch g3nav2 g3nav2_bringup_launch.py use_slam:=True  use_nav2:=False use_rviz:=False use_frontier:=False"
CMD_B="cd $WS && $SETUP_CMD && ros2 launch g3nav2 g3nav2_bringup_launch.py use_slam:=False use_nav2:=False use_rviz:=True use_frontier:=False"
CMD_C="cd $WS && $SETUP_CMD && ros2 launch g3nav2 g3nav2_bringup_launch.py use_slam:=False use_nav2:=True  use_rviz:=False use_frontier:=False"
CMD_D="cd $WS && $SETUP_CMD && ros2 launch g3nav2 g3nav2_bringup_launch.py use_slam:=False use_nav2:=False  use_rviz:=False use_frontier:=True"
# -----------------------------------

# whether to auto-run commands:
# 1 = run immediately
# 0 = only type them into the shell, wait for you to press Enter
AUTO_RUN=0

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Session already exists: $SESSION"
  echo "Attach with: tmux attach -t $SESSION"
  exit 0
fi

# create session + first pane
tmux new-session -d -s "$SESSION" -n "bringup"

# make 3 panes in the same window
tmux split-window -h -t "$SESSION:bringup"
tmux split-window -v -t "$SESSION:bringup.0"
tmux split-window -v -t "$SESSION:bringup.1"
tmux select-layout -t "$SESSION:bringup" tiled

# label each pane
tmux select-pane -t "$SESSION:bringup.0" -T "Cartographer"
tmux select-pane -t "$SESSION:bringup.1" -T "RViz"
tmux select-pane -t "$SESSION:bringup.2" -T "Nav2"
tmux select-pane -t "$SESSION:bringup.3" -T "Exploration"

send_cmd() {
  local target="$1"
  local label="$2"
  local cmd="$3"

  tmux send-keys -t "$target" "clear" C-m
  tmux send-keys -t "$target" "echo '=== $label ==='" C-m
  tmux send-keys -t "$target" "$cmd"
  if [ "$AUTO_RUN" -eq 1 ]; then
    tmux send-keys -t "$target" C-m
  fi
}

send_cmd "$SESSION:bringup.0" "Pane A - Cartographer" "$CMD_A"
send_cmd "$SESSION:bringup.1" "Pane B - RViz" "$CMD_B"
send_cmd "$SESSION:bringup.2" "Pane C - Nav2" "$CMD_C"
send_cmd "$SESSION:bringup.3" "Pane D - Exploration" "$CMD_D"

tmux attach -t "$SESSION"