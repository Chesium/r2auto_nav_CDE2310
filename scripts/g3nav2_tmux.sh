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

CMD_A="cd $WS && $SETUP_CMD && ros2 launch g3nav2 g3nav2_bringup_launch.py use_slam:=True  use_nav2:=False use_rviz:=False"
CMD_B="cd $WS && $SETUP_CMD && ros2 launch g3nav2 g3nav2_bringup_launch.py use_slam:=False use_nav2:=False use_rviz:=True"
CMD_C="cd $WS && $SETUP_CMD && ros2 launch g3nav2 g3nav2_bringup_launch.py use_slam:=False use_nav2:=True  use_rviz:=False"
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
tmux split-window -h -t "$SESSION:bringup"
tmux select-layout -t "$SESSION:bringup" even-horizontal

# label each pane
tmux select-pane -t "$SESSION:bringup.0" -T "A"
tmux select-pane -t "$SESSION:bringup.1" -T "B"
tmux select-pane -t "$SESSION:bringup.2" -T "C"

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

send_cmd "$SESSION:bringup.0" "Pane A" "$CMD_A"
send_cmd "$SESSION:bringup.1" "Pane B" "$CMD_B"
send_cmd "$SESSION:bringup.2" "Pane C" "$CMD_C"

tmux attach -t "$SESSION"