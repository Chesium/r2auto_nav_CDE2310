#!/usr/bin/env bash

# make the script exit immediately
# if any command within it returns a non-zero exit status.
# This is particularly useful for debugging and ensuring that errors are caught early.
set -e

SESSION="ros2env"

# --- edit these for your project ---
SELF_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
WS="$SELF_DIR/.."
SETUP_CMD="source $WS/install/setup.bash"

CMD_A="foxnode"
CMD_B="discovery"
CMD_C="killnav2"
CMD_D="ros2 service call /exploration/set_enabled std_srvs/srv/SetBool \"{data: true}\""
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
tmux new-session -d -s "$SESSION" -n "ros2env"

# make 3 panes in the same window
tmux split-window -h -t "$SESSION:ros2env"
tmux split-window -v -t "$SESSION:ros2env.0"
tmux split-window -v -t "$SESSION:ros2env.1"
tmux select-layout -t "$SESSION:ros2env" tiled

# label each pane
tmux select-pane -t "$SESSION:ros2env.0" -T "foxnode"
tmux select-pane -t "$SESSION:ros2env.1" -T "discovery"
tmux select-pane -t "$SESSION:ros2env.2" -T "killnav2"
tmux select-pane -t "$SESSION:ros2env.3" -T "activateExploration"

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

send_cmd "$SESSION:ros2env.0" "Pane A - foxnode" "$CMD_A"
send_cmd "$SESSION:ros2env.1" "Pane B - discovery" "$CMD_B"
send_cmd "$SESSION:ros2env.2" "killnav2" "$CMD_C"
send_cmd "$SESSION:ros2env.3" "Pane D - activateExploration" "$CMD_D"

tmux attach -t "$SESSION"