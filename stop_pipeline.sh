#!/usr/bin/env bash
# Stop all PX4/Gazebo/ROS pipeline processes started by this project.

set -euo pipefail

ROOT="$HOME/Drone"
PID_DIR="$ROOT/.run_pids"

echo "Stopping managed processes..."
if [[ -d "$PID_DIR" ]]; then
  for f in "$PID_DIR"/*.pid; do
    [[ -e "$f" ]] || continue
    pid=$(cat "$f" 2>/dev/null || true)
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
fi

sleep 1

# Safety net for any related leftover processes.
pkill -9 -f "px4|gz sim|gzserver|gzclient|MicroXRCEAgent|ros2 launch drone_vision|detection.launch.py|mission_node|person_detector|parameter_bridge|rqt_image_view|cv_viewer.py" || true

echo "Stopped."
