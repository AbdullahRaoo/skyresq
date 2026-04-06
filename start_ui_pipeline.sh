#!/usr/bin/env bash
# Stable one-command startup for PX4 + Gazebo UI + ROS pipeline.

set -euo pipefail

ROOT="$HOME/Drone"
PX4_ROOT="$ROOT/PX4-Autopilot"
LOG_DIR="$ROOT/.run_logs"
PID_DIR="$ROOT/.run_pids"
mkdir -p "$LOG_DIR" "$PID_DIR"

function clean_env_exec() {
  local cmd="$1"
  env -i \
    HOME="${HOME}" \
    USER="${USER}" \
    DISPLAY="${DISPLAY:-}" \
    WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}" \
    XDG_SESSION_TYPE="${XDG_SESSION_TYPE:-}" \
    XAUTHORITY="${XAUTHORITY:-}" \
    TERM="${TERM:-xterm}" \
    LANG="${LANG:-C.UTF-8}" \
    PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/opt/ros/jazzy/bin" \
    bash -lc "
      unset VIRTUAL_ENV PYTHONHOME
      unset GTK_EXE_PREFIX GTK_PATH LOCPATH GIO_MODULE_DIR GTK_IM_MODULE_FILE
      ${cmd}
    "
}

echo "[1/4] Starting Gazebo world (GUI)..."
clean_env_exec "
  cd '$PX4_ROOT'
  source build/px4_sitl_default/rootfs/gz_env.sh
  gz sim -r '$PX4_ROOT/Tools/simulation/gz/worlds/default.sdf'
" >"$LOG_DIR/gz.log" 2>&1 &
echo $! > "$PID_DIR/gz.pid"
sleep 3

echo "[2/4] Starting PX4 (attach to running Gazebo)..."
clean_env_exec "
  cd '$PX4_ROOT'
  source build/px4_sitl_default/rootfs/gz_env.sh
  export PX4_GZ_STANDALONE=1
  make px4_sitl gz_x500_mono_cam
" >"$LOG_DIR/px4.log" 2>&1 &
echo $! > "$PID_DIR/px4.pid"
sleep 3

echo "[3/4] Starting MicroXRCEAgent..."
clean_env_exec "
  '$ROOT/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent' udp4 -p 8888
" >"$LOG_DIR/xrce.log" 2>&1 &
echo $! > "$PID_DIR/xrce.pid"
sleep 2

echo "[4/4] Starting ROS detection pipeline..."
clean_env_exec "
  source /opt/ros/jazzy/setup.bash
  source '$ROOT/ros2_ws/install/setup.bash'
  ros2 launch drone_vision detection.launch.py mode:=search
" >"$LOG_DIR/ros.log" 2>&1 &
echo $! > "$PID_DIR/ros.pid"
sleep 2

echo "Startup complete."
echo "- Logs: $LOG_DIR"
echo "- PIDs: $PID_DIR"
echo "- Open camera: $ROOT/view_camera.sh debug"
