#!/usr/bin/env bash
# ============================================================
# view_camera.sh - deterministic viewer launcher
#
# Why this exists:
# - VS Code Snap injects GTK/locale vars that break Gazebo/rqt Qt loading.
# - Activating venv can shadow system Qt and numpy used by rqt tools.
#
# This script launches viewer tools in a sanitized environment.
#
# Usage:
#   ./view_camera.sh          # raw feed in rqt
#   ./view_camera.sh debug    # YOLO annotated feed in rqt
#   ./view_camera.sh cv       # OpenCV viewer (no Qt dependency)
# ============================================================

set -euo pipefail

MODE=${1:-raw}

# Keep display/session vars required to show windows.
KEEP_ENV=(
  HOME USER DISPLAY WAYLAND_DISPLAY XDG_RUNTIME_DIR
  DBUS_SESSION_BUS_ADDRESS XDG_SESSION_TYPE XAUTHORITY
  TERM LANG
)

function launch_clean() {
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
    QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-}" \
    bash -lc "
      source /opt/ros/jazzy/setup.bash
      source ~/Drone/ros2_ws/install/setup.bash
      unset VIRTUAL_ENV
      unset PYTHONHOME
      unset GTK_EXE_PREFIX GTK_PATH LOCPATH GIO_MODULE_DIR GTK_IM_MODULE_FILE
      ${cmd}
    "
}

case "$MODE" in
  debug)
    echo "[view_camera] Opening YOLO-annotated feed: /camera/image_debug"
    launch_clean "
      if ros2 run image_tools showimage --ros-args -r image:=/camera/image_debug; then
        exit 0
      fi
      if ros2 run rqt_image_view rqt_image_view --ros-args -r image:=/camera/image_debug; then
        exit 0
      fi
      python3 ~/Drone/ros2_ws/src/drone_vision/tools/cv_viewer.py --debug-only
    "
    ;;
  cv)
    echo "[view_camera] Opening OpenCV viewer: /camera/image_debug"
    launch_clean "python3 ~/Drone/ros2_ws/src/drone_vision/tools/cv_viewer.py --debug-only"
    ;;
  raw|*)
    echo "[view_camera] Opening raw feed: /drone/camera_raw"
    launch_clean "
      if ros2 run image_tools showimage --ros-args -r image:=/drone/camera_raw; then
        exit 0
      fi
      if ros2 run rqt_image_view rqt_image_view --ros-args -r image:=/drone/camera_raw; then
        exit 0
      fi
      python3 ~/Drone/ros2_ws/src/drone_vision/tools/cv_viewer.py --raw-only
    "
    ;;
esac
