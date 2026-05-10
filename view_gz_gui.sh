#!/usr/bin/env bash
# Launch gz gui (Gazebo visualization client) in a clean environment.
# Strips VS Code / snap library paths that break Qt at runtime.
# The Gazebo server must already be running (started by start_ui_pipeline.sh).

env -i \
  HOME="${HOME}" \
  USER="${USER}" \
  DISPLAY="${DISPLAY:-:0}" \
  WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" \
  DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}" \
  XDG_SESSION_TYPE="${XDG_SESSION_TYPE:-}" \
  XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
  TERM="${TERM:-xterm}" \
  LANG="${LANG:-C.UTF-8}" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/opt/ros/jazzy/bin" \
  bash -lc "
    unset VIRTUAL_ENV PYTHONHOME PYTHONPATH LD_LIBRARY_PATH
    unset GTK_EXE_PREFIX GTK_PATH LOCPATH GIO_MODULE_DIR GTK_IM_MODULE_FILE
    unset SNAP SNAP_NAME SNAP_VERSION SNAP_REVISION SNAP_ARCH SNAP_LIBRARY_PATH
    source /opt/ros/jazzy/setup.bash
    # 'gz sim -g' launches the GUI client and auto-connects to the running
    # 'gz sim -s' server, pulling its scene config. 'gz gui' alone starts
    # blank ('insert plugins to start') because it has no scene context.
    exec gz sim -g
  "
