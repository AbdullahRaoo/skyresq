#!/bin/bash
# launch_drone.sh — wrapper invoked by drone-ros2.service.
#
# Sources ROS 2, the workspace overlay, and the Python venv, then execs
# `ros2 launch drone_vision hardware.launch.py` with parameters read from
# /etc/default/drone-ros2 (EnvironmentFile=).

set -euo pipefail

# Sane defaults if the EnvironmentFile is missing for any reason
: "${ROS_DISTRO:=jazzy}"
: "${WORKSPACE:=/home/abdullah/Drone/ros2_ws}"
: "${VENV:=/home/abdullah/Drone/venv}"

: "${CONNECTION_STRING:=/dev/serial0}"
: "${BAUD_RATE:=57600}"
: "${RTSP_URL:=rtsp://192.168.144.108/stream1}"
: "${GIMBAL_HOST:=192.168.144.108}"
: "${GIMBAL_PORT:=2332}"
: "${GIMBAL_BACKEND:=tcp}"
: "${GST_PIPELINE:=}"
: "${GCS_IP:=100.123.87.26}"
: "${GCS_PORT:=5005}"
: "${CONFIDENCE:=0.45}"
: "${DETECTOR_BACKEND:=ncnn}"

# Source ROS 2 base
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Source workspace overlay (the install/ tree built by colcon)
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WORKSPACE}/install/setup.bash"
else
    echo "ERROR: workspace not built — run 'colcon build' in ${WORKSPACE}" >&2
    exit 1
fi

# Add the venv site-packages to PYTHONPATH (ultralytics / ncnn / pymavlink live here)
if [[ -d "${VENV}/lib" ]]; then
    PY_SITE=$(ls -d "${VENV}"/lib/python*/site-packages 2>/dev/null | head -1)
    if [[ -n "${PY_SITE}" ]]; then
        export PYTHONPATH="${PY_SITE}:${PYTHONPATH:-}"
    fi
fi

echo "[launch_drone] ROS 2: ${ROS_DISTRO}"
echo "[launch_drone] Workspace: ${WORKSPACE}"
echo "[launch_drone] FC: ${CONNECTION_STRING} @ ${BAUD_RATE}"
echo "[launch_drone] Gimbal: ${GIMBAL_HOST}:${GIMBAL_PORT} (${GIMBAL_BACKEND})"
echo "[launch_drone] GCS: ${GCS_IP}:${GCS_PORT}"

exec ros2 launch drone_vision hardware.launch.py \
    connection_string:="${CONNECTION_STRING}" \
    baud_rate:="${BAUD_RATE}" \
    rtsp_url:="${RTSP_URL}" \
    gimbal_host:="${GIMBAL_HOST}" \
    gimbal_port:="${GIMBAL_PORT}" \
    gimbal_backend:="${GIMBAL_BACKEND}" \
    gst_pipeline:="${GST_PIPELINE}" \
    gcs_ip:="${GCS_IP}" \
    gcs_port:="${GCS_PORT}" \
    confidence:="${CONFIDENCE}" \
    backend:="${DETECTOR_BACKEND}"
