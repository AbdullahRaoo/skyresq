#!/bin/bash
# launch_drone.sh — wrapper invoked by drone-ros2.service.
#
# Sources ROS 2 + the workspace overlay, then execs
# `ros2 launch drone_vision hardware.launch.py` with parameters read from
# /etc/default/drone-ros2 (EnvironmentFile=).
#
# Supports two ROS 2 install paths:
#   1. CONDA_ENV set (or ~/miniforge3 present) — RoboStack conda env
#      (the Pi 4 path: ROS 2 Humble on Debian Trixie)
#   2. /opt/ros/<distro> present — system apt install (dev PC path)

set -euo pipefail

# Sane defaults if the EnvironmentFile is missing for any reason
: "${ROS_DISTRO:=humble}"
: "${WORKSPACE:=$HOME/Drone/ros2_ws}"
: "${CONDA_ROOT:=$HOME/miniforge3}"
: "${CONDA_ENV:=ros_humble}"

: "${CONNECTION_STRING:=/dev/serial0}"
: "${BAUD_RATE:=57600}"
: "${RTSP_URL:=rtsp://127.0.0.1:8554/skyresq_cam}"
: "${GIMBAL_HOST:=192.168.144.108}"
: "${GIMBAL_PORT:=2332}"
: "${GIMBAL_BACKEND:=tcp}"
: "${GST_PIPELINE:=}"
: "${GCS_IP:=100.123.87.26}"
: "${GCS_PORT:=5005}"
: "${CONFIDENCE:=0.45}"
: "${DETECTOR_BACKEND:=ncnn}"

# ── Source ROS 2 (conda env preferred, fall back to system apt) ──
SOURCED_ROS=""
if [[ -f "${CONDA_ROOT}/etc/profile.d/conda.sh" ]]; then
    # shellcheck disable=SC1090,SC1091
    source "${CONDA_ROOT}/etc/profile.d/conda.sh"
    if conda env list | awk '{print $1}' | grep -qx "${CONDA_ENV}"; then
        conda activate "${CONDA_ENV}"
        SOURCED_ROS="conda:${CONDA_ENV}"
    fi
fi
if [[ -z "${SOURCED_ROS}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    SOURCED_ROS="apt:/opt/ros/${ROS_DISTRO}"
fi
if [[ -z "${SOURCED_ROS}" ]]; then
    echo "ERROR: no ROS 2 install found — checked ${CONDA_ROOT}/envs/${CONDA_ENV} and /opt/ros/${ROS_DISTRO}" >&2
    exit 1
fi

# Source workspace overlay
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WORKSPACE}/install/setup.bash"
else
    echo "ERROR: workspace not built — run 'colcon build' in ${WORKSPACE}" >&2
    exit 1
fi

echo "[launch_drone] ROS 2: ${SOURCED_ROS}"
echo "[launch_drone] Workspace: ${WORKSPACE}"
echo "[launch_drone] FC: ${CONNECTION_STRING} @ ${BAUD_RATE}"
echo "[launch_drone] RTSP: ${RTSP_URL}"
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
