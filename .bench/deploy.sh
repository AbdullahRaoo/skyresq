#!/bin/bash
# rsync drone_vision to the Pi and (optionally) colcon build + restart pipeline.
#
# Usage:
#   deploy.sh              # sync only
#   deploy.sh build        # sync + colcon build --packages-select drone_vision
#   deploy.sh restart      # sync + build + stop_bench + bench_pipeline (background)
set -euo pipefail

PI_HOST="${PI_HOST:-raspberrypi.tail7c9eac.ts.net}"
PI_USER="${PI_USER:-pi}"
LOCAL_SRC="${LOCAL_SRC:-$HOME/Drone/ros2_ws/src/drone_vision/}"
REMOTE_DST="${REMOTE_DST:-/home/pi/Drone/ros2_ws/src/drone_vision/}"
SSHPASS="${SSHPASS:-aw}"

MODE="${1:-sync}"

echo "==== rsync $LOCAL_SRC -> $PI_USER@$PI_HOST:$REMOTE_DST"
sshpass -p "$SSHPASS" rsync -avz --delete \
    --exclude='__pycache__' --exclude='*.pyc' \
    -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" \
    "$LOCAL_SRC" "$PI_USER@$PI_HOST:$REMOTE_DST"

run_remote() {
    sshpass -p "$SSHPASS" ssh -o StrictHostKeyChecking=no \
        -o UserKnownHostsFile=/dev/null "$PI_USER@$PI_HOST" "$1"
}

if [[ "$MODE" == "build" || "$MODE" == "restart" ]]; then
    echo "==== colcon build (drone_vision) on Pi"
    run_remote "source /opt/ros/humble/setup.bash 2>/dev/null || source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros_humble; \
        cd ~/Drone/ros2_ws && colcon build --packages-select drone_vision --symlink-install 2>&1 | tail -15"
fi

if [[ "$MODE" == "restart" ]]; then
    echo "==== restart pipeline"
    run_remote "bash /tmp/stop_bench.sh 2>/dev/null || true; \
        DURATION_S=1800 setsid bash /tmp/bench_pipeline.sh > /tmp/bench/launch.log 2>&1 < /dev/null &"
    echo "pipeline relaunched"
fi

echo "done."
