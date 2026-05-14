#!/bin/bash
# Long-running demo pipeline — 30 min by default, killable any time
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble
source ~/Drone/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

DEV_PC_IP="100.93.242.103"
DURATION_S=${DURATION_S:-1800}        # 30 min
mkdir -p /tmp/bench
declare -A PIDS

start_node() {
    local name=$1; shift
    setsid ros2 run drone_vision "$name" --ros-args "$@" \
        > "/tmp/bench/${name}.log" 2>&1 &
    PIDS[$name]=$!
    echo "  started $name PID=${PIDS[$name]}"
}

echo "==== Starting long-running pipeline (gimbal active, ${DURATION_S}s) ===="
start_node mavlink_bridge \
    -p connection_string:=/dev/serial0 \
    -p baud_rate:=57600 \
    -p gcs_forward_ip:=$DEV_PC_IP \
    -p gcs_forward_port:=14550

start_node rtsp_camera \
    -p rtsp_url:=rtsp://127.0.0.1:8554/skyresq_cam \
    -p publish_compressed:=false

start_node gimbal_controller \
    -p backend:=tcp \
    -p gimbal_host:=192.168.144.108 \
    -p gimbal_port:=2332 \
    -p command_rate_hz:=50.0 \
    -p initial_pitch_deg:=-10.0 \
    -p initial_yaw_deg:=0.0 \
    -p pixel_gain_yaw:=30.0 \
    -p pixel_gain_pitch:=20.0 \
    -p max_slew_dps:=80.0

start_node visual_servo

start_node payload_servo

start_node person_detector \
    -p image_topic:=/drone/camera_raw \
    -p backend:=ncnn \
    -p model_path:=/home/pi/Drone/yolo26n_ncnn_model \
    -p confidence_threshold:=0.30 \
    -p process_every_n:=2 \
    -p imgsz:=320 \
    -p publish_debug:=false \
    -p timing_log_period_s:=5.0 \
    -p best_by:=area

start_node geo_localiser

start_node gcs_link \
    -p gcs_ip:=$DEV_PC_IP \
    -p gcs_port:=5005 \
    -p confidence_min:=0.30 \
    -p stream_width:=1920 \
    -p stream_height:=1080

echo
echo "Pipeline running for ${DURATION_S}s — dashboard should see live packets."
echo "PIDs: ${PIDS[@]}"
echo "To stop early: bash /tmp/stop_bench.sh"
sleep $DURATION_S

echo
echo "==== Cleanup ===="
for name in "${!PIDS[@]}"; do
    pid=${PIDS[$name]}
    kill -- -$pid 2>/dev/null
done
sleep 2
for name in "${!PIDS[@]}"; do
    pid=${PIDS[$name]}
    ps -p $pid >/dev/null 2>&1 && kill -9 -- -$pid 2>/dev/null
done
echo done
