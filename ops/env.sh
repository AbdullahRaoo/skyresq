#!/bin/bash
# Sourced by every SkyResQ systemd unit before exec'ing a ROS node.
# Sets up the conda ROS Humble environment and overlay workspace.
export PATH="/home/pi/miniforge3/bin:$PATH"
source /home/pi/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble
source /home/pi/Drone/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# ROS Humble + Trixie note: rclpy doesn't need anything else; conda's libs
# are first on PATH so it picks up its own libstdc++ instead of the system's.
