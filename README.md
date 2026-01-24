# 🚁 Autonomous Search and Rescue Drone (UAV)

Autonomous drone for Person Search & Rescue.
Features: GPU-accelerated Person Detection (YOLOv8), Gazebo Simulation, and ROS 2 Jazzy.

## 📦 System Overview

- **OS**: Linux (Ubuntu 24.04 recommended)
- **ROS 2**: Jazzy Jalisco
- **Hardware Config**:
  - **PC**: Simulation & Deep Learning (RTX 5060, CUDA 13.x)
  - **RPi 4**: Flight Control (MAVROS/MAVLink)
- **Repo Structure**:
  - `ros2_ws/`: Main ROS 2 workspace
  - `requirements/`: Python dependencies (Split for PC vs RPi)

## 🚀 Quick Start (PC Simulation)

### 1. Setup
```bash
# Create virtual environment (if not exists)
python3 -m venv venv
source venv/bin/activate

# Install PC requirements (YOLO, Torch, etc.)
pip install -r requirements/pc.txt
```

### 2. Build Workspace
```bash
cd ~/Drone/ros2_ws
colcon build --packages-select drone_vision
source install/setup.bash
```

### 3. Run Simulation & Detector
```bash
ros2 launch drone_vision detection.launch.py
```
This launches:
- Gazebo Sim
- ROS-Gazebo Bridge
- Person Detector Node (YOLOv8)

## 🛠️ Hardware Setup (Raspberry Pi)
For the RPi 4, we use a lightweight setup (No Computer Vision).
```bash
pip install -r requirements/rpi.txt
```

## 👥 Authors
- Abdullah
