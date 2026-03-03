# Drone Vision Project - Context & Status

This document serves as a comprehensive summary of the project's journey, current state, and future objectives as determined through our development sessions.

## 🎯 Overall Project Goals
To build an autonomous drone system capable of:
1.  **Person Detection**: Using a camera and YOLOv8 to identify people in real-time.
2.  **Autonomous Navigation**: Utilizing ROS 2 to send offboard control commands to a PX4 flight controller to navigate towards detected targets.
3.  **Payload Delivery**: Executing a payload drop mechanism once the target is reached.
4.  **Hardware Deployment**: Eventually porting this software stack from a simulated environment (SITL) to real hardware (e.g., Raspberry Pi companion computer communicating with a real PX4 flight controller).

---

## 🛠️ What We Have Accomplished (The Journey)

Getting the simulation environment stable was our biggest hurdle. We successfully navigated complex version compatibility issues:

1.  **ROS 2 Package Creation**: 
    *   Created the `drone_vision` ROS 2 package containing our core logic.
    *   Implemented `person_detector.py` using Ultralytics YOLOv8.
2.  **The DDS Bridge Crisis**: 
    *   We initially tried using the bleeding-edge PX4 `v1.17-alpha` firmware.
    *   This resulted in severe DDS bridge failures (Error Code 255, `eprosima::fastcdr::exception::BadParamException`) because the message serialization formats between PX4 and ROS 2 were out of sync.
3.  **The Stable Downgrade (Success)**: 
    *   We carefully downgraded the entire stack to a scientifically verified compatible matrix:
        *   **OS**: Ubuntu 24.04 (Noble)
        *   **ROS 2**: Jazzy Jalisco
        *   **Gazebo**: Harmonic (Sim 8.10.0)
        *   **PX4 Autopilot**: **v1.15.4** (Stable release)
        *   **Micro-XRCE-DDS-Agent**: **v2.4.3** (Compiled from source; strictly required for PX4 v1.15.x)
        *   **`px4_msgs`**: `release/1.15` branch
    *   *Result*: The DDS bridge now works perfectly. All 45 topics are bridging with zero errors.
4.  **Mission Node Development**: 
    *   Created `mission_node.py` to test basic offboard control (Takeoff -> Fly Square Path -> Land).
    *   Debugged logic flaws where the drone was skipping the mission because it spawned at the origin (Waypoints passing immediately before takeoff). Added explicit pre-arm, takeoff altitude checks, and a mission completion state.

---

## 📍 What We Are Doing Right Now

We have just finished a clean, successful build of the entire environment (Agent, PX4 Firmware, ROS 2 Workspace). 

**Current Action**: We are about to execute the **Square Mission verification test** to prove that our ROS 2 `mission_node` can successfully command the PX4 SITL drone in Gazebo.

To run the simulation, we use three terminals:
1.  **Agent**: `MicroXRCEAgent udp4 -p 8888`
2.  **Simulation**: `cd ~/Drone/PX4-Autopilot && export GZ_SIM_RESOURCE_PATH="..." && make px4_sitl gz_x500`
3.  **ROS 2 Node**: `ros2 run drone_vision mission_node`

---

## ⏭️ Next Steps

Once the square mission is verified and the drone flies as expected in Gazebo, our next immediate goals are:

1.  **Integrate Vision in SITL**: Launch the Gazebo simulation with a camera-equipped drone model and run the `person_detector.py` node alongside it.
2.  **Dynamic Navigation**: Modify the `mission_node` to accept coordinate offsets from the `person_detector` node, allowing the drone to actively track and follow a detected person in the simulation.
3.  **Hardware Preparation**: Begin refining the codebase to ensure it is lightweight and robust enough for deployment on the Raspberry Pi companion computer.
