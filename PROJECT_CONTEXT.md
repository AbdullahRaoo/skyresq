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

All blockers are **resolved**. The full detection + navigation pipeline is code-complete and ready for SITL testing.

**Resolved Issues**:
1. ✅ YOLO26 ONNX model loads instantly with no warnings (explicit `task='detect'`).
2. ✅ Ultralytics auto-install hang fixed (`YOLO_AUTOINSTALL=false` set in code + launch files).
3. ✅ `person_detector.py` subscribes to `/drone/camera_raw` (Gazebo bridge topic) by default.
4. ✅ `mission_node.py` upgraded with full state machine supporting:
   - `mode='square'` — legacy test flight (arm → takeoff → square → land).
   - `mode='search'` — autonomous search & rescue (orbit scan → detect → track → descend → land).
5. ✅ `detection.launch.py` now launches camera bridge + detector + mission node together.

**Current Action**: Ready for end-to-end SITL verification.

---

## ⏭️ Next Steps

1.  **End-to-End SITL Verification**: Launch PX4 SITL with `x500_mono_cam`, start the XRCE-DDS agent, and run `ros2 launch drone_vision detection.launch.py` to verify the full pipeline (camera → detection → navigation) works together.
2.  **Place a Person in Gazebo**: Add a person model to the Gazebo world so the camera can detect it and trigger the TRACK → DESCEND → LAND sequence.
3.  **Tune Tracking Parameters**: Adjust `tracking_speed`, `search_radius`, `target_confirm_secs` based on SITL behaviour.
4.  **Hardware Preparation (Raspberry Pi)**: Per the `implementation_plan.md.resolved`, deploy this optimized architecture (ONNX, compressed image transport, headless OS, systemd) to the RPi companion computer.
