#!/usr/bin/env python3
"""
SITL Launch File
----------------
Brings up the full simulation pipeline:
  gz_camera_bridge   Gazebo /camera → ROS /drone/camera_raw
  person_detector    YOLO26 inference, publishes /target_position
  visual_servo       50 Hz pixel-error → /gimbal/cmd/look_at_pixel
  gimbal_sim         virtual gimbal + ROI-cropped /drone/camera_raw_stabilised
  mission_node       state machine

The gimbal subsystem is observable but NOT yet wired into the mission
state machine in PR-1a — mission_node still does its own NED-nudge
TRACK behaviour. PR-1b switches the loop to use gimbal-aware tracking.

Prerequisites:
  - PX4 SITL running with x500_mono_cam model.
  - MicroXRCEAgent running on UDP 8888.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

CONFIG_DIR = os.path.expanduser('~/Drone/ros2_ws/src/drone_vision/config')
BRIDGE_CONFIG  = os.path.join(CONFIG_DIR, 'gz_bridge.yaml')
GIMBAL_PARAMS  = os.path.join(CONFIG_DIR, 'gimbal_params.yaml')
PAYLOAD_PARAMS = os.path.join(CONFIG_DIR, 'payload_params.yaml')
MISSION_PARAMS = os.path.join(CONFIG_DIR, 'mission_params.yaml')

# venv site-packages — injected ONLY into the detector process so Qt-using
# tools (rqt/rviz) in other terminals are never affected.
VENV_SITE = os.path.expanduser('~/Drone/venv/lib/python3.12/site-packages')


def generate_launch_description():
    # === Arguments ===
    conf_arg = DeclareLaunchArgument(
        'confidence', default_value='0.45',
        description='YOLO26 confidence threshold')
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='search',
        description="Mission mode: 'square' (test) or 'search' (follow targets)")
    imgsz_arg = DeclareLaunchArgument(
        'imgsz', default_value='640',
        description='YOLO inference resolution (must match exported ONNX shape)')

    # === Gazebo → ROS 2 camera bridge ===
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_bridge',
        parameters=[{'config_file': BRIDGE_CONFIG}],
        output='screen',
    )

    # === YOLO26 Person Detector ===
    existing_pp = os.environ.get('PYTHONPATH', '')
    detector_pythonpath = f"{VENV_SITE}:{existing_pp}" if existing_pp else VENV_SITE
    detector = Node(
        package='drone_vision',
        executable='person_detector',
        name='person_detector',
        parameters=[{
            # PR-1b: detector reads the gimbaled view so visual_servo closes
            # the loop. As the gimbal pans to centre the person, the pixel
            # error in this stream goes to zero.
            'image_topic': '/drone/camera_raw_stabilised',
            'confidence_threshold': LaunchConfiguration('confidence'),
            'process_every_n': 3,
            'imgsz': LaunchConfiguration('imgsz'),
            # PR-4: backend selection. SITL stays on ultralytics (PC venv);
            # the Pi's hardware.launch.py overrides to 'ncnn'.
            'backend': 'ultralytics',
        }],
        additional_env={
            'PYTHONPATH': detector_pythonpath,
            'YOLO_AUTOINSTALL': 'false',
            'YOLO_VERBOSE': 'false',
        },
        output='screen',
    )

    # === Visual servo (PR-1a) ===
    # 50 Hz republisher. Bridges low-rate detector output to high-rate gimbal cmds.
    visual_servo = Node(
        package='drone_vision',
        executable='visual_servo',
        name='visual_servo',
        output='screen',
    )

    # === Gimbal sim (PR-1a) ===
    # Virtual 3-axis gimbal + ROI crop on the wide source camera.
    gimbal_sim = Node(
        package='drone_vision',
        executable='gimbal_sim',
        name='gimbal_sim',
        parameters=[GIMBAL_PARAMS],
        output='screen',
    )

    # === Mission Controller ===
    mission = Node(
        package='drone_vision',
        executable='mission_node',
        name='mission_node',
        parameters=[
            MISSION_PARAMS,
            {'mode': LaunchConfiguration('mode')},
        ],
        output='screen',
    )

    # === Geo-localiser (PR-2a) ===
    # Publishes /target/world. Consumed by mission_node v2 (PR-2b).
    geo_localiser = Node(
        package='drone_vision',
        executable='geo_localiser',
        name='geo_localiser',
        output='screen',
    )

    # === Payload sim stub (PR-2b) ===
    # Provides /payload/drop service with safety interlocks. Real (pigpio)
    # backend lands in PR-3.
    payload_sim = Node(
        package='drone_vision',
        executable='payload_servo_sim',
        name='payload_servo',
        parameters=[PAYLOAD_PARAMS],
        output='screen',
    )

    return LaunchDescription([
        conf_arg,
        mode_arg,
        imgsz_arg,
        gz_bridge,
        detector,
        visual_servo,
        gimbal_sim,
        mission,
        geo_localiser,
        payload_sim,
    ])
