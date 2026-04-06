#!/usr/bin/env python3
"""
Detection Launch File
---------------------
Launches the Gazebo camera -> ROS 2 bridge + the YOLO26 person detector.

Prerequisites:
  - PX4 SITL must be running with x500_mono_cam model:
      export GZ_SIM_RESOURCE_PATH=...  (or source gz_env.sh)
      cd ~/Drone/PX4-Autopilot && make px4_sitl gz_x500_mono_cam
  - MicroXRCEAgent must be running:
      MicroXRCEAgent udp4 -p 8888
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Path to the YAML config for ros_gz_bridge
BRIDGE_CONFIG = os.path.join(
    os.path.expanduser('~/Drone/ros2_ws/src/drone_vision/config'),
    'gz_bridge.yaml'
)

# venv site-packages path — injected ONLY into the detector process,
# NOT globally, so Qt-dependent tools (rqt, rviz) in other terminals
# are never affected.
VENV_SITE = os.path.expanduser('~/Drone/venv/lib/python3.12/site-packages')


def generate_launch_description():
    # === Arguments ===
    conf_arg = DeclareLaunchArgument(
        'confidence', default_value='0.45',
        description='YOLO26 confidence threshold')

    mode_arg = DeclareLaunchArgument(
        'mode', default_value='search',
        description="Mission mode: 'square' (test) or 'search' (follow targets)")

    # === Gazebo -> ROS 2 Camera Bridge (YAML config) ===
    # No venv injection needed — pure ROS node.
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_bridge',
        parameters=[{'config_file': BRIDGE_CONFIG}],
        output='screen',
    )

    # === YOLO26 Person Detector ===
    # venv PYTHONPATH scoped to THIS process only via additional_env.
    # YOLO_AUTOINSTALL=false prevents ultralytics from calling pip at
    # startup (PEP-668 protection on managed Ubuntu environments).
    existing_pp = os.environ.get('PYTHONPATH', '')
    detector_pythonpath = f"{VENV_SITE}:{existing_pp}" if existing_pp else VENV_SITE
    detector = Node(
        package='drone_vision',
        executable='person_detector',
        name='person_detector',
        parameters=[{
            'image_topic': '/drone/camera_raw',
            'confidence_threshold': LaunchConfiguration('confidence'),
            'process_every_n': 2,
        }],
        additional_env={
            'PYTHONPATH': detector_pythonpath,
            'YOLO_AUTOINSTALL': 'false',
            'YOLO_VERBOSE': 'false',
        },
        output='screen',
    )

    # === Mission Controller ===
    # No venv injection needed — uses only px4_msgs / rclpy.
    mission = Node(
        package='drone_vision',
        executable='mission_node',
        name='mission_node',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
        }],
        output='screen',
    )

    return LaunchDescription([
        conf_arg,
        mode_arg,
        gz_bridge,
        detector,
        mission,
    ])
