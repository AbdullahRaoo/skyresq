#!/usr/bin/env python3
"""
SITL Launch File — ArduPilot SITL.

Brings up the full simulation pipeline against a running ArduPilot SITL
instance (sim_vehicle.py). No PX4, no Gazebo required.

Start ArduPilot SITL first:
  cd ArduCopter
  sim_vehicle.py -v ArduCopter --model=quad --console --map

Then launch this file:
  ros2 launch drone_vision sitl.launch.py

For a real camera or video file instead of a test pattern:
  ros2 launch drone_vision sitl.launch.py camera_url:=/dev/video0
  ros2 launch drone_vision sitl.launch.py camera_url:=file:///path/to/test.mp4

Nodes launched
--------------
  mavlink_bridge     ArduPilot SITL TCP → /vehicle/* ROS topics
  gimbal_sim         Null/nadir gimbal sim + camera stabilised topic
  visual_servo       50 Hz pixel-error republisher → /gimbal/cmd/look_at_pixel
  person_detector    YOLO inference on stabilised camera feed
  geo_localiser      Pixel + gimbal + pose → /target/world
  gcs_link           /target/world → UDP JSON to SkyResQ GCS (optional)
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

CONFIG_DIR    = os.path.expanduser('~/Drone/ros2_ws/src/drone_vision/config')
GIMBAL_PARAMS = os.path.join(CONFIG_DIR, 'gimbal_params.yaml')

VENV_SITE = os.path.expanduser('~/Drone/venv/lib/python3.12/site-packages')


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'sitl_connection', default_value='tcp:127.0.0.1:5760',
            description='ArduPilot SITL MAVLink endpoint (TCP or UDP)'),
        DeclareLaunchArgument(
            'camera_url', default_value='',
            description='Camera source: /dev/video0, file:///path/vid.mp4, or empty for test pattern'),
        DeclareLaunchArgument(
            'confidence', default_value='0.45',
            description='YOLO confidence threshold'),
        DeclareLaunchArgument(
            'gcs_ip', default_value='127.0.0.1',
            description='SkyResQ GCS IP for survivor cluster UDP packets'),
        DeclareLaunchArgument(
            'gcs_port', default_value='5005'),
        DeclareLaunchArgument(
            'imgsz', default_value='640',
            description='YOLO inference resolution'),
    ]

    # ── MAVLink Bridge ────────────────────────────────────────────────
    # Connects to ArduPilot SITL via TCP and publishes /vehicle/* topics.
    # Identical to hardware; only connection_string differs.
    mavlink_bridge = Node(
        package='drone_vision',
        executable='mavlink_bridge',
        name='mavlink_bridge',
        parameters=[{
            'connection_string': LaunchConfiguration('sitl_connection'),
            'stream_hz':         10,
            'heartbeat_hz':      1.0,
        }],
        output='screen',
    )

    # ── Gimbal sim ────────────────────────────────────────────────────
    # Virtual nadir gimbal + ROI-cropped stabilised camera topic.
    # Accepts the camera_url arg to attach to a real video source.
    gimbal_sim = Node(
        package='drone_vision',
        executable='gimbal_sim',
        name='gimbal_sim',
        parameters=[
            GIMBAL_PARAMS,
            {'image_topic': LaunchConfiguration('camera_url')},
        ],
        output='screen',
    )

    # ── Visual servo ──────────────────────────────────────────────────
    visual_servo = Node(
        package='drone_vision',
        executable='visual_servo',
        name='visual_servo',
        output='screen',
    )

    # ── Person detector ───────────────────────────────────────────────
    existing_pp = os.environ.get('PYTHONPATH', '')
    detector_pythonpath = f"{VENV_SITE}:{existing_pp}" if existing_pp else VENV_SITE

    person_detector = Node(
        package='drone_vision',
        executable='person_detector',
        name='person_detector',
        parameters=[{
            'image_topic':          '/drone/camera_raw_stabilised',
            'confidence_threshold': LaunchConfiguration('confidence'),
            'process_every_n':      3,
            'imgsz':                LaunchConfiguration('imgsz'),
            'backend':              'ultralytics',
        }],
        additional_env={
            'PYTHONPATH': detector_pythonpath,
            'YOLO_AUTOINSTALL': 'false',
            'YOLO_VERBOSE': 'false',
        },
        output='screen',
    )

    # ── Geo-localiser ─────────────────────────────────────────────────
    geo_localiser = Node(
        package='drone_vision',
        executable='geo_localiser',
        name='geo_localiser',
        output='screen',
    )

    # ── GCS link ──────────────────────────────────────────────────────
    # Sends survivor_cluster JSON to SkyResQ dashboard (loopback by default).
    gcs_link = Node(
        package='drone_vision',
        executable='gcs_link',
        name='gcs_link',
        parameters=[{
            'gcs_ip':   LaunchConfiguration('gcs_ip'),
            'gcs_port': LaunchConfiguration('gcs_port'),
        }],
        output='screen',
    )

    return LaunchDescription([
        *args,
        mavlink_bridge,
        gimbal_sim,
        visual_servo,
        person_detector,
        geo_localiser,
        gcs_link,
    ])
