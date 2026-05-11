#!/usr/bin/env python3
"""
Hardware Launch File — Raspberry Pi + CubeBlack (ArduCopter).

Brings up the full on-drone pipeline:
  mavlink_bridge      ArduPilot TELEM2 → /vehicle/* ROS topics
  rtsp_camera         Z-1 Mini RTSP → /drone/camera_raw
  gimbal_controller   TCP control of Z-1 Mini + /gimbal/state feedback
  visual_servo        50 Hz pixel-error → /gimbal/cmd/look_at_pixel
  person_detector     YOLO inference on the live camera (ncnn backend)
  geo_localiser       Pixel + gimbal + pose → /target/world
  gcs_link            /target/world → UDP JSON survivor clusters to SkyResQ

Prerequisites on the Pi
-----------------------
  - ROS 2 Jazzy sourced
  - pymavlink + opencv-python installed in the runtime environment
  - ultralytics / ncnn runtime installed (see tools/export_yolo_for_rpi.py)
  - CubeBlack connected on /dev/serial0 (TELEM2) at 57600 baud
  - Z-1 Mini gimbal reachable at 192.168.144.108 (RTSP :554, TCP control :2332)
  - SkyResQ GCS reachable via Tailscale at the configured GCS IP

Usage
-----
  ros2 launch drone_vision hardware.launch.py
  ros2 launch drone_vision hardware.launch.py connection_string:=/dev/ttyAMA0
  ros2 launch drone_vision hardware.launch.py gcs_ip:=100.64.0.5
  ros2 launch drone_vision hardware.launch.py gimbal_backend:=sim  # bench test
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

VENV_SITE = os.path.expanduser("~/Drone/venv/lib/python3.12/site-packages")


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("connection_string", default_value="/dev/serial0",
                              description="UART to CubeBlack TELEM2"),
        DeclareLaunchArgument("baud_rate",         default_value="57600"),
        DeclareLaunchArgument("rtsp_url",
                              default_value="rtsp://192.168.144.108/stream1",
                              description="Z-1 Mini RTSP stream URL"),
        DeclareLaunchArgument("gimbal_host",       default_value="192.168.144.108"),
        DeclareLaunchArgument("gimbal_port",       default_value="2332"),
        DeclareLaunchArgument("gimbal_backend",    default_value="tcp",
                              description="tcp (real gimbal) | sim (no hardware)"),
        DeclareLaunchArgument("gcs_ip",            default_value="100.123.87.26",
                              description="SkyResQ GCS Tailscale IP"),
        DeclareLaunchArgument("gcs_port",          default_value="5005"),
        DeclareLaunchArgument("confidence",        default_value="0.45"),
        DeclareLaunchArgument("backend",           default_value="ncnn",
                              description="Detector backend: ncnn | onnx | ultralytics"),
        DeclareLaunchArgument("gst_pipeline",      default_value="",
                              description="Optional GStreamer pipeline for HW decode"),
        # Skip visual_servo + run gimbal_controller at low rate when the
        # gimbal protocol isn't wired yet (see docs/GIMBAL_PROTOCOL_NOTES.md).
        # Bumps detector throughput ~3x by freeing CPU cores.
        DeclareLaunchArgument("gimbal_active",     default_value="false",
                              description="true once Z-1 Mini control protocol is fixed"),
    ]

    # ── MAVLink Bridge ────────────────────────────────────────────────
    # Forwards FC MAVLink to <gcs_ip>:14550 over Tailscale so the dashboard
    # can use the 4G link as primary telemetry (SiK becomes auto fallback).
    mavlink_bridge = Node(
        package="drone_vision",
        executable="mavlink_bridge",
        name="mavlink_bridge",
        parameters=[{
            "connection_string":   LaunchConfiguration("connection_string"),
            "baud_rate":           LaunchConfiguration("baud_rate"),
            "stream_hz":           10,
            "heartbeat_hz":        1.0,
            "gcs_forward_ip":      LaunchConfiguration("gcs_ip"),
            "gcs_forward_port":    14550,
            "gcs_forward_listen":  14551,
        }],
        output="screen",
    )

    # ── RTSP Camera ───────────────────────────────────────────────────
    rtsp_camera = Node(
        package="drone_vision",
        executable="rtsp_camera",
        name="rtsp_camera",
        parameters=[{
            "rtsp_url":           LaunchConfiguration("rtsp_url"),
            "image_topic":        "/drone/camera_raw",
            "publish_compressed": True,
            "compressed_every_n": 3,
            "pipeline":           LaunchConfiguration("gst_pipeline"),
        }],
        output="screen",
    )

    # ── Gimbal Controller ─────────────────────────────────────────────
    # command_rate_hz is gated by gimbal_active: at 50 Hz when the real
    # gimbal is wired (visual_servo needs a fast outbound stream), at 10 Hz
    # when only providing /gimbal/state to geo_localiser.
    gimbal_controller = Node(
        package="drone_vision",
        executable="gimbal_controller",
        name="gimbal_controller",
        parameters=[{
            "gimbal_host":     LaunchConfiguration("gimbal_host"),
            "gimbal_port":     LaunchConfiguration("gimbal_port"),
            "backend":         LaunchConfiguration("gimbal_backend"),
            "command_rate_hz": PythonExpression([
                "50.0 if '", LaunchConfiguration("gimbal_active"),
                "'.lower() == 'true' else 10.0",
            ]),
        }],
        output="screen",
    )

    # ── Visual Servo (50 Hz pixel error republisher) ─────────────────
    # Only useful when the gimbal can actually move — gated by gimbal_active.
    visual_servo = Node(
        package="drone_vision",
        executable="visual_servo",
        name="visual_servo",
        condition=IfCondition(LaunchConfiguration("gimbal_active")),
        output="screen",
    )

    # ── Person Detector ───────────────────────────────────────────────
    # Reads /drone/camera_raw directly — the physical gimbal performs the
    # stabilisation that gimbal_sim used to fake via ROI cropping.
    existing_pp = os.environ.get("PYTHONPATH", "")
    detector_pythonpath = f"{VENV_SITE}:{existing_pp}" if existing_pp else VENV_SITE

    person_detector = Node(
        package="drone_vision",
        executable="person_detector",
        name="person_detector",
        parameters=[{
            "image_topic":          "/drone/camera_raw",
            "confidence_threshold": LaunchConfiguration("confidence"),
            "backend":              LaunchConfiguration("backend"),
            "process_every_n":      2,
            "imgsz":                320,
        }],
        additional_env={"PYTHONPATH": detector_pythonpath},
        output="screen",
    )

    # ── Geo-localiser ─────────────────────────────────────────────────
    geo_localiser = Node(
        package="drone_vision",
        executable="geo_localiser",
        name="geo_localiser",
        parameters=[{
            "image_width":    320,
            "image_height":   240,
            "confidence_min": 0.45,
        }],
        output="screen",
    )

    # ── GCS Link ──────────────────────────────────────────────────────
    gcs_link = Node(
        package="drone_vision",
        executable="gcs_link",
        name="gcs_link",
        parameters=[{
            "gcs_ip":            LaunchConfiguration("gcs_ip"),
            "gcs_port":          LaunchConfiguration("gcs_port"),
            "cluster_radius_m":  5.0,
            "update_interval_s": 2.0,
            "confidence_min":    0.50,
        }],
        output="screen",
    )

    return LaunchDescription([
        *args,
        mavlink_bridge,
        rtsp_camera,
        gimbal_controller,
        visual_servo,
        person_detector,
        geo_localiser,
        gcs_link,
    ])
