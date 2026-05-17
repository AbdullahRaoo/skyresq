#!/usr/bin/env python3
"""
SITL CORE launch — autonomy/command validation WITHOUT the camera half.

Brings up only the software that flies the mission:
  mavlink_bridge          ArduPilot SITL (tcp:5760) ↔ /vehicle/* + intent→FC
  sar_orchestrator        the REAL state machine (IDLE→…→RTL→DONE)
  sim_payload             /payload/cmd ↔ /payload/state (no Pi GPIO)
  sim_detection_injector  stands in for camera→YOLO→geo_localiser by
                          publishing a geo-locked survivor on /target/world

Nothing in the autonomy or FC-command path is mocked — only the
camera-to-world stage the injector replaces. This is the portable
subset that also runs on the 8 GB field laptop.

Run ArduCopter SITL first (headless, no GUI terminal):
  ~/ardupilot/build/sitl/bin/arducopter -w --model + --speedup 1 \
    --defaults ~/ardupilot/Tools/autotest/default_params/copter.parm \
    -I0 --home -35.363261,149.16523,584.0,353.0

Then:
  ros2 launch drone_vision sitl_core.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument('sitl_connection',
                              default_value='tcp:127.0.0.1:5760'),
        DeclareLaunchArgument('offset_north_m', default_value='35.0'),
        DeclareLaunchArgument('offset_east_m', default_value='0.0'),
        DeclareLaunchArgument('inject_auto_delay_s', default_value='0.0',
                              description='>0 = auto-inject N s after first '
                                          'GPS fix; 0 = wait for /sim/inject'),
    ]

    mavlink_bridge = Node(
        package='drone_vision', executable='mavlink_bridge',
        name='mavlink_bridge', output='screen',
        parameters=[{
            'connection_string': LaunchConfiguration('sitl_connection'),
            'stream_hz': 10,
            'heartbeat_hz': 1.0,
        }],
    )

    sar_orchestrator = Node(
        package='drone_vision', executable='sar_orchestrator',
        name='sar_orchestrator', output='screen',
    )

    sim_payload = Node(
        package='drone_vision', executable='sim_payload',
        name='sim_payload', output='screen',
    )

    sim_injector = Node(
        package='drone_vision', executable='sim_detection_injector',
        name='sim_detection_injector', output='screen',
        parameters=[{
            'offset_north_m': LaunchConfiguration('offset_north_m'),
            'offset_east_m': LaunchConfiguration('offset_east_m'),
            'auto_start_delay_s': LaunchConfiguration('inject_auto_delay_s'),
        }],
    )

    return LaunchDescription([
        *args, mavlink_bridge, sar_orchestrator, sim_payload, sim_injector,
    ])
