#!/usr/bin/env python3
"""
SITL + Gazebo full-fidelity launch — pixel-to-drop validation.

Replaces the deterministic sim_detection_injector path with a REAL
camera → YOLO person_detector → geo_localiser → orchestrator chain.
The orchestrator's autonomy and the FC command path are identical to
sitl_core.launch.py; only the camera-to-world stage now uses an
actual rendered image.

Prereqs (start in this order outside this launch file):

  1. ardupilot_gazebo plugin + sar_world.sdf:
       export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
       export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:\
$HOME/ardupilot_gazebo/worlds
       gz sim -r ros2_ws/src/drone_vision/gazebo/sar_world.sdf

  2. ArduCopter SITL connected to the plugin's JSON interface:
       ~/ardupilot/build/sitl/bin/arducopter -w --model JSON --slave 0 \
         --defaults ~/ardupilot/Tools/autotest/default_params/copter.parm \
         -I0 --home -35.363262,149.165237,584,0

  3. Force the gimbal to nadir (one shot):
       gz topic -t /gimbal/cmd_pitch -m gz.msgs.Double -p 'data: -1.57'

Then:
       ros2 launch drone_vision sitl_gazebo.launch.py

Nodes
-----
  mavlink_bridge       SITL tcp:5760 ↔ /vehicle/* + intent→FC
  ros_gz_bridge        gz camera image → /drone/camera_raw
  sim_gimbal_state     static /gimbal/state = pitch -90°, yaw 0° (nadir)
  person_detector      REAL YOLO on /drone/camera_raw
  geo_localiser        pixel → ray → ground → lat/lon (Gazebo intrinsics)
  sar_orchestrator     IDLE → … → DROP → RTL → DONE (unchanged from flight)
  sim_payload          /payload/cmd ↔ /payload/state (no Pi GPIO)
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

CONFIG_DIR = os.path.expanduser('~/Drone/ros2_ws/src/drone_vision/config')
GZ_INTRINSICS = os.path.join(CONFIG_DIR, 'camera_intrinsics_gz.yaml')
CAMERA_GZ_TOPIC = (
    '/world/sar_world/model/iris_with_gimbal/model/gimbal/link/'
    'pitch_link/sensor/camera/image'
)
VENV_SITE = os.path.expanduser('~/Drone/venv/lib/python3.12/site-packages')


def generate_launch_description():
    args = [
        DeclareLaunchArgument('sitl_connection',
                              default_value='tcp:127.0.0.1:5760'),
        DeclareLaunchArgument('confidence', default_value='0.40'),
        DeclareLaunchArgument('imgsz', default_value='640'),
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

    # Bridge the Gazebo camera image into ROS at /drone/camera_raw.
    # ros_gz_bridge syntax: <gz_topic>@<ros_type>[<gz_type>
    gz_camera_bridge = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             f'{CAMERA_GZ_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image',
             '--ros-args', '-r', f'{CAMERA_GZ_TOPIC}:=/drone/camera_raw'],
        output='screen',
    )

    sim_gimbal_state = Node(
        package='drone_vision', executable='sim_gimbal_state',
        name='sim_gimbal_state', output='screen',
        parameters=[{'pitch_deg': -90.0, 'yaw_deg': 0.0}],
    )

    person_detector = Node(
        package='drone_vision', executable='person_detector',
        name='person_detector', output='screen',
        parameters=[{
            'image_topic':          '/drone/camera_raw',
            'confidence_threshold': LaunchConfiguration('confidence'),
            'process_every_n':      2,
            'imgsz':                LaunchConfiguration('imgsz'),
            'backend':              'ultralytics',
        }],
        additional_env={
            'PYTHONPATH': f"{VENV_SITE}:{os.environ.get('PYTHONPATH','')}",
            'YOLO_AUTOINSTALL': 'false',
            'YOLO_VERBOSE': 'false',
        },
    )

    geo_localiser = Node(
        package='drone_vision', executable='geo_localiser',
        name='geo_localiser', output='screen',
        parameters=[{
            'intrinsics_file': GZ_INTRINSICS,
            'image_width':     640,
            'image_height':    480,
            'min_agl_m':       2.0,
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

    return LaunchDescription([
        *args,
        mavlink_bridge,
        gz_camera_bridge,
        sim_gimbal_state,
        person_detector,
        geo_localiser,
        sar_orchestrator,
        sim_payload,
    ])
