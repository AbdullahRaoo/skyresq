import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # PX4 Execution Command
    px4_dir = os.path.expanduser('~/Drone/PX4-Autopilot')
    px4_cmd = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500'],
        cwd=px4_dir,
        output='screen'
    )

    # ROS-Gazebo Bridge
    # Must include topics for Offboard Control:
    # - /fmu/in/offboard_control_mode
    # - /fmu/in/trajectory_setpoint
    # - /fmu/in/vehicle_command
    # - /fmu/out/vehicle_status
    # - /fmu/out/vehicle_local_position
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # We need standard PX4 bridge but for simplicity we rely on the 
            # internal micro-xrce-dds-agent which PX4 launches automatically in SITL.
            # WAIT! The 'ros_gz_bridge' is for Gazebo<->ROS.
            # PX4<->ROS uses Micro-XRCE-DDS Agent.
            # In SITL, the "make px4_sitl" command STARTS the agent automatically? 
            # NO, usually we need to start it unless we use uXRCE-DDS middleware directly.
            # Actually, standard ROS 2 / PX4 setup uses "MicroXRCEAgent udp4 -p 8888".
            # Let's add that here explicitly just in case, or rely on external agent.
            # For this specific setup (ros_gz_bridge), we are bridging Gazebo visual topics.
            # BUT efficient PX4 control uses the dedicated agent.
        ],
        output='screen'
    )
    
    # FIX: We need the Micro-XRCE-DDS Agent to bridge ROS 2 topics to PX4.
    # The 'ros_gz_bridge' only handles Gazebo (Sim) to ROS.
    # PX4 (Flight Controller) to ROS communicates via Micro-XRCE-DDS.
    xrce_agent = ExecuteProcess(
        cmd=['/home/abdullah/Drone/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # Mission Node
    mission = Node(
        package='drone_vision',
        executable='mission_node',
        name='mission_node',
        output='screen'
    )

    return LaunchDescription([
        xrce_agent,
        px4_cmd,
        mission
    ])
