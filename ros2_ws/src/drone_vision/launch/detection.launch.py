import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # PX4 Execution Command
    # Runs the specific target built earlier
    px4_dir = os.path.expanduser('~/Drone/PX4-Autopilot')
    px4_cmd = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500'],
        cwd=px4_dir,
        output='screen'
    )

    # ROS-Gazebo Bridge
    # Detailed bridge for Camera + Control + Odometry
    # GZ_TO_ROS: Camera, Odom, Battery
    # ROS_TO_GZ: cmd_vel
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Camera
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # Control (cmd_vel -> Twist)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry (Ground Truth)
            '/model/x500/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Battery/Status
            '/model/x500/battery/0/state@sensor_msgs/msg/BatteryState[gz.msgs.BatteryState'
        ],
        output='screen'
    )

    # Person Detector Node
    detector = Node(
        package='drone_vision',
        executable='person_detector',
        name='person_detector',
        output='screen'
    )

    return LaunchDescription([
        px4_cmd,
        bridge,
        detector
    ])
