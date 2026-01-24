import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Bridge configuration
    # Mapping: Gazebo Topic -> ROS Topic
    # We assume the drone camera publishes to /camera inside Gazebo
    bridge_config = """
    - topic: /camera/image_raw
      ros_type_name: sensor_msgs/msg/Image
      gz_type_name: gz.msgs.Image
      direction: GZ_TO_ROS
    - topic: /cmd_vel
      ros_type_name: geometry_msgs/msg/Twist
      gz_type_name: gz.msgs.Twist
      direction: ROS_TO_GZ
    """

    # Start Gazebo
    # For now, we launch an empty world or a sensor world. 
    # In next session, this will link to PX4 SITL.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ''}], # We could use a file, or separate args
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
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
        gz_sim,
        bridge,
        detector
    ])
