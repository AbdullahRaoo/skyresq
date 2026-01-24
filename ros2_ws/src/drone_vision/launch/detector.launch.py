"""
Launch file for person detector node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for person detector"""
    
    # Declare arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='yolov8n.pt',
        description='YOLO model to use (yolov8n.pt, yolov8s.pt, etc.)'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence',
        default_value='0.5',
        description='Detection confidence threshold (0.0-1.0)'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda:0',
        description='Device for inference (cuda:0, cpu, etc.)'
    )
    
    show_preview_arg = DeclareLaunchArgument(
        'show_preview',
        default_value='false',
        description='Show OpenCV preview window'
    )
    
    # Person detector node
    detector_node = Node(
        package='drone_vision',
        executable='person_detector',
        name='person_detector',
        output='screen',
        parameters=[{
            'model': LaunchConfiguration('model'),
            'confidence': LaunchConfiguration('confidence'),
            'device': LaunchConfiguration('device'),
            'show_preview': LaunchConfiguration('show_preview'),
        }],
        remappings=[
            # Remap if your camera publishes to a different topic
            # ('/camera/image_raw', '/your_camera/image_raw'),
        ]
    )
    
    return LaunchDescription([
        model_arg,
        confidence_arg,
        device_arg,
        show_preview_arg,
        detector_node,
    ])
