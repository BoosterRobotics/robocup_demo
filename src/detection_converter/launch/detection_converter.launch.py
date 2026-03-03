#!/usr/bin/env python3
"""
Launch file for Detection Converter Node
Launch the detection message conversion node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description"""
    
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/robot0_rgbd_camera/detections',
        description='Input topic name for JSON detection messages'
    )
    
    detections_topic_arg = DeclareLaunchArgument(
        'detections_topic',
        default_value='/booster_soccer/detection',
        description='Output topic name for Detections messages'
    )

    line_segments_topic_arg = DeclareLaunchArgument(
        'line_segments_topic',
        default_value='/booster_soccer/line_segments',
        description='Output topic name for LineSegments messages'
    )
    
    # Create detection converter node
    detection_converter_node = Node(
        package='detection_converter',
        executable='detection_converter_node.py',
        name='detection_converter',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'line_segments_topic': LaunchConfiguration('line_segments_topic'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        input_topic_arg,
        detections_topic_arg,
        line_segments_topic_arg,
        detection_converter_node,
    ])
