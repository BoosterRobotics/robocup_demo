import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def handle_configuration(context, *args, **kwargs):
    config_path = os.path.join(os.path.dirname(__file__), '../config')
    config_file = os.path.join(config_path, 'vision.yaml') 
    config_sim_file = os.path.join(config_path, 'vision_sim.yaml')
    config_local_file = os.path.join(config_path, 'vision_local.yaml') 

    config = {}
    sim = context.perform_substitution(LaunchConfiguration('sim'))
    if sim in ['true', 'True', '1']:
        config['use_sim_time'] = True
        config_file = config_sim_file

    robot_name = context.perform_substitution(LaunchConfiguration('robot_name'))
    if not robot_name == '':
        config['robot_name'] = robot_name

    return [
        Node(
            package='vision',
            executable='vision_node',
            name='vision_node',
            output='screen',
            arguments=[config_file, config_local_file],
            parameters=[config]
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', 
            default_value='false',
            description='if it is in sim'
        ),

        DeclareLaunchArgument(
            'robot_name', 
            default_value='',
            description='name of the robot'
        ),
        OpaqueFunction(function=handle_configuration)
    ])