# coding: utf8

import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def handle_configuration(context, *args, **kwargs):
    # get launch argument 'vision_config_path' and construct paths to vision.yaml and vision_local.yaml
    vision_config_dir = context.perform_substitution(LaunchConfiguration('vision_config_path'))
    vision_config_file = os.path.join(vision_config_dir, 'vision.yaml')
    vision_config_local_file = os.path.join(vision_config_dir, 'vision_local.yaml')

    config_path = os.path.join(os.path.dirname(__file__), '../config')
    config_file = os.path.join(config_path, 'config.yaml') 
    config_local_file = os.path.join(config_path, 'config_local.yaml') 
    config_home_file = os.path.join(os.path.expanduser('~'), 'agents/booster_soccer/brain.yaml') 


    behavior_trees_dir = os.path.join(os.path.dirname(__file__), '../behavior_trees')
    def make_tree_path(name):
        if not name.endswith('.xml'):
            name += '.xml'
        return os.path.join(behavior_trees_dir, name)
    tree = context.perform_substitution(LaunchConfiguration('tree'))
    tree_path = make_tree_path(tree)

    # if there are parameters that are commonly changed for different matches, you can add them here to make it more convenient to modify when launching. For example, you can specify player role, team id, player id, etc. when launching, instead of modifying config.yaml every time before launching.
    config = {
            # The behavior tree file to load, should be placed in src/brain/config/behavior_trees
            "tree_file_path": tree_path,
            "vision_config_path": vision_config_file,
            "vision_config_local_path": vision_config_local_file,
    }

    role = context.perform_substitution(LaunchConfiguration('role'))
    if not role == '':
        config['game.player_role'] = role
    
    team_id = context.perform_substitution(LaunchConfiguration('team_id'))
    if not team_id == '':
        config['game.team_id'] = int(team_id)

    player_id = context.perform_substitution(LaunchConfiguration('player_id'))
    if not player_id == '':
        config['game.player_id'] = int(player_id)

    robot_name = context.perform_substitution(LaunchConfiguration('robot_name'))
    if not robot_name == '':
        config['robot.robot_name'] = robot_name

    agent_mode = context.perform_substitution(LaunchConfiguration('agent_mode'))
    if agent_mode in ['true', 'True', '1']:
        config['game.agent_mode'] = True

    sim = context.perform_substitution(LaunchConfiguration('sim'))
    if sim in ['true', 'True', '1']:
        config['use_sim_time'] = True


    disableCom = context.perform_substitution(LaunchConfiguration('disable_com'))
    if disableCom in ['true', 'True', '1']:
        config['enable_com'] = False

    return [
        Node(
            package ='brain',
            executable='brain_node',
            output='screen',
            parameters=[
                config_file,
                config_local_file,
                config_home_file,
                config
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vision_config_path',
            default_value=os.path.join(os.path.dirname(__file__), '../../../../vision/share/vision/config'),
            description='Directory containing vision.yaml and vision_local.yaml'
        ),
        # Parameters that can be provided through ros2 launch brain launch.py param:=value need to be declared here using DeclareLaunchArgument, and then handled in handle_configuration
        DeclareLaunchArgument(
            'tree', 
            default_value='game.xml',
            description='Specify behavior tree file name. DO NOT include full path, file should be in src/brain/config/behavior_trees'
        ),
        DeclareLaunchArgument(
            'role', 
            default_value='',
            description='If you need to override game.player_role in config.yaml, you can specify the parameter role:=striker when launching'
        ),
        DeclareLaunchArgument(
            'team_id',
            default_value='',
            description='Set the team ID, for example team_id:=1'
        ),
        DeclareLaunchArgument(
            'player_id',
            default_value='',
            description='Set the player ID, for example player_id:=2'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='',
            description='Set the robot name, for example robot_name:=robot0'
        ),
        DeclareLaunchArgument(
            'sim', 
            default_value='false',
            description='Whether to run in simulation'

        ),
        DeclareLaunchArgument(
            'disable_com', 
            default_value='false',
            description='Force disable communication'
        ),
        DeclareLaunchArgument(
            'agent_mode', 
            default_value='false',
            description='Whether to run in agent mode. In agent mode, the robot is controlled by the APP and does not receive commands from the referee or physical controller'
        ),
        OpaqueFunction(function=handle_configuration) # Continue processing in handle_configuration
    ])
