# coding: utf8

import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def handle_configuration(context, *args, **kwargs):
    config_path = os.path.join(os.path.dirname(__file__), '../config')
    config_file = os.path.join(config_path, 'brain.yaml') 
    config_local_file = os.path.join(config_path, 'brain_local.yaml') 

    behavior_trees_dir = os.path.join(os.path.dirname(__file__), '../behavior_trees')
    def make_tree_path(name):
        if not name.endswith('.xml'):
            name += '.xml'
        return os.path.join(behavior_trees_dir, name)
    tree = context.perform_substitution(LaunchConfiguration('tree'))
    tree_path = make_tree_path(tree)

    config = {
            "tree_file_path": tree_path,
    }
    start_pos = context.perform_substitution(LaunchConfiguration('pos'))
    if not start_pos == '':
        config['game.player_start_pos'] = start_pos
    role = context.perform_substitution(LaunchConfiguration('role'))
    if not role == '':
        config['game.player_role'] = role

    sim = context.perform_substitution(LaunchConfiguration('sim'))
    if sim in ['true', 'True', '1']:
        config['use_sim_time'] = True

    robot_name = context.perform_substitution(LaunchConfiguration('robot_name'))
    if not robot_name == '':
        config['robot.robot_name'] = robot_name

    return [
        Node(
            package ='brain',
            executable='brain_node',
            name='brain_node',
            output='screen',
            parameters=[
                config_file,
                config_local_file,
                config
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tree', 
            default_value='game.xml',
            description='Specify behavior tree file name. DO NOT include full path, file should be in src/brain/config/behavior_trees'
        ),
        DeclareLaunchArgument(
            'pos', 
            default_value='',
            description='If you need to override the game.player_start_pos in the config.yaml, you can specify the parameter pos:=left when launching.'
        ),
        DeclareLaunchArgument(
            'role', 
            default_value='',
            description='If you need to override the game.player_role in the config.yaml, you can specify the parameter role:=striker when launching'
        ),
        DeclareLaunchArgument(
            'sim', 
            default_value='false',
            description='If it is in sim-env'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='',
            description='Name of the robot to use in multi-robot setup'
        ),

        OpaqueFunction(function=handle_configuration)
    ])
