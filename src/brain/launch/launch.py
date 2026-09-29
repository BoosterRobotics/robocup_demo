# coding: utf8

import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def handle_configuration(context, *args, **kwargs):
    vision_config_path = context.perform_substitution(LaunchConfiguration('vision_config_path')).strip()
    default_vision_dir = os.path.join(get_package_share_directory('vision'), 'config')

    # vision.yaml is the complete local runtime template. A robot-side or
    # user-supplied vision.yaml is optional and is merged on top of it.
    vision_template_file = os.path.join(default_vision_dir, 'vision.yaml')
    calibration_override_file = ''
    if os.path.isfile(vision_config_path):
        calibration_override_file = vision_config_path
    elif vision_config_path:
        user_main_file = os.path.join(vision_config_path, 'vision.yaml')
        if os.path.isfile(user_main_file):
            calibration_override_file = user_main_file
            user_local_file = os.path.join(vision_config_path, 'vision_local.yaml')
            if os.path.isfile(user_local_file):
                # Brain currently accepts one local override. The generated
                # vision.yaml is the calibration result; retain it as the
                # selected override when both files are present.
                print(f"[brain launch] ignoring optional secondary override '{user_local_file}'; vision.yaml takes precedence")

    if not os.path.isfile(vision_template_file):
        print(f"[brain launch] runtime template '{vision_template_file}' is missing")

    if vision_config_path and not calibration_override_file:
        print(f"[brain launch] no calibration YAML found under '{vision_config_path}'; using template defaults")

    vision_config_file = vision_template_file
    vision_config_local_file = calibration_override_file
    if not vision_config_local_file:
        print(f"[brain launch] no calibration override selected; using '{vision_config_file}' only")

    config_path = os.path.join(os.path.dirname(__file__), '../config')
    config_file = os.path.join(config_path, 'config.yaml')
    config_local_file = os.path.join(config_path, 'config_local.yaml')
    parameter_files = [config_file]
    if os.path.isfile(config_local_file):
        parameter_files.append(config_local_file)
    else:
        print(f"[brain launch] optional local override '{config_local_file}' not present")

    behavior_trees_dir = os.path.join(os.path.dirname(__file__), '../behavior_trees')
    def make_tree_path(name):
        if not name.endswith('.xml'):
            name += '.xml'
        return os.path.join(behavior_trees_dir, name)
    tree = context.perform_substitution(LaunchConfiguration('tree'))
    tree_path = make_tree_path(tree)

    # Launch arguments override matching fields from config_file so common settings can be changed without editing config.yaml.
    config = {
            # Behavior tree to load.
            "tree_file_path": tree_path,
            "vision_config_path": vision_config_file,
            "vision_config_local_path": vision_config_local_file,
    }

    # Normalize optional string arguments.
    start_pos = context.perform_substitution(LaunchConfiguration('pos'))
    if start_pos:
        config['game.player_start_pos'] = start_pos
    role = context.perform_substitution(LaunchConfiguration('role'))
    if role:
        config['game.player_role'] = role

    sim = context.perform_substitution(LaunchConfiguration('sim'))
    if sim in ['true', 'True', '1']:
        config['use_sim_time'] = True

    disableLog = context.perform_substitution(LaunchConfiguration('disable_log'))
    if disableLog in ['true', 'True', '1']:
        config['rerunLog.enable_file'] = False
        config['rerunLog.enable_tcp'] = False
        config['odomLog.enable'] = False

    disableCom = context.perform_substitution(LaunchConfiguration('disable_com'))
    if disableCom in ['true', 'True', '1']:
        config['enable_com'] = False

    return [
        Node(
            package ='brain',
            executable='brain_node',
            output='screen',
            parameters=[
                *parameter_files,
                config
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vision_config_path',
            default_value=os.path.join(get_package_share_directory('vision'), 'config'),
            description='Vision config directory or standalone calibration YAML override file'
        ),
        # Declare every parameter exposed through `ros2 launch brain launch.py name:=value` and process it in handle_configuration.
        DeclareLaunchArgument(
            'tree',
            default_value='game.xml',
            description='Specify behavior tree file name. DO NOT include full path, file should be in src/brain/config/behavior_trees'
        ),
        DeclareLaunchArgument(
            'pos',
            default_value='',
            description='Override game.player_start_pos from config.yaml, for example pos:=left.'
        ),
        DeclareLaunchArgument(
            'role',
            default_value='',
            description='Override game.player_role from config.yaml, for example role:=striker.'
        ),
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Run in simulation mode.'
        ),
        DeclareLaunchArgument(
            'disable_log',
            default_value='false',
            description='Force-disable file logging.'
        ),
        DeclareLaunchArgument(
            'disable_com',
            default_value='false',
            description='Force-disable teammate communication.'
        ),
        OpaqueFunction(function=handle_configuration) # Resolve launch arguments and construct the node.
    ])
