# coding: utf8

import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def handle_configuration(context, *args, **kwargs):
    vision_config_dir = context.perform_substitution(LaunchConfiguration('vision_config_path')).strip()
    default_vision_dir = os.path.join(get_package_share_directory('vision'), 'config')

    user_main_file = os.path.join(vision_config_dir, 'vision.yaml')
    reason = ""
    if not vision_config_dir:
        reason = "empty vision_config_path"
    elif not os.path.isdir(vision_config_dir):
        reason = f"directory '{vision_config_dir}' not found"
    elif not os.path.isfile(user_main_file):
        reason = f"'{user_main_file}' missing"

    if reason:
        print(f"[brain launch] {reason}, fallback to '{default_vision_dir}'")
        vision_config_dir = default_vision_dir

    vision_config_file = os.path.join(vision_config_dir, 'vision.yaml')
    vision_config_local_file = os.path.join(vision_config_dir, 'vision_local.yaml')
    if not os.path.isfile(vision_config_local_file):
        print(f"[brain launch] optional local override '{vision_config_local_file}' not present")

    config_path = os.path.join(os.path.dirname(__file__), '../config')
    config_file = os.path.join(config_path, 'config.yaml') 
    config_local_file = os.path.join(config_path, 'config_local.yaml') 

    behavior_trees_dir = os.path.join(os.path.dirname(__file__), '../behavior_trees')
    def make_tree_path(name):
        if not name.endswith('.xml'):
            name += '.xml'
        return os.path.join(behavior_trees_dir, name)
    tree = context.perform_substitution(LaunchConfiguration('tree'))
    tree_path = make_tree_path(tree)

    # 这里的 config 覆盖 config_file 中相同字段(如有) 用于在 launch 时快速指定参数, 而不需要频繁修改 config.yaml
    config = {
            # 加载哪个行为树文件
            "tree_file_path": tree_path,
            "vision_config_path": vision_config_file,
            "vision_config_local_path": vision_config_local_file,
    }

    # 简化字符串判空
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
                config
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vision_config_path',
            default_value=os.path.join(get_package_share_directory('vision'), 'config'),
            description='Directory containing vision.yaml / vision_local.yaml'
        ),
        # 需要可以通过 ros2 launch brain launch.py param:=value 形式提供的参数, 需要在这里用 DeclarelaunchArgument 声明, 然后在 handle_configuration 处理
        DeclareLaunchArgument(
            'tree', 
            default_value='game.xml',
            description='Specify behavior tree file name. DO NOT include full path, file should be in src/brain/config/behavior_trees'
        ),
        DeclareLaunchArgument(
            'pos', 
            default_value='',
            description='如果需要覆盖 config.yaml 中的 game.player_start_pos, 可以在 launch 时指定参数 pos:=left'
        ),
        DeclareLaunchArgument(
            'role', 
            default_value='',
            description='如果需要覆盖 config.yaml 中的 game.player_role, 可以在 launch 时指定参数 role:=striker'
        ),
        DeclareLaunchArgument(
            'sim', 
            default_value='false',
            description='是否在仿真中'
        ),
        DeclareLaunchArgument(
            'disable_log', 
            default_value='false',
            description='强制禁用在文件中记录日志'
        ),
        DeclareLaunchArgument(
            'disable_com', 
            default_value='false',
            description='强制禁用开启通信'
        ),
        OpaqueFunction(function=handle_configuration) # 转到 handle_configuration 中继续处理
    ])
