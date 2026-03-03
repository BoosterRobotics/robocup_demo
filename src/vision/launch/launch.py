import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def handle_configuration(context, *args, **kwargs):
    default_config_path = PathJoinSubstitution([FindPackageShare('vision'), 'config']).perform(context)

    user_cfg_dir = LaunchConfiguration('vision_config_path').perform(context)
    config_path = default_config_path  
    if user_cfg_dir and user_cfg_dir.strip():
        cand = user_cfg_dir.rstrip('/')
        if os.path.exists(os.path.join(cand, 'vision.yaml')):
            config_path = cand
        else:
            print(f"[vision launch] warning: {cand}/vision.yaml not found, fallback to {default_config_path}")
    config_file = os.path.join(config_path, 'vision.yaml')
    config_local_file = os.path.join(config_path, 'vision_local.yaml')
    config_user_file = os.path.join(os.path.expanduser('~'), 'agents/booster_soccer/vision.yaml')


    show_det = LaunchConfiguration('show_det')
    show_seg = LaunchConfiguration('show_seg')
    save_data = LaunchConfiguration('save_data')
    save_depth = LaunchConfiguration('save_depth')
    offline_mode = LaunchConfiguration('offline_mode')
    save_fps = LaunchConfiguration('save_fps')
    detection_model_path = LaunchConfiguration('detection_model_path')
    segmentation_model_path = LaunchConfiguration('segmentation_model_path')
    sim = LaunchConfiguration('sim')
    color_topic = LaunchConfiguration('color_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    intrin_topic = LaunchConfiguration('intrin_topic')

    return [
        Node(
            package='vision',
            executable='vision_node',
            name='vision_node',
            output='screen',
            arguments=[config_file, config_local_file, config_user_file],
            parameters=[{
                'offline_mode': offline_mode,
                'show_det': show_det,
                'show_seg': show_seg,
                'save_data': save_data,
                'save_depth': save_depth,
                'save_fps': save_fps,
                'detection_model_path': detection_model_path,
                'segmentation_model_path': segmentation_model_path,
                'use_sim_time': sim,
                'color_topic': color_topic,
                'depth_topic': depth_topic,
                'intrin_topic': intrin_topic,
            }]
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vision_config_path',
            default_value='',
            description='Optional directory containing vision.yaml & vision_local.yaml (empty => use package default)'
        ),
        DeclareLaunchArgument(
            "offline_mode",
            default_value='false',
            description="enable offline model"
        ),
        DeclareLaunchArgument(
            "show_det",
            default_value='false',
            description="Show detection result"
        ),
        DeclareLaunchArgument(
            "show_seg",
            default_value='false',
            description="Show segmentation result"
        ),
        DeclareLaunchArgument(
            "save_data",
            default_value='true',
            description="Save recevied image data"
        ),
        DeclareLaunchArgument(
            "save_depth",
            default_value='true',
            description="Save recevied depth img data"
        ),
        DeclareLaunchArgument(
            "save_fps",
            default_value='2',
            description="Save n frames of data each second"
        ),
        DeclareLaunchArgument(
            'detection_model_path',
            default_value="",
            description="param to override detection_model.model_path. will not override if empty"
        ),
        DeclareLaunchArgument(
            'segmentation_model_path',
            default_value="",
            description="param to override segmentation_model.model_path. will not override if empty"
        ),
        DeclareLaunchArgument(
            'sim', 
            default_value='false',
            description='whether to use simulation time'
        ),
        DeclareLaunchArgument(
            'color_topic',
            default_value='',
            description='Topic for color image'
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='',
            description='Topic for depth image'
        ),
        DeclareLaunchArgument(
            'intrin_topic',
            default_value='',
            description='Topic for camera intrinsics'
        ),
        OpaqueFunction(function=handle_configuration)
    ])