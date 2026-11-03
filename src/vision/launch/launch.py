import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def handle_configuration(context, *args, **kwargs):
    default_config_path = PathJoinSubstitution([FindPackageShare('vision'), 'config']).perform(context)

    user_cfg_path = LaunchConfiguration('vision_config_path').perform(context).strip()
    # vision.yaml is the complete local runtime template. A robot-side or
    # user-supplied vision.yaml may override only camera/calibration sections.
    config_file = os.path.join(default_config_path, 'vision.yaml')
    config_override_files = []
    if user_cfg_path:
        if os.path.isfile(user_cfg_path):
            config_override_files.append(user_cfg_path)
        else:
            cand = user_cfg_path.rstrip('/')
            if os.path.isfile(os.path.join(cand, 'vision.yaml')):
                config_override_files.append(os.path.join(cand, 'vision.yaml'))
            else:
                print(f"[vision launch] warning: '{user_cfg_path}' is neither a YAML file nor a directory containing vision.yaml; fallback to {default_config_path}")
    show_det = LaunchConfiguration('show_det')
    show_seg = LaunchConfiguration('show_seg')
    save_data = LaunchConfiguration('save_data')
    save_depth = LaunchConfiguration('save_depth')
    offline_mode = LaunchConfiguration('offline_mode')
    save_fps = LaunchConfiguration('save_fps')
    detection_model_path = LaunchConfiguration('detection_model_path')
    segmentation_model_path = LaunchConfiguration('segmentation_model_path')
    camera_type = LaunchConfiguration('camera_type')

    return [
        Node(
            package='vision',
            executable='vision_node',
            name='vision_node',
            output='screen',
            arguments=[config_file] + config_override_files,
            parameters=[{
                'offline_mode': offline_mode,
                'show_det': show_det,
                'show_seg': show_seg,
                'save_data': save_data,
                'save_depth': save_depth,
                'save_fps': save_fps,
                'detection_model_path': detection_model_path,
                'segmentation_model_path': segmentation_model_path,
                'camera_type': camera_type
            }]
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vision_config_path',
            default_value='',
            description='Optional config directory or YAML file; only camera/calibration override package defaults (empty => package defaults only)'
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
            'camera_type',
            default_value="",
            description="param to override camera.type. will not override if empty"
        ),
        OpaqueFunction(function=handle_configuration)
    ])
