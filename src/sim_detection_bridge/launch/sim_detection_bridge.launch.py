import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取配置文件路径
    sim_detection_bridge_dir = get_package_share_directory('sim_detection_bridge')
    config_file = os.path.join(sim_detection_bridge_dir, 'config', 'sim_detection_bridge.yaml')
    
    # 声明启动参数 - robot_name 由启动脚本传入
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot0',
        description='Name of the robot'
    )
    
    return LaunchDescription([
        robot_name_arg,
        Node(
            package='sim_detection_bridge',
            executable='sim_detection_bridge_node',
            name='sim_detection_bridge',
            output='screen',
            # 从配置文件和启动参数中加载参数
            parameters=[
                config_file,
                {'robot.robot_name': LaunchConfiguration('robot_name')}
            ],
        ),
    ])
