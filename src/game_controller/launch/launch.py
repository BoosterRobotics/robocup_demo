# coding: utf8

import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package ='game_controller',
            executable='game_controller_node',
            name='game_controller',
            output='screen',
            parameters=[
                {
                    # 监听的端口，默认为 GameController 广播端口 3838
                    "port": 3838,

                    # 是否开启 IP 白名单，开启后会忽略非白名单 IP 发的广播消息，默认不应该开启
                    "enable_ip_white_list": True,

                    # 只接收指定
                    "ip_white_list": [
                        "192.168.50.217",
                    ],
                }
            ]
        ),
    ])
