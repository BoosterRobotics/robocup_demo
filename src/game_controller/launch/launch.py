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
                    "port": 3838,

                    "enable_ip_white_list": True,

                    "ip_white_list": [
                        "192.168.10.10",
                    ],
                }
            ]
        ),
    ])
