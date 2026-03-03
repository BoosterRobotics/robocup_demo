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
                    # The port to listen on, default is the GameController broadcast port 3838
                    "port": 3838,

                    # control if to enable IP white list check, default False, if True, only packets from IPs in ip_white_list will be accepted
                    "enable_ip_white_list": False,

                    # only accept packets from these IP addresses if enable_ip_white_list is True, default empty
                    "ip_white_list": [
                        "127.0.0.1",
                    ],
                }
            ]
        ),
    ])
