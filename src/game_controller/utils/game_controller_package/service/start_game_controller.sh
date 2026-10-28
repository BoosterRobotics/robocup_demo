#!/bin/bash

source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/robocup/service/fastdds.xml
source /opt/robocup/install/setup.bash
ros2 launch game_controller launch.py
