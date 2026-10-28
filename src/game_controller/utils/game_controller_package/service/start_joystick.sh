#!/bin/bash

source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/robocup/service/fastdds.xml
source /opt/robocup/install/setup.bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0
