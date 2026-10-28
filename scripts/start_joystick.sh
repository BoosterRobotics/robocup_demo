#!/bin/bash
echo "[START JOYSTICK]"

cd `dirname $0`
cd ..

export FASTRTPS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml

ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0