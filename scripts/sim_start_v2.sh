#!/bin/bash

cd `dirname $0`
cd ..

./scripts/sim_stop_v2.sh

source ./install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml

nohup ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0 > joystick.log 2>&1 &
nohup ros2 launch game_controller launch.py > game_controller.log 2>&1 &
nohup ros2 launch brain launch.py "$@" sim:=true > brain.log 2>&1 &
nohup ros2 launch sim_detection_bridge sim_detection_bridge.launch.py "$@" > sim_detection_bridge.log 2>&1 &
