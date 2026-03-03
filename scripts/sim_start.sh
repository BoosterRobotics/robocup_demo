#!/bin/bash

cd `dirname $0`
cd ..

echo "[STOP EXISTING NODES (IF ANY), TO AVOID CONFILICT]"
./scripts/stop.sh

source /opt/ros/humble/setup.bash
source ./install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml

echo "[START ROBOCUP NODES]"
echo "[START VISION]"

nohup ros2 launch vision launch.py sim:=true > vision.log 2>&1 &
# nohup ros2 launch detection_converter detection_converter.launch.py > vision.log 2>&1 &
echo "[START BRAIN]"
nohup ros2 launch brain launch.py sim:=true "$@"  > brain.log 2>&1 &
echo "[START GAME_CONTROLLER]"
nohup ros2 launch game_controller launch.py sim:=true > game_controller.log 2>&1 &
echo "[DONE]"
