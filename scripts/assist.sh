#!/bin/bash

cd `dirname $0`
cd ..

cd `dirname $0`
cd ..

echo "[STOP EXISTING NODES (IF ANY), TO AVOID CONFILICT]"
./scripts/stop.sh

source ./install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml

echo "[START ROBOCUP NODES]"
echo "[START VISION]"
nohup ros2 launch vision launch.py save_data:=true > vision.log 2>&1 &
echo "[START BRAIN]"
nohup ros2 launch brain launch.py tree:=assist disable_log:=true disable_com:=true  > brain.log 2>&1 &
echo "[START GAME_CONTROLLER]"
nohup ros2 launch game_controller launch.py > game_controller.log 2>&1 &
echo "[START SOUND]"
nohup ros2 run sound_play sound_play_node > sound.log 2>&1 &
echo "[DONE]"
