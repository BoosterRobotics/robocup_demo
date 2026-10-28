#!/bin/bash
echo "[START VISION]"
cd `dirname $0`
cd ..

source ./install/setup.bash
#export FASTRTPS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml

# ros2 launch vision launch.py show_det:=true > vision_d.log 2>&1
ros2 launch vision launch.py show_det:=false save_data:=false
