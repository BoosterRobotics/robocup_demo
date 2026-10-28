#!/bin/bash
echo "[START BRAIN]"

cd `dirname $0`
cd ..

source ./install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml

ros2 launch brain launch.py "$@"
