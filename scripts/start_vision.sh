#!/bin/bash

cd `dirname $0`
cd ..

echo "[STOP EXISTING NODES (IF ANY), TO AVOID CONFILICT]"
./scripts/stop.sh

source ./install/setup.bash
#export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml

ros2 daemon stop
ros2 daemon start

ros2 launch vision launch.py sim:=true "$@"
