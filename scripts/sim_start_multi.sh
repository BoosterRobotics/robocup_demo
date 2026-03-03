#!/bin/bash

cd `dirname $0`
cd ..

echo "[STOP EXISTING NODES (IF ANY), TO AVOID CONFLICT]"
./scripts/stop.sh

source /opt/ros/humble/setup.bash
source ./install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml

echo "[START ROBOCUP NODES]"
echo "[START GAME_CONTROLLER]"
nohup ros2 launch game_controller launch.py sim:=true > game_controller.log 2>&1 &

# define robot configurations: name, team_id, player_id, role
declare -a ROBOTS=(
    "robot0:29:1:goal_keeper"
    "robot1:29:2:striker"
    "robot2:29:3:striker"

    "robot3:30:1:goal_keeper"
    "robot4:30:2:striker"
    "robot5:30:3:striker"

)

# loop through each robot configuration and start their nodes
for robot_config in "${ROBOTS[@]}"; do
    IFS=':' read -r robot_name team_id player_id role <<< "$robot_config"
    
    echo "[START PLAYER: $robot_name (Team: $team_id, Player: $player_id, Role: $role)]"
    
    # start detection_converter
    nohup ros2 launch detection_converter detection_converter.launch.py \
        input_topic:="/camera/${robot_name}_rgbd_camera/detections" \
        detections_topic:="/booster_soccer/detection/$robot_name" \
        line_segments_topic:="/booster_soccer/line_segments/$robot_name" \
        > "${robot_name}_vision.log" 2>&1 &
    
    # start brain
    nohup ros2 launch brain launch.py \
        sim:=true \
        team_id:=$team_id \
        player_id:=$player_id \
        role:=$role \
        robot_name:=$robot_name \
        disable_com:=false \
        "$@" > "${robot_name}_brain.log" 2>&1 &
done

echo "[DONE]"
