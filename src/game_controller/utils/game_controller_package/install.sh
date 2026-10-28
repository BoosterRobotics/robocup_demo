#!/bin/sh
# 获取当前脚本文件的绝对路径
script_path=$(readlink -f "$0")

echo "Script path: $script_path"
# 提取父目录路径作为根路径
root_path=$(dirname $script_path)
echo "Proj root path: $root_path"


install_path=$root_path/install
service_path=$root_path/service
system_robocup_path=/opt/robocup

# 先停止系统服务
sudo systemctl stop robocup_game_controller.service
sudo cp $service_path/robocup_game_controller.service /etc/systemd/system

sudo mkdir -p $system_robocup_path

sudo systemctl stop robocup_joystick.service
sudo cp $service_path/robocup_joystick.service /etc/systemd/system


echo "Delete old files ..."
sudo rm -rf $system_robocup_path/install || true
sudo rm -rf $system_robocup_path/service || true


echo "Install new files ..."
target_path=$system_robocup_path

sudo cp -r $install_path $target_path
sudo cp -r $service_path $target_path

# 重启系统服务
sudo systemctl daemon-reload

sudo systemctl start robocup_game_controller.service
sudo systemctl enable robocup_game_controller.service

sudo systemctl start robocup_joystick.service
sudo systemctl enable robocup_joystick.service

