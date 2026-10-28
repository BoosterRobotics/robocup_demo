#!/bin/sh
# 获取当前脚本文件的绝对路径
script_path=$(readlink -f "$0")
echo "Script path: $script_path"

# 提取父目录路径作为根路径
root_path=$(dirname $script_path)
echo "root_path: $root_path"

service_path=$root_path/service

# 先停止系统服务
systemctl --user stop robocup_game_assist.service
mkdir -p ~/.config/systemd/user/
cp $service_path/robocup_game_assist.service ~/.config/systemd/user/

# 重启系统服务
systemctl --user daemon-reload

systemctl --user start robocup_game_assist.service
systemctl --user enable robocup_game_assist.service

