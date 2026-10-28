#!/bin/bash
script_path=$(readlink -f "$0")

echo "Script path: $script_path"
# 提取父目录路径作为根路径
root_path=$(dirname $script_path)
echo "Proj root path: $root_path"

rm -rf /home/booster/Workspace/robocup
mkdir -p /home/booster/Workspace/robocup

cp -r $root_path/* /home/booster/Workspace/robocup
bash /home/booster/Workspace/robocup/utils/install_auto_start_assist.sh #自启动
echo "Install success"
