#!/bin/bash

# 获取脚本所在目录的父目录（robocup_demo）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$PROJECT_DIR/log"

# 创建日志目录（如果不存在）
mkdir -p "$LOG_DIR"

# 日志文件路径
LOG_FILE="$LOG_DIR/realsense_log"

# RealSense ROS 路径
REALSENSE_PATH="$HOME/ThirdParty/realsense-ros"

# 检查 RealSense ROS 路径是否存在
if [ ! -d "$REALSENSE_PATH" ]; then
    echo "错误: RealSense ROS 路径不存在: $REALSENSE_PATH"
    exit 1
fi

# 进入 RealSense ROS 目录并 source setup.bash
cd "$REALSENSE_PATH"

# Source ROS2 环境
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
else
    echo "错误: 找不到 ROS2 环境"
    exit 1
fi

# 启动 RealSense 相机，输出重定向到日志文件，后台运行
echo "正在启动 RealSense 相机..."
echo "日志文件: $LOG_FILE"
echo "========================================" > "$LOG_FILE"
echo "RealSense 相机启动时间: $(date)" >> "$LOG_FILE"
echo "========================================" >> "$LOG_FILE"
nohup ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true >> "$LOG_FILE" 2>&1 &

# 获取进程 PID
REALSENSE_PID=$!

# 保存 PID 到文件，方便后续停止
echo $REALSENSE_PID > "$LOG_DIR/realsense.pid"

echo "RealSense 相机已在后台启动"
echo "进程 PID: $REALSENSE_PID"
echo "查看日志: tail -f $LOG_FILE"
echo "停止相机: kill $REALSENSE_PID 或使用 stop_realsense.sh"
