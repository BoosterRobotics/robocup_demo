#!/bin/bash
echo ["STOP VISION"]
pkill -9 vision_node
echo ["stop detection_converter"]
pkill -9 -f detection_converter_node.py
echo ["STOP BRAIN"]
pkill -9 brain_node
echo ["STOP GAMECONTROLLER"]
ps aux | grep "game_controller" | grep -v "game_controller_app" | grep -v "grep" | awk '{print $2}' | xargs -r kill -9