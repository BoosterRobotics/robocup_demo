#!/bin/bash
echo ["STOP VISION"]
sudo killall -9 vision_node
echo ["STOP BRAIN"]
sudo killall -9 brain_node
echo ["STOP SOUND"]
sudo killall -9 sound_play_node
echo ["STOP GAMECONTROLLER"]
sudo killall -9 game_controller
