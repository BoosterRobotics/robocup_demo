#!/bin/bash

/home/booster/Workspace/robocup/scripts/stop.sh
rm -rf /home/booster/Workspace/robocup
systemctl --user stop robocup_game_assist.service
systemctl --user disable robocup_game_assist.service
systemctl --user daemon-reload
rm /home/booster/.config/systemd/user/robocup_game_assist.service
