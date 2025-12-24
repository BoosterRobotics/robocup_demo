# Robocup Demo
## Introduction
The Booster T1 Robocup official demo allows the robot to make autonomous decisions to kick the ball and complete the full Robocup match. It includes three programs: vision, brain, and game_controller.

- vision
    - The Robocup vision recognition program, based on Yolo-v8, detects objects such as robots, soccer balls, and the field, and calculates their positions in the robot's coordinate system using geometric relationships.
- brain
    - The Robocup decision-making program reads visual data and GameController game control data, integrates all available information, makes judgments, and controls the robot to perform corresponding actions, completing the match process.
- game_controller
    - Reads the game control data packets broadcast by the referee machine on the local area network, converts them into ROS2 topic messages, and makes them available for the brain to use.

### K1 3v3 Demo
We now provide K1 3v3 demo in branch [sandbox/feat/k1_3v3_demo](https://github.com/BoosterRobotics/robocup_demo/tree/sandbox/feat/k1_3v3_demo).

## Install extra dependency
sudo apt-get install ros-humble-backward-ros

## Build and Run

### Note    
This repo support both jetpack 6.0 and 6.2. If the repo is deployed on jetpack 6.2 machine, please modify src/vision/config/vision.yaml to selelct correct trt model

vision.yaml for jetpack 6.0 machine
``` yaml
detection_model:
  model_path: "./src/vision/model/best_orin.engine"
  confidence_threshold: 0.2
```
vision.yaml for jetpack 6.2 machine
```yaml
detection_model:
  model_path: "./src/vision/model/best_orin_10.3.engine"
  confidence_threshold: 0.2
```

To decide jetpack version, please execute `dpkg -l | grep jetpack` on host.
```
# Build the programs
./scripts/build.sh

# Run in the simulation environment
./scripts/sim_start.sh

# Run on the actual robot
./scripts/start.sh
```

## Documents
[Chinese Version](https://booster.feishu.cn/wiki/P5kJw6nDGib5wskZ3Yfc289lnIg)

[English Version](https://booster.feishu.cn/wiki/XY6Kwrq1bizif4kq7X9c14twnle)
