# Booster RoboCup Demo

## NOTICE
This branch supports T1 and K1. The branch supporting T2 is [support_T2](https://github.com/BoosterRobotics/robocup_demo/tree/sandbox/support_T2).

Before each match, use the remote controller to press `LT + A` to switch the robot to `kSoccer` mode and complete pre-match localization. After localization succeeds, press `LT + B` to hand control over to the GameController.

## introduction
The Booster RoboCup demo allows the robot to make autonomous decisions to kick the ball and complete the full RoboCup match. It includes three programs: vision, brain, and game_controller.

-   vision
    -   The vision recognition program, based on Yolo-v8, detects objects such as robots, soccer balls, and the field, and calculates their positions in the robot's coordinate system using geometric relationships.
-   brain
    -   The decision-making program reads visual data and GameController game control data, integrates all available information, makes judgments, and controls the robot to perform corresponding actions, completing the match process.
-   game_controller
    -   Reads the game control data packets broadcast by the referee machine on the local area network, converts them into ROS2 topic messages, and makes them available for the brain to use.

##  Install extra dependency
```bash
# Install ros-humble-backward-ros
sudo apt-get install ros-humble-backward-ros
```

```bash
# Install whistle_detection build dependencies (ALSA capture + FFTW float)
sudo apt-get update
sudo apt-get install libasound2-dev libfftw3-dev
```

## RUN ON Booster K1 or T1
This demo is designed for running on Booster K1, if you'd like to run this demo on Booster T1, few configs need to be updated.
### src/brain/config/config.yaml or src/brain/config/config_local.yaml
1. set robot.robot_height to 1.12
2. set robot.odom_factor to 1.2
3. set RLVisionKick.enableAutoVisualKick to false
4. set vision.image_camera_info_topic, vision.depth_image_topic, vision.depth_camera_info_topic to corresponding image topic
### src/vision/config/vision.yaml or src/vision/config/vision_local.yaml
1. set camera.color_topic, camera.depth_topic, camera.intrin_topic to corresponding ones
2. set detection_model.model_path to ./src/vision/model/best_1223_10.3.engine
3. set segmentation_model.model_path to ./src/vision/model/best_seg_orin_10.3.engine

### About config: RLVisionKick.enableAutoVisualKick
```yaml
RLVisionKick
    enableAutoVisualKick: true // This feature is only supported on the Booster K1, requiring firmware version 1.6.2 or higher.
```
VisionKick is only support on K1 and requiring firmware version 1.6.1 or higher. See [Firmware 1.6.2 Install Reference](https://booster.feishu.cn/wiki/E3q5wF5SnitXZgkY18Uc8odBnXb#share-BYaDdL4nAoPfdcx2y3BcwORyndf) for install instruction.

Please make sure you have installed latest Booster Robotics SDK.

## Note
This repo support jetpack 6.2. Adapted to the default TRT model in src/vision/config/vision.yaml.

vision.yaml for jetpack 6.2 machine

    detection_model:
	    model_path: ./src/vision/model/best_digua_second_10.3.engine
	    confidence_threshold: 0.2

## Build


### Build the programs with cuda (real robot)
```bash
./scripts/build.sh
```

## Run

### Run on real robot
```bash
./scripts/start.sh
```
