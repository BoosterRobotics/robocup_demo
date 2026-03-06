# Booster RoboCup Demo

## NOTICE
The latest RoboCup rules do not allow unicast communication between robots and impose limits on the data size of communication packets during matches. This part of the rules has not yet been addressed in the current open‑source code. If robot communication is required in the competition, the implementation in brain_communication.cpp needs to be modified to comply with the rules.

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
# Install onnxruntime (If you want to run without cuda)

./third_party_aarch64/install_onnxruntime.sh # if aarch64

./third_party/install_onnxruntime.sh # if x64
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
    enableAutoVisualKick: true // This feature is only supported on the Booster K1, requiring firmware version 1.5.2 or higher.
```

## Note
This repo support jetpack 6.2. Adapted to the default TRT model in src/vision/config/vision.yaml.

vision.yaml for jetpack 6.2 machine

    detection_model:
	    model_path: ./src/vision/model/best_digua_second_10.3.engine
	    confidence_threshold: 0.2

## Visualization
booster_soccer.layout can be imported into Booster Studio for inspect logs

## Build

### Build the programs without cuda (inference with onnx, so you need to run install onnx in advance)
```bash
./scripts/build_no_cuda.sh
```
#### build error
if you clone this repo in windows and run in Booster Studio Simulator, you may encounter problems blow
```bash
./build_no_cuda.sh 
-bash: ./build_no_cuda.sh: /bin/bash^M: bad interpreter: No such file or directory
```
This is caused by difference of Windows and Linux/Unix based system. 

Run cmd below
```bash
cd booster_soccer/scripts
find . -type f -print0 | xargs -0 sed -i 's/\r$//'
```
Then run build_no_cuda.sh. 

### Build the programs with cuda (real robot)
```bash
./scripts/build.sh
```

## Run
### Run on the virtual robot
#### src/brain/config/config.yaml or src/brain/config/config_local.yaml
1. set vision.image_camera_info_topic, vision.depth_image_topic, vision.depth_camera_info_topic to corresponding image topic
#### update src/vision/config/vision.yaml or src/vision/config/vision_local.yaml
1. set camera.color_topic, camera.depth_topic, camera.intrin_topic to corresponding ones
2. set detection_model.model_path to ./src/vision/model/sim_data_det_0126.onnx
3. set segmentation_model.model_path to ./src/vision/model/sim_data_seg_0126.onnx
```bash
./scripts/sim_start.sh
```

### Run on real robot
```bash
./scripts/start.sh
```

