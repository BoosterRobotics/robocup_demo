#!/bin/bash
echo "[START VISION]"
cd `dirname $0`
cd ..

source ./install/setup.bash
#export FASTRTPS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml

ros2 run vision calibration_node handeye ./src/vision/config/vision.yaml
sudo cp /tmp/vision.yaml /opt/booster/vision.yaml && \
  echo "[OK] Calibration result copied to /opt/booster/vision.yaml" || \
  echo "[WARN] /tmp/vision.yaml not found, system calibration file not updated"