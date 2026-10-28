#!/bin/bash

# Define source and target directories
SYSTEMD_SOURCE_DIR="systemd"
SYSTEMD_TARGET_DIR="/etc/systemd/system"
LOGGER_SERVICE_SOURCE_DIR="scripts/logger_service"
LOGGER_SERVICE_TARGET_DIR="/opt/booster/vision"

# Copy systemd files
echo "Copying systemd files to ${SYSTEMD_TARGET_DIR}..."
sudo cp -r ${SYSTEMD_SOURCE_DIR}/* ${SYSTEMD_TARGET_DIR}

# Copy logger service scripts
echo "Copying logger service scripts to ${LOGGER_SERVICE_TARGET_DIR}..."
sudo mkdir -p ${LOGGER_SERVICE_TARGET_DIR}
sudo cp -r ${LOGGER_SERVICE_SOURCE_DIR}/* ${LOGGER_SERVICE_TARGET_DIR}

echo "Files copied successfully."

sudo apt-get install -y libnfsidmap1=1:2.6.1-1ubuntu1
sudo apt-get install -y nfs-common
sudo systemctl daemon-reload
sudo systemctl enable booster-vision-data.service
sudo systemctl start booster-vision-data.service
sudo systemctl status booster-vision-data.service