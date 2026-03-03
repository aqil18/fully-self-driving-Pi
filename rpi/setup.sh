#!/bin/bash
BOLDGREEN="\e[1;${GREEN}m"
echo -e "${BOLDGREEN}Setting up ROS2 ${ENDCOLOR}"
source /opt/ros/kilted/setup.bash

echo -e "${BOLDGREEN}Building packages${ENDCOLOR}"
colcon build --packages-select interfaces
colcon build --packages-select self_driving_pkg --symlink-install
colcon build --packages-select web_video_server
source ./install/setup.bash

echo -e "${BOLDGREEN}Adding permissions for i2C gateway${ENDCOLOR}"
sudo chmod 666 /dev/i2c-1

echo -e "${BOLDGREEN}Removing previously recorded datasets${ENDCOLOR}"
rm -r ./launch/datasets/*

echo -e "${BOLDGREEN}Setting up Python venv${ENDCOLOR}"
export PYTHONPATH="/home/aqil/fully-self-driving-Pi/envx/lib/python3.12/site-packages"
source ../envx/bin/activate
echo "Setup complete."
