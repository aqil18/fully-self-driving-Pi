#!/bin/bash
echo -e "\nSetting up ROS2"
source /opt/ros/kilted/setup.bash

ros2

echo -e "\nBuilding packages"
colcon build --packages-select interfaces
colcon build --packages-select self_driving_pkg --symlink-install
colcon build --packages-select web_video_server
source ./install/setup.bash

echo -e "\nAdding permissions for i2C gateway"
sudo chmod 666 /dev/i2c-1

echo -e "\nRemoving previously recorded datasets"
rm -r ./launch/datasets/*

echo -e "\nSetting up Python venv"
export PYTHONPATH="/home/aqil/fully-self-driving-Pi/envx/lib/python3.12/site-packages"
source ../envx/bin/activate
echo "\nSetup complete."
