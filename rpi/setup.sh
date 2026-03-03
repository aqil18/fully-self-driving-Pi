#!/bin/bash
source /opt/ros/kilted/setup.bash
colcon build --packages-select interfaces
colcon build --packages-select self_driving_pkg --symlink-install
colcon build --packages-select web_video_server
source ./install/setup.bash
sudo chmod 666 /dev/i2c-1
rm -r ./launch/datasets/*
export PYTHONPATH="/home/aqil/fully-self-driving-Pi/envx/lib/python3.12/site-packages"
source ../envx/bin/activate
echo "Setup complete."
