#!/bin/bash
source /opt/ros/kilted/setup.bash
colcon build --packages-select interfaces
colcon build --packages-select self_driving_pkg --symlink-install
source ./install/setup.bash
sudo chmod 666 /dev/i2c-1
rm -r ./launch/datasets/*
echo "Setup complete."
