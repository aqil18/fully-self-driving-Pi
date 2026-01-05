#!/bin/bash
source /opt/ros/kilted/setup.bash
colcon build --packages-select self_driving_pkg --symlink-install
colcon build --packages-select interfaces
source ./install/setup.bash
sudo chmod 666 /dev/i2c-1
echo "Setup complete."