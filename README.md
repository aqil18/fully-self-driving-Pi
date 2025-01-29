# fully-self-driving-Pi
Fully (but not so fully) self driving Raspberry Pi car.

## Dev Setup
1. Remote connections
2. Reopen in container
3. Run source /opt/ros/$ROS_DISTRO/setup.bash
4. Run ros2

## Problem 1 - Lane Tracking
The goal is to effectively follow a line by supplying angle commands to a motor based on the results of a deep learning model for lane detection.