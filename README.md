# fully-self-driving-Pi
Fully (but not so fully) self driving Raspberry Pi car.

## Dev Setup
1. Remote connections
2. Reopen in container
3. Run source /opt/ros/$ROS_DISTRO/setup.bash or . install/setup.bash
4. Run ros2

## Adding new nodes?
Update setup.py.

## Updating nodes?
rosdep install -i --from-path src --rosdistro jazzy -y (Update dependencies)
colcon build --packages-select self_driving_pkg
. install/setup.bash
ros2 run self_driving_pkg motor_node

## Problem 1 - Lane Tracking
The goal is to effectively follow a line by supplying angle commands to a motor based on the results of a deep learning model for lane detection.