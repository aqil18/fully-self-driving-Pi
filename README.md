# fully-self-driving-Pi
Fully (but not so fully) self driving Raspberry Pi car.

## Dev Setup
1. Remote connections
2. Reopen in container
3. Run source /opt/ros/$ROS_DISTRO/setup.bash or . install/setup.bash
4. Run ros2

## Reopen in RPI
1. VSCODE ssh into rpi
2. VSCODE reopen in container
3. Run source /opt/ros/$ROS_DISTRO/setup.bash
4. Run ros2

## Adding new nodes?
Update setup.py.

## Updating nodes?
### At ws root -> If you have multiple packages
#### Update dependencies
rosdep install -i --from-path src --rosdistro jazzy -y
#### Build package
colcon build --packages-select self_driving_pkg
#### Source project specific setup
source ./install/setup.bash 

# How to run the nodes?
## At ws root as this is where you built the packages
ros2 run self_driving_pkg motor_node
ros2 topic pub /motor/cmd std_msgs/msg/String '{"data": "{\"action\": \"move\", \"angle\": 0, \"speed\": 70}"}

## Problem 1 - Lane Tracking
The goal is to effectively follow a line by supplying angle commands to a motor based on the results of a deep learning model for lane detection.