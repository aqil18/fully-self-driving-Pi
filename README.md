# fully-self-driving-Pi
Fully (but not so fully) self driving Raspberry Pi car.

# Problem 1 - Lane Tracking
The goal is to effectively follow a line by supplying angle commands to a motor based on the results of a deep learning model for lane detection.

# First time dev setup?
## Connecting to the dev container
1. Remote connections
2. Reopen in container
3. Run source /opt/ros/$ROS_DISTRO/setup.bash or . install/setup.bash
4. Run ros2
## Creating a venv
sudo apt update
sudo apt install python3-virtualenv
sudo chmod 666 /dev/i2c-1
virtualenv --version
virtualenv -p python3 ./envx
source ./envx/bin/activate
touch ./envx/COLCON_IGNORE
python3 -m pip install smbus2
## ROS setup
### Update dependencies
rosdep install -i --from-path src --rosdistro jazzy -y

# How to run the nodes?
### 1. At 'ws' root as this is where you built the packages
export PYTHONPATH="/home/aqil/envx/lib/python3.12/site-packages" # NOTE ensure this is right for your system
### 2. Setup ROS distro specific setup
source /opt/ros/$ROS_DISTRO/setup.bash
### 3. Build ROS package 
colcon build --packages-select self_driving_pkg --symlink-install
### 4. Setup project specific setup
source ./install/setup.bash 
### 5. Run the node
ros2 run self_driving_pkg motor_node
### 6. Publishing to the nodes (In another terminal)
#### Ensure power to motor is supplied
ros2 topic pub /motor/cmd std_msgs/msg/String '{"data": "{\"action\": \"move\", \"angle\": 0, \"speed\": 70}"}

# Adding new nodes?
Update setup.py.

# To test camera
1. Open car utils
2. Run camera.py