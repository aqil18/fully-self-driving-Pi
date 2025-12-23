# fully-self-driving-Pi
Fully (but not so fully) self driving Raspberry Pi car.

<img src="demos/IMG_4602.jpg" width="50%">



# Problem 1 - Lane Tracking
The goal is to effectively follow a line by supplying angle commands to a motor based on the results of a deep learning model for lane detection.

# First time dev setup?
## Cloning cv_bridge
cd src
git clone https://github.com/ros-perception/vision_opencv.git -b ros2
cd ..
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


export PYTHONPATH="/usr/bin/python3"
which python3

# Adding new nodes?
Update setup.py.

# Testing Update 21/06
- The image_test_node can be used to upload select images from example_images
- The inference_node can be used to complete 'FAKE' inference on the images and gives angle and speed
- The display_node can be used to see the output images in output_images
- The motor_node can receive callbacks but has not been teested to follow angles and speed

# To test camera
1. Open car utils
2. Run camera.py

# To test motor
1. Run Motor.py
python3 -m self_driving_pkg.car_utils.Motor

# To launch
cd src
cd launch
ros2 launch fsd_launch.py

# ROS2 Camera
vcgencmd get_camera
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640, 480]"

# Image server
colcon build --packages-select web_video_server
source install/setup.bash
ros2 run web_video_server web_video_server
https://github.com/RobotWebTools/web_video_server