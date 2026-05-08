# fully-self-driving-Pi
Click on the youtube video below for a demo.


[![Watch the video](/demos/pi%20thumbnail.jpg)](https://youtu.be/4L_sISlG2ks)


# Problem 1 - Lane Tracking
The goal is to effectively follow a line by supplying angle commands to a motor based on the results of a deep learning model for lane detection.

# Architecture
The system is a ROS 2 pipeline. Each node runs as a separate process and communicates via topics.

<img src="demos/pipeline 2.png" width="100%">


| Node | What it does |
|---|---|
| `v4l2_camera` | Publishes raw camera frames to `/fsd/image_raw` |
| `detection_node` | Classical CV - thresholds image, finds lane contour, publishes pixel offset to `/detection/offset` |
| `inference_node` | Runs PiPilotNet CNN on image + offset, publishes steering/throttle to `/motor/cmd` |
| `pid_node` | PID controller on offset. Falls back to CNN if line is lost (watchdog) |
| `motor_node` | Subscribes to `/motor/cmd`, drives motors and servo |
| `teleop_node` | WASD keyboard control. Publishes to `/motor/cmd` and `/teleop/shutdown` |
| `dataset_recorder` | Records images + motor commands + offset to CSV for training |
| `web_video_server` | Streams any ROS image topic over HTTP — view camera feed in a browser without being on the Pi |

# Launch files
All launch files are in `rpi/launch/`

| File | What it runs |
|---|---|
| `fsd_launch_v3.py` | **Current approach** - camera + detection + CNN inference + motor |
| `fsd_launch_v2.py` | camera + detection + CNN inference + PID fallback + motor + web video server |
| `fsd_launch_v1.py` | camera + CNN inference only + motor (no perception node) |
| `data_collect_launch.py` | camera + detection + motor + dataset recorder |

# How to obtain the dataset 
Record the dataset from the pi
```
ros2 launch data_collect_launch.py (from launch folder)
ros2 run self_driving_pkg teleop_node
```
Teleop controls: W/S speed up/down, A/D steer left/right, SPACE stop, CTRL+C stop and signal recorder to shut down

To view camera output go to
http://IP:8080/stream_viewer?topic=/fsd/image_raw

From a terminal thats not on the pi run 
```
rsync -av --progress
```

Dataset lands in `rpi/launch/datasets/<timestamp>/` with structure:
```
datasets/
  2024-01-01_12-00-00/
    images/        ← 160x120 JPGs
    labels/
      labels.csv   ← filename, steering, throttle, offset
```

# Training the model

## 1. Merge datasets
Edit `ml/merge_datasets.py` to set the range of dataset folders to merge, then run:
```
cd ml
python merge_datasets.py
```
Merged CSV lands at `ml/datasets/merged/labels/labels.csv`

## 2. Configure training
Edit `ml/config.py` to adjust hyperparameters:
```python
batch_size    = 64
learning_rate = 1e-3
epochs        = 50
val_frac      = 0.15   # 15% held out for validation
max_throttle  = 40     # motor units
max_angle     = 90     # degrees
```

## 3. Train
```
cd ml
python train.py
```
Best model (lowest val MSE) saved to `rpi/src/self_driving_pkg/self_driving_pkg/models/model.pt`

# Setup on Raspberry Pi

## First time setup
```
cd rpi
source setup.sh
```

## Every session
```
# Source ROS and project setup
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
```

## Build the package
```
colcon build --packages-select self_driving_pkg --symlink-install
source install/setup.bash
```

## Run a launch file (from rpi/ root)
```
ros2 launch launch/fsd_launch_v3.py
```

## Run individual nodes
```
ros2 run self_driving_pkg motor_node
ros2 run self_driving_pkg detection_node
ros2 run self_driving_pkg inference_node
ros2 run self_driving_pkg pid_node
ros2 run self_driving_pkg teleop_node
ros2 run self_driving_pkg dataset_recorder
```

## Manually publish a motor command (for testing)
```
ros2 topic pub /motor/cmd interfaces/msg/Motor '{"angle": 0, "speed": 15}'
```

## View live topics
```
ros2 topic list
ros2 topic echo /detection/offset
ros2 topic echo /motor/cmd
```

# Setup on Dev Container

## Cloning cv_bridge
```
cd src
git clone https://github.com/ros-perception/vision_opencv.git -b ros2
cd ..
```

## Connecting to the dev container
1. Remote connections
2. Reopen in container
3. Run `source /opt/ros/$ROS_DISTRO/setup.bash` or `. install/setup.bash`
4. Run ros2

## Creating a venv
```
sudo apt update
sudo apt install python3-virtualenv
sudo chmod 666 /dev/i2c-1
virtualenv -p python3 ./envx
source ./envx/bin/activate
touch ./envx/COLCON_IGNORE
python3 -m pip install smbus2
```

## ROS setup
```
export PYTHONPATH="/home/..."  # adjust for your system
rosdep install -i --from-path src --rosdistro jazzy -y
```

# Adding new nodes?
Update `setup.py` with the new entry point.

# ROS2 Camera
```
vcgencmd get_camera
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640, 480]"
```

# Image server (live stream over HTTP)
```
colcon build --packages-select web_video_server
source install/setup.bash
ros2 run web_video_server web_video_server
```
Then open http://IP:8080/stream_viewer?topic=/fsd/image_raw
