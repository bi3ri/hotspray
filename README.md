## HOTSPRAY

#TODO GIF


## Installation

```shell

# Create a new ROS workspace
mkdir -p ~/hotspray_ws/src && cd ~/hotspray_ws/src

# Download demo repository
git clone https://github.com/bi3ri/hotspray.git

# Download dependencies
wstool init .
wstool merge ~/hotspray_ws/src/hotspray/hotspray.rosinstall
wstool up

wstool merge --merge-keep ~/hotspray_ws/src/tesseract_ros/dependencies.rosinstall
wstool up

# Reset ROS_PACKAGE_PATH
source /opt/ros/noetic/setup.bash

# Install dependencies 
rosdep update && rosdep install -y --from-paths ~/hotspray_ws/src --ignore-src 

# Build workspace
cd ~/hotspray_ws && catkin build 

# Source workspace
source ~/hotspray_ws/devel/setup.bash

```

## Dependencies for packages
```shell
# CUDA
sudo apt install -y nvidia-cuda-toolkit 

# Tesseract
sudo add-apt-repository ppa:ros-industrial/ppa
sudo apt-get update

sudo apt install -y libbullet-dev libbullet-extras-dev ros-noetic-fcl ros-noetic-taskflow 
sudo apt install ros-noetic-opw-kinematics 
sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev

# Tesseract_ros
sudo apt-get install ros-noetic-octomap-ros

```

## Run application

```shell
# Simulation and application helper packages
roslaunch hotspray_bringup application_bringup.launch
# Info: Make sure to press play in Gazebo

# Start application

# 1 Create scan poses:
roslaunch hotspray_application create_scan_poses.launch <scan_poses_file_name> 
# Info: if scan_poses_file_name is left empty a default name is used

# 2 Load scan poses:
roslaunch hotspray_application load_scan_poses.launch <scan_poses_file_name> 
# Info: if scan_poses_file_name is left empty a default name is used

```
