## HOTSPRAY




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

# Install realsense driver
Add the server to the list of repositories:
- Ubuntu 16 LTS:
    sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
- Ubuntu 18 LTS:
    sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u

sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

```





## Run application

```shell
# Simulation:
roslaunch hotspray_bringup application_bringup.launch
# Info: Make sure to press play in Gazebo

# Real robot:
roslaunch hotspray_bringup application_bringup.launch sim_robot:=false

```


# Troubleshooting

# Qt Glyph Loading Segfault (Kinetic)

Rviz on Kinetic is prone to a segmentation fault caused by internal functions in the Qt library. Our current work-around is to set the following environment variable:

export QT_NO_FT_CACHE=1




# Dependencies for packages

# YAK
# CUDA
sudo apt update
sudo apt install -y nvidia-cuda-toolkit 



# Tesseract
sudo add-apt-repository ppa:ros-industrial/ppa
sudo apt-get update

sudo apt install -y 
libbullet-dev libbullet-extras-dev ros-noetic-fcl ros-noetic-taskflow 

sudo apt install ros-noetic-opw-kinematics 

sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev


# Tesseract_ros
sudo apt-get install ros-noetic-octomap-ros






stuff used:
nohlman json