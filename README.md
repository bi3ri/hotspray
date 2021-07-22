# Scan and Spray application
This project if forked from <a href="https://github.com/rosin-project/automatica18_scan_and_plan_demo">automatica18_scan_and_plan_demo</a>

## Application implements
- `godel` (Scan and Plan): https://github.com/ros-industrial-consortium/godel


## Installation

```shell

# Create a new ROS workspace
mkdir -p ~/hotspray_ws/src && cd ~/hotspray_ws/src

# Download demo repository
git clone https://github.com/bierm4nn/hotspray_demo.git

# Download dependencies
wstool init .
wstool merge ~/hotspray_ws/hotspray_demo/hotspray_demo.rosinstall
wstool up

# Reset ROS_PACKAGE_PATH
source /opt/ros/kinetic/setup.bash

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

## Qt Glyph Loading Segfault (Kinetic)

Rviz on Kinetic is prone to a segmentation fault caused by internal functions in the Qt library. Our current work-around is to set the following environment variable:

export QT_NO_FT_CACHE=1




## Run application

```shell
# Simulation:
roslaunch hotspray_bringup application_bringup.launch
# Info: Make sure to press play in Gazebo

# Real robot:
roslaunch hotspray_bringup application_bringup.launch sim_robot:=false

```

## Scan and Plan: Remove local scan_parameter cache
Make sure to clear possible local `godel_robot_scan_parameters` since they would overwrite the ones of this repo (`hotspray_bringup/config/robot_scan.yaml`).

```shell
rm -f ~/.ros/godel_robot_scan_parameters.msg
```

## Systems settings for real devices




# hotspray
