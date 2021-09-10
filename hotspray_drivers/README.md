
docker pull osrf/ros:kinetic-desktop-xenial

docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    osrf/ros:indigo-desktop-full \
    rqt

    http://wiki.ros.org/docker/Tutorials/GUI


## Install Docker

...


## Build Image

'docker build --tag ur_modern_driver .'

## Run Image

..

https://answers.ros.org/question/228292/exposing-ros-containers-to-host-machine/