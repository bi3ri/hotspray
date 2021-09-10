FROM osrf/ros:kinetic-desktop-full-xenial

RUN apt update && \
    apt upgrade -y && \
    apt install -y ros-kinetic-industrial-core && \
    apt-get install -y ros-kinetic-ros-control ros-kinetic-ros-controllers && \
    apt-get install -y ros-kinetic-universal-robots && \
    mkdir -p /root/ur_workspace/src && \
    cd /root/ur_workspace/src && \
    git clone https://github.com/ros-industrial/ur_modern_driver.git -b kinetic-devel && \
    cd /root/ur_workspace && \
    /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /root/ur_workspace; catkin_make'

ENV ROBOT_IP_ADDRESS=192.168.0.1

CMD /bin/bash -c '. /root/ur_workspace/devel/setup.bash; roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=$ROBOT_IP_ADDRESS'
