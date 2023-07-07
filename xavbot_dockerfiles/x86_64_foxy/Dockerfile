FROM althack/ros2:foxy-cuda-dev AS base

# Configure group to give access toe /dev/ttyACM0
RUN usermod -aG dialout ros

# Install librealsense
RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u && \
    sudo apt-get update && sudo apt-get install -y librealsense2-dkms librealsense2-utils

# Install xavbot dependencies
RUN sudo apt-get update && sudo apt-get install -y \
    libserial-dev \
    ros-foxy-hardware-interface \
    ros-foxy-joint-state-broadcaster \
    ros-foxy-joint-state-broadcaster \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-nav2-bringup \
    ros-foxy-navigation2 \
    ros-foxy-realsense2-camera \
    ros-foxy-robot-localization \
    ros-foxy-ros2-control \
    ros-foxy-rviz2 \
    ros-foxy-spatio-temporal-voxel-layer \
    ros-foxy-xacro

# Install testing dependencies
RUN sudo apt-get update && sudo apt-get install -y \
    x11-apps && \
pip install \
    pyserial

# Setup env
RUN echo "export ROS_DOMAIN_ID=0" >> /home/ros/.bashrc
