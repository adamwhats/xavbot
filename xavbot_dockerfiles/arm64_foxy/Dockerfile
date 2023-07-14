FROM dustynv/ros:foxy-ros-base-l4t-r35.2.1 AS base

# Install librealsense
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u \
    && apt-get install -y librealsense2-utils librealsense2-dev

# Remove pre-existing opencv installation so future installation of ros-foxy-cv-bridge works
RUN sudo apt-get purge -y '*opencv*'

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
    ros-foxy-spatio-temporal-voxel-layer \
    ros-foxy-rviz2 \
    ros-foxy-xacro

RUN pip install \
    pyserial

# Install testing dependencies
RUN sudo apt-get update && sudo apt-get install -y \
    x11-apps

# Setup env
RUN echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc
