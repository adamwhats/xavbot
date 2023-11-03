FROM dustynv/ros:foxy-ros-base-l4t-r35.4.1

# # Install librealsense
# RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
#     apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
#     && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u \
#     && apt-get install -y librealsense2-utils librealsense2-dev

# # Remove pre-existing opencv installation so future installation of ros-foxy-cv-bridge works
# RUN sudo apt-get purge -y '*opencv*'

# Install xavbot dependencies
RUN apt-get update && apt-get install -y \
    libserial-dev \
    ros-foxy-ament-lint-auto \
    ros-foxy-hardware-interface \
    ros-foxy-joint-state-broadcaster \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-nav2-bringup \
    # ros-foxy-navigation2 \
    # ros-foxy-robot-localization \
    ros-foxy-robot-state-publisher \
    ros-foxy-ros2-control \
    ros-foxy-spatio-temporal-voxel-layer \
    ros-foxy-xacro

RUN pip install \
    pyserial \
    setuptools==58.2.0

# Install testing dependenciesros2
RUN apt-get update && apt-get install -y \
    x11-apps

# Setup env
RUN echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc
RUN echo ". /ros_entrypoint.sh" >> /root/.bashrc
# RUN echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /root/.bashrc
RUN echo "alias cbs='colcon build --symlink-install'" >> /root/.bash_aliases

