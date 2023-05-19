FROM dustynv/ros:foxy-ros-base-l4t-r35.2.1

# Build librealsense
RUN sudo apt-get update && sudo apt-get -y upgrade
RUN sudo apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
RUN sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
RUN cd ~/ \
    && git clone https://github.com/IntelRealSense/librealsense.git \
    && cd librealsense/ && mkdir build && cd build \
    && cmake ../ -DFORCE_RSUSB_BACKEND=true -DBUILD_PYTHON_BINDINGS=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DBUILD_WITH_CUDA=true\
    && sudo make uninstall && make clean && make && sudo make install

# Remove pre-existing opencv installation so future installation of ros-foxy-cv-bridge works
RUN sudo apt-get purge -y '*opencv*'

# Build RealSense ROS wrapper
RUN sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
WORKDIR /realsense_ros_clone_dir
RUN mkdir ./src && cd src \
    && git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development \
    && cd .. \
    && rosdep init && rosdep update && rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
COPY . .
# RUN curl https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules >> /etc/udev/rules.d/99-realsense-libusb.rules

# X11 apps for testing
RUN sudo apt-get install -y x11-apps