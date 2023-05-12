# XavBot

My journey into building a robot from scratch, with the initial aim of exploring the Nav2 framework. Future goals are experimenting with building my own SLAM and perception algorithms as well as Gazebo/ Isaac Sim simulation.

XavBot is a small holonomic robot based around the NVIDIA Jetson Xavier NX with a Pimoroni Motor2040 motor control board and Intel RealSense D415 RGB-D camera.

# Packages

## xavbot_bringup

Contains the launch files and configs.

## xavbot_controller

The controller for the mecanum wheel kinematics.

## xavbot_description

Contains the URDF xacros. 

## xavbot_hardware

Hardware interface and holonomic motion control. Written with lots of guidance from [Articulated Robotics'](https://www.youtube.com/c/ArticulatedRobotics) excellent series.

