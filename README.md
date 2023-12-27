## INSERR_alcino

## Installing up ROS noetic:
https://wiki.ros.org/noetic/Installation/Ubuntu

## Setup Environment
1) gedit ~/.bashrc
2) source /opt/ros/noetic/setup.bash

## Setting up ROS workspace

# Make ROS Work space
1) mkdir catkin_ws

# Make ROS src folder
2) ~/catkin_ws$ mkdir src

# Compile Workspace (Be sure not in src folder)
3) ~/catkin_ws$ catkin_make

# Source setup.bash
4) gedit ~/.bashrc
5) source ~/catkin_ws/devel/setup.bash

## Create a ROS package
1) ~/catkin_ws/src$ catkin_create_pkg YOUR_PACKAGE_NAME rospy

## Build the package
1) ~/catkin_ws$ catkin_make
