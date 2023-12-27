# INSERR_alcino

# Installing up ROS noetic:
https://wiki.ros.org/noetic/Installation/Ubuntu

# Setup Environment
1) gedit ~/.bashrc
2) source /opt/ros/noetic/setup.bash

# Setting up ROS workspace

## 1. Create ROS Work space
mkdir catkin_ws

## 2. Make ROS src folder
~/catkin_ws$ mkdir src

## 3. Compile Workspace (Be sure not in src folder)
~/catkin_ws$ catkin_make

## 4. Source setup.bash
1) gedit ~/.bashrc
2) source ~/catkin_ws/devel/setup.bash

# Create a ROS package
~/catkin_ws/src$ catkin_create_pkg YOUR_PACKAGE_NAME rospy

# Build the package
~/catkin_ws$ catkin_make

# Create a folder to host ROS node files and write ROS node in that file.
1) ~/catkin_ws/src/YOUR_PACKAGE_NAME$ mkdir scripts
2) ~/catkin_ws/src/YOUR_PACKAGE_NAME/scripts$ touch YOUR_NODE_FILE.py
3) ~/catkin_ws/src/YOUR_PACKAGE_NAME/scripts$ chmod +x YOUR_NODE_FILE.py
