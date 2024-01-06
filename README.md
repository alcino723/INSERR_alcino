# INSERR_alcino Documentation

# Installing up ROS noetic:
https://wiki.ros.org/noetic/Installation/Ubuntu

# Setup Environment
```bash
gedit ~/.bashrc
```
Add
```bash
source /opt/ros/noetic/setup.bash
```
you can close all the current terminals and open new ones<br>
or source ```bashrc``` in all your current terminal
```bash
source ~/.bashrc
```

# Setting up ROS workspace

## 1. Create ROS Work space
```bash
mkdir catkin_ws
```

## 2. Make ROS src folder
Navigate to ```~/catkin_ws```
```bash
cd ~/catkin_ws
```
Create ```/src``` Folder
```bash
mkdir src
```

## 3. Compile Workspace
Navigate to ```~/catkin_ws```
```bash
cd ~/catkin_ws
```
Complile workspace
```bash
catkin_make
```

## 4. Source setup.bash
```bash
gedit ~/.bashrc
```
Add
```bash
source ~/catkin_ws/devel/setup.bash
```

# Create a ROS package
Navigate to ```~/catkin_ws/src```
```bash
cd ~/catkin_ws/src
```
Create a ROS package (replace ```YOUR_PACKAGE_NAME``` with your own package name)
```bash
catkin_create_pkg YOUR_PACKAGE_NAME rospy
```

# Build the package
Navigate to ```~/catkin_ws```
```bash
cd ~/catkin_ws
```
Build the package
```bash
catkin_make
```

# Create a folder to host ROS node files
Navigate to ```~/catkin_ws/src/YOUR_PACKAGE_NAME```
```bash
cd ~/catkin_ws/src/YOUR_PACKAGE_NAME
```
Create a folder to write your nodes in
```bash
mkdir scripts
```
# Create a ROS node file
Navigate to ```~/catkin_ws/src/YOUR_PACKAGE_NAME/scripts```
```bash
cd ~/catkin_ws/src/YOUR_PACKAGE_NAME/scripts
```
Create Node file
```bash
touch YOUR_NODE_FILE.py
```
Make the node executable
```bash
chmod +x YOUR_NODE_FILE.py
```
# Setting up ROS communication between machines
1) Pick 1 machine to be ROS Master
2) On Master Machine, Edit ```~/.bashrc``` file
```bash
sudo nano ~/.bashrc
```
3) On Master machine ```~/.bashrc``` file, Add
```yaml
export ROS_MASTER_URI=http://MASTER_URI:11311
export ROS_IP=MASTER_IP
```
4) On Slave Machine, Edit ```~/.bashrc``` file
```bash
sudo nano ~/.bashrc
```
5) On Slave Machine ```~/.bashrc``` file, Add
```yaml
export ROS_MASTER_URI=http://MASTER_IP:11311
export ROS_IP=SLAVE_IP
```

# Trouble Shoot
## Unbuntu DHCP release and renew 
Release All DHCP
```bash
sudo dhclient -r
```
Renew All DHCP
```bash
sudo dhclient 
```

## Ubuntu DHCP not getting IP 
Navigate to ```/etc/netplan```
```bash
cd /etc/netplan
```
Create network manager file 
```bash
sudo touch 01-network-manager-all.yaml
```
Edit ```01-network-manager-all.yaml```
```bash
sudo nano /etc/netplan/01-network-manager-all.yaml
```
Add
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: true
```
Apply new setting
```bash
sudo netplan apply
```
Check IP
```bash
hostname -i
```

Reboot (If needed) 
```bash
sudo reboot
```
   
## /usr/bin/env: ‘python3\r’: No such file or directory
Solution : https://askubuntu.com/questions/896860/usr-bin-env-python3-r-no-such-file-or-directory <br>

Install ```dos2unix```
```bash
sudo apt install dos2unix
```
Convert files
```bash
dos2unix /PATH/YOUR_FILE
```

