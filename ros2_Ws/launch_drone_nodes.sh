#!/usr/bin/env bash
#
# launch_nodes_gui.sh - simplified to use a single ROS 2 launch file
#
# When run over SSH, this opens one terminal window on the Jetson Nano’s display (DISPLAY=:0), launching your full stack.

### 0) Setup DISPLAY and XAUTHORITY
export DISPLAY=:0
export XAUTHORITY=/home/nits/.Xauthority

### 1) Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source /home/nits/master_controller/nits-yantrarnav/ros2_Ws/install/setup.bash

### 2) Launch all in one gnome-terminal window
gnome-terminal -- bash -c "ros2 launch my_drone_launcher drone.launch.py; exec bash"
