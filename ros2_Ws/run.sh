#!/bin/bash
echo "Launching robot launcher.."
source ./install/setup.bash
ros2 launch my_robot_launcher robot.launch.py &

wait
echo "robot launcher has terminated succesfully"