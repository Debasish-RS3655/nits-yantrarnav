# for building
# colcon build --symlink-install --packages-select my_robot_controller my_robot_launcher

source ./install/setup.bash
ros2 launch my_robot_launcher robot.launch.py
wait
echo