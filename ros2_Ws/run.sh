# #!/bin/bash

# echo "Launching robot launcher.."
# source ./install/setup.bash

# # the camera package
# ros2 launch realsense2_camera rs_launch.py color_fps:=15 depth_fps:=15\ &
#    unite_imu_method:=1 \
#    align_depth:=true \
#    sync:=true \
#    enable_gyro:=true \
#    enable_accel:=true &
# # fix the missing TF transform
# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link &

# # launch RTAB Map with clearing previous session clearing the directory
# rm -f ~/.ros/rtabmap.db && ros2 launch rtabmap_launch rtabmap.launch.py \
#    rgb_topic:=/camera/camera/color/image_raw \
#    depth_topic:=/camera/camera/depth/image_rect_raw \
#    camera_info_topic:=/camera/camera/color/camera_info \
#    approx_sync:=true approx_sync_max_interval:=0.02\
#    rtabmap_args:="--Vis/DepthAsMask false --RGBD/Decimation 2" &

# # launch the master controller
# ros2 launch my_robot_launcher robot.launch.py &
# wait
# echo "robot launcher has terminated succesfully"


# for building
# colcon build --symlink-install --packages-select my_robot_controller my_robot_launcher

source ./install/setup.bash
ros2 launch my_robot_launcher robot.launch.py
wait
echo