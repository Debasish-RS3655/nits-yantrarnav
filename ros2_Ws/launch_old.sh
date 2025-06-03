#!/usr/bin/env bash

#
# launch_all_terminals.sh
#
# Opens a separate gnome-terminal window for each ROS 2 command,
# waits the specified buffer time between launches, and prints a status message
# in the main script after each terminal is opened.

### 0) Source ROS 2 and your workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash   # adjust path if needed

### Helper to launch a command in its own gnome-terminal
_launch_term() {
  local cmd="$1"
  gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ${cmd}; exec bash"
}

### 1) Command 1: Launch MAVROS
# (buffer time to fully initialise: 30 seconds)
_launch_term "ros2 launch mavros apm.launch \
  fcu_url:=/dev/ttyACM1:921600 \
  use_sim_time:=false \
  config_yaml:=~/mavros_config/custom_mavros_config.yaml"
echo "[1/7] MAVROS initialized successfully"
sleep 30

### 2) Command 2: Set stream rate 10
# (buffer time to fully initialise: 7 sec)
_launch_term "ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
  \"{stream_id: 10, message_rate: 200, on_off: true}\""
echo "[2/7] IMU 1 streaming accelerated to 200Hz"
sleep 7

### 3) Command 3: Set stream rate 1
# (buffer time to fully initialise: 7 sec)
_launch_term "ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
  \"{stream_id: 1, message_rate: 200, on_off: true}\""
echo "[3/7] IMU 2 streaming accelerated to 200Hz"
sleep 7

### 4) Command 4: Launch RealSense
# (buffer time to fully initialise: 15 sec)
_launch_term "ros2 launch realsense2_camera rs_launch.py \
  color_width:=640 \
  color_height:=360 \
  depth_width:=640 \
  depth_height:=360 \
  color_fps:=30 \
  depth_fps:=30 \
  unite_imu_method:=1 \
  align_depth:=true \
  sync:=false \
  enable_gyro:=true \
  enable_accel:=true \
  gyro_fps:=200 \
  accel_fps:=200 \
  enable_pointcloud:=true"
echo "[4/7] Intel RealSense D455 initialized successfully"
sleep 15

### 5) Command 5: Launch static transforms
# (buffer time to fully initialise: 10 sec)
_launch_term "ros2 launch static_transforms_pkg static_transforms.launch.py"
echo "[5/7] Static transform initialized for ANAV"
sleep 10

## 6) Command 6: Clear RTAB-Map DB & launch RTAB-Map
(buffer time to fully initialise: 10 sec)
## 6) Command 6: Clear RTAB-Map DB & launch RTAB-Map
(buffer time to fully initialise: 10 sec)
## 6) Command 6: Clear RTAB-Map DB & launch RTAB-Map
# (buffer time to fully initialise: 10 sec)
_launch_term "rm -f ~/.ros/rtabmap.db && \
  ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_viz:=false \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/depth/image_rect_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    imu_topic:=/mavros/imu/data \
    approx_sync:=true \
    approx_sync_max_interval:=0.01 \
    rtabmap_args:=\"--Rtabmap/DetectionRate=2 \
--Vis/EstimationRate=30 \
--Vis/FeatureType=4 \
--Vis/MaxFeatures=150 \
--Vis/MinInliers=8 \
--Vis/EstimationType=1 \
--RGBD/Decimation=3 \
--Rtabmap/CreateIntermediateNodes=false \
--Mem/InitWMWithAllNodes=false \
--Mem/DepthAsMask=false \
--Vis/DepthAsMask=false \
--Rtabmap/MaxRepublished=0\""
echo "[6/7] RTAB-Map initialized successfully, SLAM online"
sleep 10


### 7) Command 7: Launch EKF IMU fusion
# (buffer time to fully initialise: 5 sec)
_launch_term "ros2 launch ekf_imu_fusion_pkg ekf_imu_fusion.launch.py"
echo "[7/7] IMU–EKF fusion node for VIO initialized successfully"
sleep 5

### 8) Command 8: Launch downward-facing camera
_launch_term "ros2 run v4l2_camera v4l2_camera_node"
echo "[8] Downward-facing camera is launched"

### 9) Command 9: Set GUIDED mode
_launch_term "ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  \"{base_mode: 0, custom_mode: 'GUIDED'}\""
echo "[9] Guided mode set"

### 10) Command 10: Arm the drone
_launch_term "ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  \"{value: true}\""
echo "[10] Drone armed"

### 11) Command 11: Take off
_launch_term "ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL \
  \"{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 1.8}\""
echo "[11] Takeoff command sent"

# (Optional) Prevent script from exiting until all terminals are closed:
wait
