#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 1) MAVROS launch
    ld.add_action(
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'mavros', 'apm.launch.py',
                'fcu_url:=/dev/ttyACM1:921600',
                'use_sim_time:=false',
                'config_yaml:='+str(Path.home()/'mavros_config'/'custom_mavros_config.yaml')
            ],
            output='screen'
        )
    )

    # 2) after 30s, set stream rate 10
    ld.add_action(
        TimerAction(
            period=30.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/mavros/set_stream_rate',
                        'mavros_msgs/srv/StreamRate',
                        '{"stream_id": 10, "message_rate": 200, "on_off": true}'
                    ],
                    output='screen'
                )
            ]
        )
    )

    # 3) after 37s (30+7), set stream rate 1
    ld.add_action(
        TimerAction(
            period=37.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/mavros/set_stream_rate',
                        'mavros_msgs/srv/StreamRate',
                        '{"stream_id": 1, "message_rate": 200, "on_off": true}'
                    ],
                    output='screen'
                )
            ]
        )
    )

    # 4) after 44s (30+7+7), launch realsense
    ld.add_action(
        TimerAction(
            period=44.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                        'color_width:=640', 'color_height:=360',
                        'depth_width:=640', 'depth_height:=360',
                        'color_fps:=30', 'depth_fps:=30',
                        'unite_imu_method:=1', 'align_depth:=true',
                        'sync:=false', 'enable_gyro:=true',
                        'enable_accel:=true', 'gyro_fps:=200',
                        'accel_fps:=200', 'enable_pointcloud:=true'
                    ],
                    output='screen'
                )
            ]
        )
    )

    # 5) at 59s (44+15)
    ld.add_action(
        TimerAction(
            period=59.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'launch', 'static_transforms_pkg',
                        'static_transforms.launch.py'
                    ],
                    output='screen'
                )
            ]
        )
    )

    # 6) at 69s (59+10): clear db then launch rtabmap
    ld.add_action(
        TimerAction(
            period=69.0,
            actions=[
                ExecuteProcess(cmd=['rm','-f','~/.ros/rtabmap.db']),
                ExecuteProcess(
                    cmd=[
                        'ros2', 'launch', 'rtabmap_launch',
                        'rtabmap.launch.py',
                        'rtabmap_viz:=false',
                        'rgb_topic:=/camera/camera/color/image_raw',
                        'depth_topic:=/camera/camera/depth/image_rect_raw',
                        'camera_info_topic:=/camera/camera/color/camera_info',
                        'imu_topic:=/mavros/imu/data',
                        'approx_sync:=true',
                        'approx_sync_max_interval:=0.01',
                        'rtabmap_args:="--Rtabmap/DetectionRate=2 --Vis/EstimationRate=30 '
                        '--Vis/FeatureType=4 --Vis/MaxFeatures=150 --Vis/MinInliers=8 '
                        '--Vis/EstimationType=1 --RGBD/Decimation=3 --Rtabmap/CreateIntermediateNodes=false '
                        '--Mem/InitWMWithAllNodes false --Mem/DepthAsMask false '
                        '--Vis/DepthAsMask false --Rtabmap/MaxRepublished 0"'
                    ],
                    output='screen'
                )
            ]
        )
    )

    # 7) at 74s (69+5): EKF IMU fusion
    ld.add_action(
        TimerAction(
            period=74.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'launch', 'ekf_imu_fusion_pkg',
                        'ekf_imu_fusion.launch.py'
                    ],
                    output='screen'
                )
            ]
        )
    )

    # 8) immediately after (you could add delay if you want)
    ld.add_action(
        TimerAction(
            period=74.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'v4l2_camera', 'v4l2_camera_node'],
                    output='screen'
                )
            ]
        )
    )

    # 9–11) guided mode, arm, takeoff—chain them with small pauses
    ld.add_action(
        TimerAction(
            period=75.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/mavros/set_mode',
                        'mavros_msgs/srv/SetMode',
                        '{"base_mode":0,"custom_mode":"GUIDED"}'
                    ],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/mavros/cmd/arming',
                        'mavros_msgs/srv/CommandBool',
                        '{"value": true}'
                    ],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/mavros/cmd/takeoff',
                        'mavros_msgs/srv/CommandTOL',
                        '{"min_pitch":0.0,"yaw":0.0,"latitude":0.0,"longitude":0.0,"altitude":1.8}'
                    ],
                    output='screen'
                ),
            ]
        )
    )

    return ld
