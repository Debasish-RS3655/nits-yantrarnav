#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# define all the necessary topics here

# subscribes to the necessary topics 
topic_list = []

class LaunchTopicMonitor(Node):
    def __init__(self):
        super().__init__('launch_topic_monitor')

        # List of required topics to check for.
        self.topic_list = topic_list        
        self.displayed = False

        # Publisher for the system launch status.
        self.publisher = self.create_publisher(Bool, '/system_launch_status', 10)

        # Timer to periodically check if all required topics exist.
        self.timer = self.create_timer(1.0, self.state_updater)

    def state_updater(self):
        # Get the list of all currently active topics.
        topics_and_types = self.get_topic_names_and_types()
        # Extract the topic names from the returned list of tuples.
        existing_topics = [topic for topic, types in topics_and_types]

        # Check if all required topics exist.
        if all(topic in existing_topics for topic in self.topic_list):
            out_msg = Bool(data=True)
            if not self.displayed:
                self.get_logger().info("All required topics are available. System is ready.")
                self.displayed = True
        else:
            out_msg = Bool(data=False)

        self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaunchTopicMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



"""
'/camera/camera/accel/imu_info','/camera/camera/accel/sample','/camera/camera/depth/camera_info','/camera/camera/depth/image_rect_raw','/camera/camera/depth/metadata','/camera/camera/accel/metadata',
              '/camera/camera/color/camera_info','/camera/camera/color/metadata','/camera/camera/color/image_raw','/camera/camera/extrinsics/depth_to_accel','/camera/camera/extrinsics/depth_to_color',
              '/camera/camera/extrinsics/depth_to_gyro','/camera/camera/gyro/imu_info','/camera/camera/gyro/metadata','/camera/camera/gyro/sample','/camera/camera/imu','/clicked_point','/diagnostics',
              '/goal_pose','/gps/fix','/imu/data','/initialpose','/landing_spots','/mavros/actuator_control','/mavros/adsb/send','/mavros/adsb/vehicle','/mavros/altitude','/mavros/battery','/mavros/cam_imu_sync/cam_imu_stamp',
              '/mavros/camera/image_captured','/mavros/cellular_status/status','/mavros/companion_process/status','/mavros/debug_value/debug','/mavros/debug_value/debug_float_array','/mavros/debug_value/debug_vector',
              '/mavros/debug_value/named_value_float','/mavros/debug_value/named_value_int','/mavros/debug_value/send','/mavros/esc_status/info','/mavros/esc_status/status','/mavros/esc_telemetry/telemetry',
              '/mavros/estimator_status','/mavros/extended_state','/mavros/fake_gps/mocap/tf','/mavros/geofence/fences','/mavros/gimbal_control/device/attitude_status','/mavros/gimbal_control/device/info',
              '/mavros/gimbal_control/device/set_attitude','/mavros/gimbal_control/manager/info','/mavros/gimbal_control/manager/set_attitude','/mavros/gimbal_control/manager/set_manual_control',
              '/mavros/gimbal_control/manager/set_pitchyaw','/mavros/gimbal_control/manager/status','/mavros/global_position/compass_hdg','/mavros/global_position/global','/mavros/global_position/gp_lp_offset',
              '/mavros/global_position/gp_origin','/mavros/global_position/local','/mavros/global_position/raw/fix','/mavros/global_position/raw/gps_vel','/mavros/global_position/raw/satellites',
              '/mavros/global_position/rel_alt','/mavros/global_position/set_gp_origin','/mavros/gps_input/gps_input','/mavros/gps_rtk/rtk_baseline','/mavros/gps_rtk/send_rtcm','/mavros/gpsstatus/gps1/raw',
              '/mavros/gpsstatus/gps1/rtk','/mavros/gpsstatus/gps2/raw','/mavros/gpsstatus/gps2/rtk','/mavros/hil/actuator_controls','/mavros/hil/controls','/mavros/hil/gps','/mavros/hil/imu_ned',
              '/mavros/hil/optical_flow','/mavros/hil/rc_inputs','/mavros/hil/state','/mavros/home_position/home','/mavros/home_position/set','/mavros/imu/data','/mavros/imu/data_raw','/mavros/imu/diff_pressure',
              '/mavros/imu/mag','/mavros/imu/static_pressure','/mavros/imu/temperature_baro','/mavros/imu/temperature_imu','/mavros/landing_target/lt_marker','/mavros/landing_target/pose','/mavros/landing_target/pose_in',
              '/mavros/local_position/accel','/mavros/local_position/odom','/mavros/local_position/pose','/mavros/local_position/pose_cov','/mavros/local_position/velocity_body','/mavros/local_position/velocity_body_cov',
              '/mavros/local_position/velocity_local','/mavros/log_transfer/raw/log_data','/mavros/log_transfer/raw/log_entry','/mavros/mag_calibration/report','/mavros/mag_calibration/status','/mavros/manual_control/control',
              '/mavros/manual_control/send','/mavros/mission/reached','/mavros/mission/waypoints','/mavros/mocap/pose','/mavros/mocap/tf','/mavros/mount_control/command','/mavros/mount_control/orientation',
              '/mavros/mount_control/status','/mavros/nav_controller_output/output','/mavros/obstacle/send','/mavros/odometry/in','/mavros/odometry/out','/mavros/onboard_computer/status','/mavros/optical_flow/ground_distance',
              '/mavros/optical_flow/raw/optical_flow','/mavros/optical_flow/raw/send','/mavros/param/event','/mavros/play_tune','/mavros/px4flow/ground_distance','/mavros/px4flow/raw/optical_flow_rad','/mavros/px4flow/raw/send',
              '/mavros/px4flow/temperature','/mavros/radio_status','/mavros/rallypoint/rallypoints','/mavros/rc/in','/mavros/rc/out','/mavros/rc/override','/mavros/setpoint_accel/accel','/mavros/setpoint_attitude/cmd_vel',
              '/mavros/setpoint_attitude/thrust','/mavros/setpoint_position/global','/mavros/setpoint_position/global_to_local','/mavros/setpoint_position/local','/mavros/setpoint_raw/attitude','/mavros/setpoint_raw/global',
              '/mavros/setpoint_raw/local','/mavros/setpoint_raw/target_attitude','/mavros/setpoint_raw/target_global','/mavros/setpoint_raw/target_local','/mavros/setpoint_trajectory/desired','/mavros/setpoint_trajectory/local',
              '/mavros/setpoint_velocity/cmd_vel','/mavros/setpoint_velocity/cmd_vel_unstamped','/mavros/state','/mavros/status_event','/mavros/statustext/recv','/mavros/statustext/send','/mavros/sys_status','/mavros/target_actuator_control',
              '/mavros/terrain/report','/mavros/time_reference','/mavros/timesync_status','/mavros/trajectory/desired','/mavros/trajectory/generated','/mavros/trajectory/path','/mavros/tunnel/in','/mavros/tunnel/out','/mavros/vfr_hud',
              '/mavros/vision_pose/pose','/mavros/vision_pose/pose_cov','/mavros/vision_speed/speed_twist','/mavros/vision_speed/speed_twist_cov','/mavros/vision_speed/speed_vector','/mavros/wind_estimation','/move_base_simple/goal',
              '/parameter_events','/rosout','/rtabmap/cloud_ground','/rtabmap/cloud_map','/rtabmap/cloud_obstacles','/rtabmap/global_path','/rtabmap/global_path_nodes','/rtabmap/global_pose','/rtabmap/goal','/rtabmap/goal_node',
              '/rtabmap/goal_reached','/rtabmap/grid_prob_map','/rtabmap/info','/rtabmap/initialpose','/rtabmap/labels','/rtabmap/landmark_detection','/rtabmap/landmark_detections','/rtabmap/landmarks','/rtabmap/local_grid_empty',
              '/rtabmap/local_grid_ground','/rtabmap/local_grid_obstacle','/rtabmap/local_path','/rtabmap/local_path_nodes','/rtabmap/localization_pose','/rtabmap/map','/rtabmap/mapData','/rtabmap/mapGraph','/rtabmap/mapOdomCache',
              '/rtabmap/mapPath','/rtabmap/octomap_binary','/rtabmap/octomap_empty_space','/rtabmap/octomap_full','/rtabmap/octomap_global_frontier_space','/rtabmap/octomap_grid','/rtabmap/octomap_ground','/rtabmap/octomap_obstacles',
              '/rtabmap/octomap_occupied_space','/rtabmap/odom','/rtabmap/odom_info','/rtabmap/odom_info_lite','/rtabmap/odom_last_frame','/rtabmap/odom_local_map','/rtabmap/odom_local_scan_map','/rtabmap/odom_rgbd_image',
              '/rtabmap/odom_sensor_data/compressed','/rtabmap/odom_sensor_data/features','/rtabmap/odom_sensor_data/raw','/rtabmap/rtabmap/republish_node_data','/tf','/tf_static','/uas1/mavlink_sink','/uas1/mavlink_source'
"""