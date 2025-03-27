#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('Coordinate_Pub')
        
        # Subscribe to the /rtabmap/odom topic using Odometry messages.
        self.rtab_sub = self.create_subscription(
            Odometry, 
            '/rtabmap/odom', 
            self.updateRtabCoord, 
            10
        )
        
        # Publishers for current position and origin.
        self.current_pub = self.create_publisher(String, 'position/current', 10)
        self.origin_pub = self.create_publisher(String, 'position/origin', 10)
        
        # Subscriber for the launch/land status
        self.status_sub = self.create_subscription(
            String,
            'launch_land_status',
            self.updateLaunchStatus,
            10
        )
        
        # Timer to continuously publish origin coordinates (every 0.1 sec).
        self.origin_timer = self.create_timer(0.1, self.publish_origin)
        
        # Initialize current and origin coordinates.
        self.x_ = 0.0
        self.y_ = 0.0
        self.z_ = 0.0
        
        self.launch_status = None  # Current status string
        
        self.x_origin = None
        self.y_origin = None
        self.z_origin = None
        
    def updateLaunchStatus(self, msg: String):
        # Acceptable statuses.
        valid_statuses = ['landed', 'launched', 'landing', 'launching']
        new_status = msg.data.strip().lower()
        
        if new_status not in valid_statuses:
            self.get_logger().warn(f"Received invalid launch/land status: {new_status}")
            return
        
        # Check for the transition from "launching" to "launched"
        if self.launch_status == "launching" and new_status == "launched":
            # Set the origin only if it hasn't been set yet.
            if self.x_origin is None:
                self.x_origin = self.x_
                self.y_origin = self.y_
                self.z_origin = self.z_
                origin_position_str = f"x={self.x_origin} y={self.y_origin} z={self.z_origin}"
                self.origin_pub.publish(String(data=origin_position_str))
                self.get_logger().info(f"Origin set on launch transition: {origin_position_str}")
                
        self.launch_status = new_status
        self.get_logger().info(f"Updated launch/land status: {self.launch_status}")

    def updateRtabCoord(self, msg: Odometry):
        # Update x, y, z with values from msg.pose.pose.position
        self.x_ = msg.pose.pose.position.x
        self.y_ = msg.pose.pose.position.y
        self.z_ = msg.pose.pose.position.z
        
        # Publish the current position as a string.
        current_position_str = f"x={self.x_} y={self.y_} z={self.z_}"
        self.current_pub.publish(String(data=current_position_str))
        self.get_logger().info(f"Published Rtab coordinates: {current_position_str}")
        
        # Set the origin to the first received position if not already set AND if launch status is already "launched"
        # (in case the status transition happened before any odometry messages were received)
        if self.x_origin is None and self.launch_status == "launched":
            self.x_origin = self.x_
            self.y_origin = self.y_
            self.z_origin = self.z_
            origin_position_str = f"x={self.x_origin} y={self.y_origin} z={self.z_origin}"
            self.origin_pub.publish(String(data=origin_position_str))
            self.get_logger().info(f"Origin set on receiving odometry: {origin_position_str}")

    def publish_origin(self):
        # Publish the origin coordinates every 0.1 second if they have been set.
        if self.x_origin is not None:
            origin_position_str = f"x={self.x_origin} y={self.y_origin} z={self.z_origin}"
            self.origin_pub.publish(String(data=origin_position_str))
            self.get_logger().debug(f"Published origin: {origin_position_str}")

def main(args=None):
    rclpy.init(args=args)
    coordinate_publisher = CoordinatePublisher()
    rclpy.spin(coordinate_publisher)
    coordinate_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
