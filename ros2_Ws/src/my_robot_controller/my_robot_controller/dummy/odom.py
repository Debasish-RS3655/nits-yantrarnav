#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import random

class DummyOdomPublisher(Node):
    def __init__(self):
        super().__init__('dummy_odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/rtabmap/odom', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10 Hz
        self.get_logger().info("Dummy Odometry Publisher Started.")

    def timer_callback(self):
        msg = Odometry()
        # Generate random coordinates between 0 and 10
        msg.pose.pose.position.x = random.uniform(0, 10)
        msg.pose.pose.position.y = random.uniform(0, 10)
        msg.pose.pose.position.z = random.uniform(0, 10)
        # Use an identity quaternion for orientation
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published odom: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, z={msg.pose.pose.position.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DummyOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down dummy odom publisher.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
