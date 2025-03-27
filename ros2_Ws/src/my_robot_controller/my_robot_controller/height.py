#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String

class RangeToHeightPublisher(Node):
    def __init__(self):
        super().__init__('range_to_height_publisher')
        # Subscriber to the rangefinder topic
        self.create_subscription(
            Range,
            '/mavros/rangefinder/rangefinder',
            self.range_callback,
            10
        )
        # Publisher for the current height as a string
        self.height_pub = self.create_publisher(String, '/current_height', 10)

    def range_callback(self, msg: Range):
        # Extract the range field
        current_range = msg.range
        # Create a string message
        height_msg = String()
        height_msg.data = str(current_range)
        # Publish the string message
        self.height_pub.publish(height_msg)
        self.get_logger().info(f"Published current height: {height_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = RangeToHeightPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt. Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
