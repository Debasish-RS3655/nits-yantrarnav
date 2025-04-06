#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String, Float32

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
        # New publisher for z_velocity (vertical speed)
        self.z_velocity_pub = self.create_publisher(Float32, '/z_velocity', 10)

        # Variables to store current and previous height measurements
        self.current_height = None
        self.last_height = None
        self.last_time = None

        # Timer to compute the vertical speed every 0.1 seconds
        self.create_timer(0.1, self.compute_velocity_callback)

    def range_callback(self, msg: Range):
        # Extract the range (height) from the Range message
        current_range = msg.range
        # Publish the current height as a string
        height_msg = String()
        height_msg.data = str(current_range)
        self.height_pub.publish(height_msg)
        self.get_logger().info(f"Published current height: {height_msg.data}")

        # Update current height and record the time
        self.current_height = current_range
        current_time = self.get_clock().now().nanoseconds * 1e-9  # Convert nanoseconds to seconds
        # Initialize last values if this is the first measurement
        if self.last_height is None:
            self.last_height = current_range
            self.last_time = current_time

    def compute_velocity_callback(self):
        # Only compute if we have valid current and last height values
        if self.current_height is not None and self.last_time is not None:
            current_time = self.get_clock().now().nanoseconds * 1e-9  # Current time in seconds
            dt = current_time - self.last_time
            if dt > 0:
                # Compute vertical speed: difference in height divided by elapsed time
                z_velocity = (self.current_height - self.last_height) / dt
                velocity_msg = Float32()
                velocity_msg.data = z_velocity
                self.z_velocity_pub.publish(velocity_msg)
                self.get_logger().info(f"Published z_velocity: {z_velocity:.3f} m/s")
                # Update last measurement for next computation
                self.last_height = self.current_height
                self.last_time = current_time

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
