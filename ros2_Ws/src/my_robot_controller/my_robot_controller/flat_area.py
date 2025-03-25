#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random

class FlatAreaNode(Node):
    def __init__(self):
        super().__init__('dummy_flat_area_publisher')
        # Publisher for flat area candidate points
        self.flat_area_pub = self.create_publisher(Point, '/landing_spots', 10)
        self.get_logger().info("Dummy Flat Area Publisher Node Initialized.")
        # Create a timer that calls the callback after 3 seconds (one-shot)
        self.create_timer(3.0, self.publish_flat_area_once)

    def publish_flat_area_once(self):
        # Generate a dummy flat area point.
        # Here we choose a random point within a predefined range.
        flat_point = Point()
        flat_point.x = round(random.uniform(0.0, 10.0), 2)
        flat_point.y = round(random.uniform(0.0, 10.0), 2)
        flat_point.z = 0.0  # Assuming flat areas are on ground (z = 0)
        
        self.flat_area_pub.publish(flat_point)
        self.get_logger().info(f"Published dummy flat area point: x={flat_point.x}, y={flat_point.y}, z={flat_point.z}")
        # Since this is a one-shot publisher, we can shut down the timer by calling rclpy.shutdown() if desired
        # or simply let the node continue running if further testing is needed.

def main(args=None):
    rclpy.init(args=args)
    node = FlatAreaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
