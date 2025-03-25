#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import random

class DummyBoundaryMapper(Node):
    def __init__(self):
        super().__init__('dummy_phase_publisher')
        # Publisher for yellow boundary signal (Phase 0)
        self.yellow_pub = self.create_publisher(Bool, '/yellow_boundary_found', 10)
        # Publisher for edge coordinates (Phase 1)
        self.edge_pub = self.create_publisher(String, '/edge_coordinates', 10)
        
        # Create a one-shot timer to simulate Phase 0 completion after 5 seconds
        self.yellow_timer = self.create_timer(5.0, self.publish_yellow_boundary_once)
        # Create a one-shot timer to simulate Phase 1 after 10 seconds (from startup)
        self.edge_timer = self.create_timer(10.0, self.publish_edge_coordinates_once)
        
        self.yellow_timer.cancel()  # Cancel immediately; we'll use a one-shot approach below
        self.edge_timer.cancel()
        # Instead, we'll schedule the callbacks using a simple delay mechanism:
        self.create_timer(5.0, self.publish_yellow_boundary_once)
        self.create_timer(10.0, self.publish_edge_coordinates_once)
        
        self.get_logger().info("DummyBoundaryMapper node started. Waiting to simulate phases...")

    def publish_yellow_boundary_once(self):
        # This function publishes the yellow boundary found signal once
        msg = Bool()
        msg.data = True
        self.yellow_pub.publish(msg)
        self.get_logger().info("Phase 0 complete: Published yellow boundary found signal.")
        # Cancel the timer by not doing anything further (one-shot behavior achieved)
        # Note: Since create_timer returns a timer and we don't store it, the callback will be called once if we set it up carefully.

    def publish_edge_coordinates_once(self):
        # Generate 4 random edge coordinates and publish as a comma-separated string.
        edges = []
        for i in range(4):
            x = round(random.uniform(0, 10), 2)
            y = round(random.uniform(0, 10), 2)
            z = 0.0  # for simplicity, we use 0 for z
            edge_str = f"x={x} y={y} z={z}"
            edges.append(edge_str)
        msg_data = ", ".join(edges)
        msg = String()
        msg.data = msg_data
        self.edge_pub.publish(msg)
        self.get_logger().info(f"Phase 1 complete: Published edge coordinates: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = DummyBoundaryMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
