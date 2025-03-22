#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import time

# hovering mode test

class HoverController(Node):
    def __init__(self):
        super().__init__('hover_controller')

        # Subscribers
        self.mode_sub = self.create_subscription(String, 'position/mode', self.mode_callback, 10)
        self.current_sub = self.create_subscription(String, 'position/current', self.current_callback, 10)
        self.origin_sub = self.create_subscription(String, 'position/origin', self.origin_callback, 10)

        # Publishers
        self.hover_pub = self.create_publisher(String, '/hovering_mode', 10)
        self.target_pub = self.create_publisher(String, 'position/target', 10)

        # State variables
        self.mode = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_z = 0.0

        # Target coordinate (z) variables
        self.target_z = None  # Will be initialized on the first hover_loop call
        self.delta = 0.1      # Constant delta for z-coordinate adjustment
        self.threshold = 0.5  # Threshold to determine if hover process is complete
        self.tolerance = 0.05 # Tolerance to consider current_z has reached target_z

        self.hover_active = False  # Flag to control the hover loop
        self.ascent_completed = False  # Flag to indicate if ascent is completed
        self.descent_started = False  # Flag to indicate if descent has started

        # Timer for hover loop (10 Hz)
        self.timer = self.create_timer(0.1, self.hover_loop)

    def mode_callback(self, msg):
        self.mode = msg.data
        if self.mode == "hover" and not self.hover_active:
            self.get_logger().info("Hover mode detected. Starting hover process.")
            self.hover_pub.publish(String(data="ongoing"))
            self.hover_active = True

    def current_callback(self, msg):
        try:
            # Parse current position message
            data = msg.data
            coords = data.split()
            self.current_x = float(coords[0].split('=')[1])
            self.current_y = float(coords[1].split('=')[1])
            self.current_z = float(coords[2].split('=')[1])
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Failed to parse current position: {e}")

    def origin_callback(self, msg):
        try:
            # Parse origin position message
            data = msg.data
            coords = data.split()
            self.origin_x = float(coords[0].split('=')[1])
            self.origin_y = float(coords[1].split('=')[1])
            self.origin_z = float(coords[2].split('=')[1])
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Failed to parse origin position: {e}")

    def hover_loop(self):
        if self.hover_active:
            if self.target_z is None:
                # Initialize target_z to current_z + delta
                self.target_z = self.current_z + self.delta
                self.publish_target()

            # Check if current_z has reached target_z within tolerance
            if abs(self.current_z - self.target_z) <= self.tolerance:
                if not self.ascent_completed:
                    # Update target_z to the next step
                    self.target_z += self.delta
                    self.publish_target()

                    # Check if the ascent is completed
                    if self.target_z >= self.origin_z + self.threshold:
                        self.ascent_completed = True
                        self.hover_pub.publish(String(data="COMPLETED"))
                        self.get_logger().info("Ascent completed. Waiting for 5 seconds before starting descent.")
                        time.sleep(5)  # Delay before starting descent
                        self.descent_started = True
                elif self.descent_started:
                    # Start decreasing target_z for descent
                    self.target_z -= self.delta
                    self.publish_target()

                    # Check if the descent is completed
                    if self.target_z <= self.origin_z:
                        self.hover_active = False
                        self.get_logger().info("Descent completed. Hover process finished.")

    def publish_target(self):
        # Publish the new target position
        target_msg = String()
        target_msg.data = f'x={self.origin_x} y={self.origin_y} z={self.target_z}'
        self.target_pub.publish(target_msg)
        self.get_logger().info(f'Published new target: {target_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = HoverController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()