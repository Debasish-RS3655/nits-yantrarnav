#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math

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
            self.get_logger_
