#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

# Dummy implementation for the flat area detector
# We assume a dummy area in the square in this implementation

# the flat area detector will be publish the flat areas when the phase is set to 3
# so for the manual mode to trigger flat area we publish the phase 3

class Flat_Area_Detector(Node):
    def __init__(self):
        super().__init__('flat_area')
        self.pos_current_sub = self.create_subscription(String, 'position/current', self.update_current_pos, 10)
        self.phase_sub = self.create_subscription(String, 'position/phase', self.phase_change, 10)
        
        self.flat_area_pub = self.create_publisher(String, 'position/flat_area', 10)
        
        # Updated by subscription
        self.phase = 1
                
        # Dummy implementation.. in reality, we would get this from the boundary mapper publisher
        self.side = 50.0  # Side length for square and lawnmower area

        # Current position
        self.x_ = 0.0
        self.y_ = 0.0
        self.z_ = 0.0

        # Record the starting origin
        self.origin_x = self.x_
        self.origin_y = self.y_
        self.origin_z = self.z_
        
        # display the starting log to the screen
        self.get_logger().info('Flat Area node has been started.')

    def update_current_pos(self, msg: String):
        # Expect message format: "x=<value> y=<value> z=<value>"
        try:
            parts = msg.data.split()
            for part in parts:
                key, value = part.split('=')
                value = float(value)
                if key == 'x':
                    self.x_ = value
                elif key == 'y':
                    self.y_ = value
                elif key == 'z':
                    self.z_ = value
            # self.get_logger().info(f'Updated position: x={self.x_}, y={self.y_}, z={self.z_}')
        except Exception as e:
            self.get_logger().error('Failed to parse current position: ' + str(e))
            
    def phase_change(self, phase_msg: String):            
        self.phase = int(phase_msg.data)
        
        if self.phase == 3:
            # flat area detection logic goes here
            for _ in range(2):
                flat_area = self.determine_random_area()
                self.publish_flat_area(flat_area)

    # Dummy implementation, we assume random areas inside the square as flat landing areas
    def determine_random_area(self):
        x0, y0, z0 = self.origin_x, self.origin_y, self.origin_z
        rx = random.uniform(x0, x0 + self.side)
        ry = random.uniform(y0, y0 + self.side)
        rz = z0  # Assuming the flat area is on the ground level
        return rx, ry, rz

    def publish_flat_area(self, flat_area):
        msg = String()
        msg.data = f'x={flat_area[0]} y={flat_area[1]} z={flat_area[2]}'
        self.flat_area_pub.publish(msg)
        self.get_logger().info(f'Flat area published: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = Flat_Area_Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()