#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import random

class PathPlanner(Node):

    def __init__(self):
        super().__init__('path_planner')
        self.pos_target_pub = self.create_publisher(String, 'position/target', 10)
        self.pos_current_sub = self.create_subscription(String, 'position/current', self.update_pos, 10)

        # Initialize current position and record the starting origin.
        self.x_ = 0.0
        self.y_ = 0.0
        self.z_ = 0.0

        self.origin_x = self.x_
        self.origin_y = self.y_
        self.origin_z = self.z_

        # Parameters for target generation
        self.side = 10.0   # side length for square and lawnmower area
        self.lawn_gap = 2.0  # gap between lines in the lawnmower pattern
        self.threshold = 0.5  # distance threshold to switch to the next target

        # Phase management: phase 1 = square, phase 2 = lawnmower, phase 3 = random, phase 4 = return
        self.phase = 1
        self.target_list = []
        self.target_index = 0
        self.current_target = None

        # Setup the first phase
        self.setup_phase(self.phase)

        # Create a timer that calls send_pos at regular intervals.
        self.timer_ = self.create_timer(1.0, self.send_pos)
        self.get_logger().info('Path planner node has been started.')

    def update_pos(self, msg: String):
        # Expecting message format: "x=<value> y=<value> z=<value>"
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
            self.get_logger().info(f'Updated position: x={self.x_}, y={self.y_}, z={self.z_}')
        except Exception as e:
            self.get_logger().error('Failed to parse current position: ' + str(e))

    def setup_phase(self, phase):
        """Set up the target list for the given phase."""
        if phase == 1:
            self.get_logger().info("Setting up Phase 1: Square Path")
            x0 = self.origin_x
            y0 = self.origin_y
            z0 = self.origin_z
            # Define the square vertices (clockwise or anticlockwise)
            self.target_list = [
                (x0 + self.side, y0, z0),
                (x0 + self.side, y0 + self.side, z0),
                (x0, y0 + self.side, z0),
                (x0, y0, z0)
            ]
        elif phase == 2:
            self.get_logger().info("Setting up Phase 2: Lawnmower Pattern")
            x0 = self.origin_x
            y0 = self.origin_y
            z0 = self.origin_z
            points = []
            # Calculate how many rows (back-and-forth sweeps)
            rows = int(self.side // self.lawn_gap) + 1
            for i in range(rows):
                y = y0 + i * self.lawn_gap
                if y > y0 + self.side:
                    y = y0 + self.side
                # Alternate direction each row
                if i % 2 == 0:
                    points.append((x0, y, z0))
                    points.append((x0 + self.side, y, z0))
                else:
                    points.append((x0 + self.side, y, z0))
                    points.append((x0, y, z0))
            self.target_list = points
        elif phase == 3:
            self.get_logger().info("Setting up Phase 3: Random Point in Square")
            x0 = self.origin_x
            y0 = self.origin_y
            z0 = self.origin_z
            rx = random.uniform(x0, x0 + self.side)
            ry = random.uniform(y0, y0 + self.side)
            self.target_list = [(rx, ry, z0)]
        elif phase == 4:
            self.get_logger().info("Setting up Phase 4: Return to Origin")
            self.target_list = [(self.origin_x, self.origin_y, self.origin_z)]
        else:
            self.get_logger().error("Unknown phase specified.")

        self.target_index = 0
        if self.target_list:
            self.current_target = self.target_list[self.target_index]
        else:
            self.get_logger().error("Target list is empty!")

    def send_pos(self):
        """Publish the current target. If the current position overlaps the target (within a threshold),
        move to the next target or phase."""
        if self.current_target is None:
            return

        # Compute Euclidean distance from current position to the target
        dx = self.x_ - self.current_target[0]
        dy = self.y_ - self.current_target[1]
        dz = self.z_ - self.current_target[2]
        distance = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        if distance < self.threshold:
            self.get_logger().info(f'Target reached: {self.current_target}')
            # Move to next target
            self.target_index += 1
            if self.target_index >= len(self.target_list):
                # Finished current phase; move to next phase if available.
                if self.phase < 4:
                    self.phase += 1
                    self.get_logger().info(f'Moving to Phase {self.phase}')
                    self.setup_phase(self.phase)
                else:
                    self.get_logger().info('Completed all phases.')
                    self.timer_.cancel()
                    return
            else:
                self.current_target = self.target_list[self.target_index]
                self.get_logger().info(f'Switching to next target: {self.current_target}')

        # Publish the current target position
        msg = String()
        msg.data = f'x={self.current_target[0]} y={self.current_target[1]} z={self.current_target[2]}'
        self.pos_target_pub.publish(msg)
        self.get_logger().info(f'Target published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()  # clean up
    rclpy.shutdown()

if __name__ == '__main__':
    main()
