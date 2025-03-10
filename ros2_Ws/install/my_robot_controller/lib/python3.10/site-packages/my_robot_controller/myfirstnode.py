#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# the node is defined here
class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")  # name of the node        
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)     # timing loop with 1 second delay
        # log to console
        # self.get_logger().info("ROS2 node running")
        
    # function to be called inside the timer
    def timer_callback(self):
        self.get_logger().info("Linux sucks Windows rocks!!! " + str(self.counter_))    # log to console
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)   # init ros2 communication
    # nodes here    
    node = MyNode()
    rclpy.spin(node)    # kept alive indefinitely until killed
    rclpy.shutdown()    # destroy the node

if __name__ == "__main__":
    main()
