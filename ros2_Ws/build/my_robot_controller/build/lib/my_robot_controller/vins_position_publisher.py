#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# dummy implementation where the VINS fusion slam publishes the current coordinates of the drone

class VINS_Position_Publisher(Node):
    def __init__(self):
        super().__init__('vins_position_publisher')
        self