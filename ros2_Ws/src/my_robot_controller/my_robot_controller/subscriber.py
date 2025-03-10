#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscriber(Node):
    
    def __init__(self):
        super().__init__("subscriber")
        self.subscriber = self.create_subscription(String, 'chatter', self.subscriber_callback, 10)
        self.get_logger().info("Node Subscribe has been started.")
            
    def subscriber_callback(self, msg: String):
        self.get_logger().info(str(msg.data))

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()