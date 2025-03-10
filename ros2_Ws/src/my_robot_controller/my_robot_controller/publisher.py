#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publish(Node):
    
    def __init__(self):
        super().__init__('publisher')
        self.data_pub_ = self.create_publisher(String, 'chatter', 10)        # data type, topic name and queue size
        self.counter_ = 0
        self.timer_ = self.create_timer(0.5, self.send_data)
        self.get_logger().info('Node Publish has been started.')

    def send_data(self):        
        msg = String()
        msg.data = 'Publisher transmitting: ' + str(self.counter_)
        self.counter_ += 1
        self.data_pub_.publish(msg)        
        
def main(args=None):
    rclpy.init(args=args)
    node = Publish()
    rclpy.spin(node)
    # node.destroy_node() # clean up
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()