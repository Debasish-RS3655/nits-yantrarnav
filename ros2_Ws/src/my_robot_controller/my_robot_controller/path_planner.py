#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PathPlanner(Node):
    
    def __init__(self):
        super().__init__('path_planner')
        self.pos_target_pub = self.create_publisher(String, 'position/target', 10)        # data type, topic name and queue size
        
        # initialize current position 
        self.x_ = 0
        self.y_ = 0
        self.z = 0
                
        # Target positions (will be updated via subscription)
        self.target_x = 50
        self.target_y = 50
        self.target_z = 50
                
        # self.counter_ = 30
        
        self.timer_ = self.create_timer(1, self.send_pos)
        self.get_logger().info('Path planner node has been started.')

    # publish the current position
    def send_pos(self):        
        msg = String()
        msg.data = f'x={self.target_x} y={self.target_y} z={self.target_z}'
        self.pos_target_pub.publish(msg)        
        self.get_logger().info(f'Target postion published: {msg.data}')   
        
def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node() # clean up
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()