#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# this is the main endpoint where the target positions will be determined from the current position

class PathPlanner(Node):
    
    def __init__(self):
        super().__init__('path_planner')
        self.pos_target_pub = self.create_publisher(String, 'position/target', 10)        # data type, topic name and queue size
        # path planner also needs to subscribe to the current position from the position publisher
                
        
        # initialize current position 
        self.x_ = 0
        self.y_ = 0
        self.z = 0
        
        self.pos_current_sub = self.create_subscription(String, 'position/current', self.update_pos, 10)                        
                
        # Target positions (will be updated via subscription)
        self.target_x = 50
        self.target_y = 50
        self.target_z = 50
                
        # self.counter_ = 30        
        self.timer_ = self.create_timer(1, self.send_pos)
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
                self.get_logger().info(
                    f'Updated position: x={self.x_}, y={self.y_}, z={self.z_}'
                )
            except Exception as e:
                self.get_logger().error('Failed to parse target position: ' + str(e))

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