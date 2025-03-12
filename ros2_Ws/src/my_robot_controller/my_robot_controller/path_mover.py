#!usr/bin/env python3

# Debashish Buragohain
# dummy implementation for the mover code which moves to the target position sending commands to Pixhawk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PathMover(Node):    
    def __init__(self):
        super().__init__('path_mover')

        # publishing the current positions
        self.data_pub_ = self.create_publisher(String, 'position/current', 10)
        
        # positions to be actually read using the VINS-slam
        self.velocity_step = 4        
        # initial position values
        self.x_ = 0.0
        self.y_ = 0.0
        self.z_ = 0.0
        
        # Target positions (will be updated via subscription)
        self.target_x = self.x_
        self.target_y = self.y_
        self.target_z = self.z_
        
        self.timer_ = self.create_timer(0.5, self.send_pos)        
        self.target_pos_subscriber = self.create_subscription(String, 'position/target', self.update_pos, 10)
        
        self.get_logger().info('Position publisher node has been started.')

    # publish the current positions regularly
    def send_pos(self):                
        self.update_current_position()
        msg = String()
        msg.data = f'x={self.x_} y={self.y_} z={self.z_}'
        self.data_pub_.publish(msg)        
        self.get_logger().info(f'Current postion Published: {msg.data}')        
        
    # !! dummy implementation
    # in the actual implementation we 
    def update_current_position(self):
        # update X coord
        if abs(self.x_ - self.target_x) < self.velocity_step:
            self.x_ = self.target_x
        elif self.x_ < self.target_x:
            self.x_ += self.velocity_step
        else:
            self.x_ -= self.velocity_step
            
        # Update Y coordinate
        if abs(self.y_ - self.target_y) < self.velocity_step:
            self.y_ = self.target_y
        elif self.y_ < self.target_y:
            self.y_ += self.velocity_step
        else:
            self.y_ -= self.velocity_step

        # Update Z coordinate
        if abs(self.z_ - self.target_z) < self.velocity_step:
            self.z_ = self.target_z
        elif self.z_ < self.target_z:
            self.z_ += self.velocity_step
        else:
            self.z_ -= self.velocity_step


    def update_pos(self, msg: String):
        # Expecting message format: "x=<value> y=<value> z=<value>"
        try:
            parts = msg.data.split()
            for part in parts:
                key, value = part.split('=')
                value = float(value)
                if key == 'x':
                    self.target_x = value
                elif key == 'y':
                    self.target_y = value
                elif key == 'z':
                    self.target_z = value
            self.get_logger().info(
                f'Received new target: x={self.target_x}, y={self.target_y}, z={self.target_z}'
            )
        except Exception as e:
            self.get_logger().error('Failed to parse target position: ' + str(e))    
        
def main(args=None):
    rclpy.init(args=args)
    node = PathMover()
    rclpy.spin(node)
    node.destroy_node() # clean up
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()