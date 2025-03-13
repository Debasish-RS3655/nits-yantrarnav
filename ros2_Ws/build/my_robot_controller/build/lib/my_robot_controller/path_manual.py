#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Debashish Buragohain
# In manual mode we keep the target coordinates as the current coordinates itself

class ManualMode(Node):
    def __init__(self):
        super().__init__('manual_mode')
        # Publisher for target position
        self.pos_target_pub = self.create_publisher(String, 'position/target', 10)
        # Subscriber for current position
        self.pos_current_sub = self.create_subscription(String, 'position/current', self.update_pos, 10)
        
        # Initialize target and current positions
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        self.x_ = 0.0
        self.y_ = 0.0
        self.z_ = 0.0

    def update_pos(self, msg: String):
        """
        Parse the incoming message in the format:
        "x=value y=value z=value"
        Update the current position and set target to the same.
        Then publish the target position.
        """
        try:
            # Split the message by spaces
            parts = msg.data.split()
            for part in parts:
                # Split by "=" and remove extra whitespace
                key, value = part.split('=')
                key = key.strip().lower()
                value = float(value.strip())
                if key == 'x':
                    self.x_ = value
                elif key == 'y':
                    self.y_ = value
                elif key == 'z':
                    self.z_ = value
            # In manual mode, target equals current position
            self.target_x = self.x_
            self.target_y = self.y_
            self.target_z = self.z_
            
            # Publish the target position as a string message
            out_msg = String()
            out_msg.data = f"x={self.target_x} y={self.target_y} z={self.target_z}"
            self.pos_target_pub.publish(out_msg)
            self.get_logger().info(f"Published target: {out_msg.data}")
        except Exception as e:
            self.get_logger().error("Error parsing current position: " + str(e))


def main(args=None):
    rclpy.init(args=args)
    node = ManualMode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
