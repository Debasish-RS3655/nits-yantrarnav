#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('coordinate_publisher')
        
        # Subscribe to the vision_pose topic
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.update_pose,
            10
        )
        
        # Publishers for position, origin, and orientation
        self.current_pub = self.create_publisher(String, 'position/current', 10)
        self.origin_pub = self.create_publisher(String, 'position/origin', 10)
        self.orientation_pub = self.create_publisher(String, 'position/orientation', 10)
        
        # Subscriber for launch/land status
        self.status_sub = self.create_subscription(
            String,
            'launch_land_status',
            self.update_launch_status,
            10
        )
        
        # Timer to publish origin periodically
        self.origin_timer = self.create_timer(1.0, self.publish_origin)
        
        # Internal state
        self.x_ = 0.0
        self.y_ = 0.0
        self.z_ = 0.0
        self.ox_ = 0.0
        self.oy_ = 0.0
        self.oz_ = 0.0
        self.ow_ = 1.0
        self.launch_status = None
        self.x_origin = None
        self.y_origin = None
        self.z_origin = None
        self.recent_coords = []

    def update_launch_status(self, msg: String):
        valid = ['landed', 'launched', 'landing', 'launching']
        status = msg.data.strip().lower()
        if status not in valid:
            self.get_logger().warn(f"Invalid status: {status}")
            return
        # On transition launching->launched, set origin
        if self.launch_status == 'launching' and status == 'launched':
            if self.x_origin is None and (self.x_ or self.y_ or self.z_):
                self.set_origin()
        self.launch_status = status
        self.get_logger().info(f"Launch status: {self.launch_status}")

    def update_pose(self, msg: PoseStamped):
        # Extract position
        new_x = msg.pose.position.x
        new_y = msg.pose.position.y
        new_z = msg.pose.position.z
        # Handle zero readings
        if new_x == 0.0 and new_y == 0.0 and new_z == 0.0:
            for coord in reversed(self.recent_coords):
                if any(coord):
                    new_x, new_y, new_z = coord
                    self.get_logger().debug("Using last valid position.")
                    break
        else:
            self.recent_coords.append((new_x, new_y, new_z))
            if len(self.recent_coords) > 10:
                self.recent_coords.pop(0)
        self.x_, self.y_, self.z_ = new_x, new_y, new_z
        # Extract orientation
        self.ox_ = msg.pose.orientation.x
        self.oy_ = msg.pose.orientation.y
        self.oz_ = msg.pose.orientation.z
        self.ow_ = msg.pose.orientation.w
        # Publish if valid
        if any((self.x_, self.y_, self.z_)):
            pos_str = f"x={self.x_} y={self.y_} z={self.z_}"
            ori_str = f"ox={self.ox_} oy={self.oy_} oz={self.oz_} ow={self.ow_}"
            self.current_pub.publish(String(data=pos_str))
            self.orientation_pub.publish(String(data=ori_str))
            self.get_logger().info(f"Published pos: {pos_str} and ori: {ori_str}")
        # Set origin if launched
        if self.x_origin is None and self.launch_status == 'launched' and any((self.x_, self.y_, self.z_)):
            self.set_origin()

    def set_origin(self):
        self.x_origin = self.x_
        self.y_origin = self.y_
        self.z_origin = self.z_
        origin_str = f"x={self.x_origin} y={self.y_origin} z={self.z_origin}"
        self.origin_pub.publish(String(data=origin_str))
        self.get_logger().info(f"Origin set: {origin_str}")

    def publish_origin(self):
        if self.x_origin is not None:
            origin_str = f"x={self.x_origin} y={self.y_origin} z={self.z_origin}"
            self.origin_pub.publish(String(data=origin_str))
            self.get_logger().debug(f"Published origin: {origin_str}")


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
