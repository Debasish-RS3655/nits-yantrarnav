#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('Coordinate_Pub')
        
        # Subscribe to the /mavros/vision_pose/pose topic using PoseStamped messages.
        self.rtab_sub = self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.updateRtabCoord,
            10
        )
        
        # Publishers for current position and origin.
        self.current_pub = self.create_publisher(String, 'position/current', 10)
        self.origin_pub = self.create_publisher(String, 'position/origin', 10)
        
        # Subscriber for the launch/land status.
        self.status_sub = self.create_subscription(
            String,
            'launch_land_status',
            self.updateLaunchStatus,
            10
        )
        
        # Timer to continuously publish origin coordinates (every 1 sec).
        self.origin_timer = self.create_timer(1, self.publish_origin)
        
        # Initialize current and origin coordinates.
        self.x_ = 0.0
        self.y_ = 0.0
        self.z_ = 0.0
        
        self.launch_status = None  # Current status string
        
        self.x_origin = None
        self.y_origin = None
        self.z_origin = None

        # List to store recent non-zero coordinates (maximum length of 10).
        self.recent_coords = []
        
    def updateLaunchStatus(self, msg: String):
        # Acceptable statuses.
        valid_statuses = ['landed', 'launched', 'landing', 'launching']
        new_status = msg.data.strip().lower()
        
        if new_status not in valid_statuses:
            self.get_logger().warn(f"Received invalid launch/land status: {new_status}")
            return
        
        # Check for the transition from "launching" to "launched".
        if self.launch_status == "launching" and new_status == "launched":
            # Set the origin only if not set and current coordinates are not (0,0,0).
            if self.x_origin is None and (self.x_ != 0.0 or self.y_ != 0.0 or self.z_ != 0.0):
                self.x_origin = self.x_
                self.y_origin = self.y_
                self.z_origin = self.z_
                origin_position_str = f"x={self.x_origin} y={self.y_origin} z={self.z_origin}"
                self.origin_pub.publish(String(data=origin_position_str))
                self.get_logger().info(f"Origin set on launch transition: {origin_position_str}")
                
        self.launch_status = new_status
        self.get_logger().info(f"Updated launch/land status: {self.launch_status}")

    def updateRtabCoord(self, msg: PoseStamped):
        # Retrieve the new coordinates from the PoseStamped message.
        new_x = msg.pose.position.x
        new_y = msg.pose.position.y
        new_z = msg.pose.position.z

        # If new coordinate is (0,0,0), use the most recent non-zero coordinate if available.
        if new_x == 0.0 and new_y == 0.0 and new_z == 0.0:
            for coord in reversed(self.recent_coords):
                if not (coord[0] == 0.0 and coord[1] == 0.0 and coord[2] == 0.0):
                    new_x, new_y, new_z = coord
                    self.get_logger().debug("Using last valid coordinate due to (0,0,0) reading.")
                    break
            # If no valid coordinate is found, new_x, new_y, new_z remain (0,0,0).
        else:
            # Store the new valid (non-zero) coordinate.
            self.recent_coords.append((new_x, new_y, new_z))
            # Keep only the latest 10 entries.
            if len(self.recent_coords) > 10:
                self.recent_coords.pop(0)

        # Update the internal state with the chosen coordinates.
        self.x_, self.y_, self.z_ = new_x, new_y, new_z

        # Publish only if coordinates are not (0,0,0).
        if self.x_ != 0.0 or self.y_ != 0.0 or self.z_ != 0.0:
            current_position_str = f"x={self.x_} y={self.y_} z={self.z_}"
            self.current_pub.publish(String(data=current_position_str))
            self.get_logger().info(f"Published coordinates: {current_position_str}")
        
        # Set the origin if not set, status is "launched", and coordinates are not (0,0,0).
        if self.x_origin is None and self.launch_status == "launched" and (self.x_ != 0.0 or self.y_ != 0.0 or self.z_ != 0.0):
            self.x_origin = self.x_
            self.y_origin = self.y_
            self.z_origin = self.z_
            origin_position_str = f"x={self.x_origin} y={self.y_origin} z={self.z_origin}"
            self.origin_pub.publish(String(data=origin_position_str))
            self.get_logger().info(f"Origin set on receiving pose: {origin_position_str}")

    def publish_origin(self):
        # Publish the origin coordinates every 1 second if they have been set.
        if self.x_origin is not None:
            origin_position_str = f"x={self.x_origin} y={self.y_origin} z={self.z_origin}"
            self.origin_pub.publish(String(data=origin_position_str))
            self.get_logger().debug(f"Published origin: {origin_position_str}")

def main(args=None):
    rclpy.init(args=args)
    coordinate_publisher = CoordinatePublisher()
    rclpy.spin(coordinate_publisher)
    coordinate_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from std_msgs.msg import String

# class CoordinatePublisher(Node):
#     def __init__(self):
#         super().__init__('Coordinate_Pub')
        
#         # Subscribe to the /rtabmap/odom topic using Odometry messages.
#         self.rtab_sub = self.create_subscription(
#             Odometry, 
#             '/rtabmap/odom', 
#             self.updateRtabCoord, 
#             10
#         )
        
#         # Publishers for current position and origin.
#         self.current_pub = self.create_publisher(String, 'position/current', 10)
#         self.origin_pub = self.create_publisher(String, 'position/origin', 10)
        
#         # Subscriber for the launch/land status.
#         self.status_sub = self.create_subscription(
#             String,
#             'launch_land_status',
#             self.updateLaunchStatus,
#             10
#         )
        
#         # Timer to continuously publish origin coordinates (every 1 sec).
#         self.origin_timer = self.create_timer(1, self.publish_origin)
        
#         # Initialize current and origin coordinates.
#         self.x_ = 0.0
#         self.y_ = 0.0
#         self.z_ = 0.0
        
#         self.launch_status = None  # Current status string
        
#         self.x_origin = None
#         self.y_origin = None
#         self.z_origin = None

#         # List to store recent non-zero coordinates (maximum length of 10).
#         self.recent_coords = []
        
#     def updateLaunchStatus(self, msg: String):
#         # Acceptable statuses.
#         valid_statuses = ['landed', 'launched', 'landing', 'launching']
#         new_status = msg.data.strip().lower()
        
#         if new_status not in valid_statuses:
#             self.get_logger().warn(f"Received invalid launch/land status: {new_status}")
#             return
        
#         # Check for the transition from "launching" to "launched".
#         if self.launch_status == "launching" and new_status == "launched":
#             # Set the origin only if not set and current coordinates are not (0,0,0).
#             if self.x_origin is None and (self.x_ != 0.0 or self.y_ != 0.0 or self.z_ != 0.0):
#                 self.x_origin = self.x_
#                 self.y_origin = self.y_
#                 self.z_origin = self.z_
#                 origin_position_str = f"x={self.x_origin} y={self.y_origin} z={self.z_origin}"
#                 self.origin_pub.publish(String(data=origin_position_str))
#                 self.get_logger().info(f"Origin set on launch transition: {origin_position_str}")
                
#         self.launch_status = new_status
#         self.get_logger().info(f"Updated launch/land status: {self.launch_status}")

#     def updateRtabCoord(self, msg: Odometry):
#         # Retrieve the new coordinates from the odometry message.
#         new_x = msg.pose.pose.position.x
#         new_y = msg.pose.pose.position.y
#         new_z = msg.pose.pose.position.z

#         # If new coordinate is (0,0,0), use the most recent non-zero coordinate if available.
#         if new_x == 0.0 and new_y == 0.0 and new_z == 0.0:
#             for coord in reversed(self.recent_coords):
#                 if not (coord[0] == 0.0 and coord[1] == 0.0 and coord[2] == 0.0):
#                     new_x, new_y, new_z = coord
#                     self.get_logger().debug("Using last valid coordinate due to (0,0,0) reading.")
#                     break
#             # If no valid coordinate is found, new_x, new_y, new_z remain (0,0,0).
#         else:
#             # Store the new valid (non-zero) coordinate.
#             self.recent_coords.append((new_x, new_y, new_z))
#             # Keep only the latest 10 entries.
#             if len(self.recent_coords) > 10:
#                 self.recent_coords.pop(0)

#         # Update the internal state with the chosen coordinates.
#         self.x_, self.y_, self.z_ = new_x, new_y, new_z

#         # Publish only if coordinates are not (0,0,0).
#         if self.x_ != 0.0 or self.y_ != 0.0 or self.z_ != 0.0:
#             current_position_str = f"x={self.x_} y={self.y_} z={self.z_}"
#             self.current_pub.publish(String(data=current_position_str))
#             self.get_logger().info(f"Published Rtab coordinates: {current_position_str}")
        
#         # Set the origin if not set, status is "launched", and coordinates are not (0,0,0).
#         if self.x_origin is None and self.launch_status == "launched" and (self.x_ != 0.0 or self.y_ != 0.0 or self.z_ != 0.0):
#             self.x_origin = self.x_
#             self.y_origin = self.y_
#             self.z_origin = self.z_
#             origin_position_str = f"x={self.x_origin} y={self.y_origin} z={self.z_origin}"
#             self.origin_pub.publish(String(data=origin_position_str))
#             self.get_logger().info(f"Origin set on receiving odometry: {origin_position_str}")

#     def publish_origin(self):
#         # Publish the origin coordinates every 1 second if they have been set.
#         if self.x_origin is not None:
#             origin_position_str = f"x={self.x_origin} y={self.y_origin} z={self.z_origin}"
#             self.origin_pub.publish(String(data=origin_position_str))
#             self.get_logger().debug(f"Published origin: {origin_position_str}")

# def main(args=None):
#     rclpy.init(args=args)
#     coordinate_publisher = CoordinatePublisher()
#     rclpy.spin(coordinate_publisher)
#     coordinate_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
