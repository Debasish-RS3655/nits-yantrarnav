#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import math

# Main Path planner node
# Debashish Buragohain

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner_server')
                                                            
        # Subscriber for the launch land status
        self.drone_launch_land_sub = self.create_subscription(
            String, '/launch_land_status', self.update_launch_status, 10)
        self.drone_launch_status = None
        
        # Publisher for the drone commands
        self.drone_commands_pub = self.create_publisher(String, '/drone_commands', 10)
        # Subscriber for the current mode
        self.pos_mode = self.create_subscription(String, '/position/mode', self.update_mode, 10)
        # Subscriber for the current position
        self.pos_current_sub = self.create_subscription(String, 'position/current', self.update_pos, 10)            
        # Publisher for the target position
        self.pos_target_pub = self.create_publisher(String, 'position/target', 10)
        # Current position (updated continuously)
        self.x_ = 0.0
        self.y_ = 0.0
        self.z_ = 0.0
    
        # By default the mode is set to manual
        self.mode = 'manual'
        
        # For the hovering mode
        self.hovering_status_sub = self.create_subscription(String, '/hovering_mode', self.update_hovering_mode, 10)
        self.hovering_status = None
        
        # Subscriber for the edge coordinates    
        self.edge_sub = self.create_subscription(String, '/edge_coordinates', self.update_edges, 10)            
        self.edge_coordinates = []      # Expect four edges: [(x1,y1,z1), (x2,y2,z2), ...]
                
        # Subscriber for the origin coordinate (set via the 'position/origin' topic)
        self.origin_sub = self.create_subscription(String, 'position/origin', self.update_origin, 10)
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_z = 0.0

        # Yellow boundary subscriber
        self.yellow_boundary_found = self.create_subscription(Bool, '/yellow_boundary_found', self.update_yellow_boundary_found, 10)
        self.yellow_boundary_found_status = False

        # Subscriber for the flat area scanner
        self.flat_area_sub = self.create_subscription(String, 'position/flat_area', self.update_flat_areas, 10)
        self.flat_areas = []  # List to store multiple (x, y, z) flat area coordinates

        # Parameters for target generation
        # For a rectangular area, the side lengths will be computed from edge coordinates.
        self.long_side = None    # Initially None until computed
        self.short_side = None   # Initially None until computed
        self.lawn_gap = 2.0      # Gap between lines in lawnmower pattern
        self.threshold = 0.5     # Distance threshold to switch to the next target

        # State machine phases:
        # Phase 0: Move straight until yellow boundary reached.
        # Phase 1: Waiting till Line Follower Node has published the edge coordinates.
        # Phase 2: Lawnmower pattern inside the rectangle.
        # Phase 3: Flat area lander (using flat areas from flat_area_sub).
        # Phase 4: Return to origin.
        # Phase 5: Landing phase.
        self.phase = None  # Will be set after receiving origin.
        self.phase_pub = self.create_publisher(String, 'position/phase', 10)

        self.target_list = []
        self.target_index = 0
        self.current_target = None
        
        self.fallback_to_origin_timer = None
        # Main loop timer (runs every 0.5 sec)
        self.main_timer_ = self.create_timer(0.5, self.main_loop)                         
        self.get_logger().info('Path planner node started.')
        
        # Launch the drone (pre-checks and command publishing)
        self.launch_drone()
                
    # Publishes launch command to the drone commands topic
    def launch_drone(self):
        msg = String()
        msg.data = 'launch'
        self.drone_commands_pub.publish(msg)        
        self.get_logger().info('Launch command published to drone commands.')
        
    def land_drone(self):
        msg = String()
        msg.data = 'land'
        self.drone_commands_pub.publish(msg)        
        self.get_logger().info('Land command published to drone commands.')

    # Update the launch status of the drone
    def update_launch_status(self, msg: String):
        new_launch_status = msg.data
        if new_launch_status not in ['launched', 'landed', 'launching', 'landing']:
            self.get_logger().info("Invalid launch status received: " + str(new_launch_status))
            return
        self.drone_launch_status = new_launch_status        
        self.get_logger().info(f"Launch status updated to: {self.drone_launch_status}")        
    
    # Update the hovering status of the drone
    def update_hovering_mode(self, msg: String):
        new_hovering_mode = msg.data
        if new_hovering_mode not in ['ongoing', 'completed']:
            self.get_logger().info("Invalid hovering status received: " + str(new_hovering_mode))
            return
        self.hovering_status = new_hovering_mode        
        self.get_logger().info(f"Hovering status updated to: {self.hovering_status}")        

    # Update the current mode of the drone
    def update_mode(self, msg: String):
        new_mode = msg.data
        if new_mode not in ['manual', 'auto', 'hover']:
            self.get_logger().info("Invalid mode received: " + str(new_mode))
            return
        self.mode = new_mode
        self.get_logger().info(f"Mode updated to: {self.mode}")

    def update_yellow_boundary_found(self, msg: Bool):
        if not self.yellow_boundary_found_status and msg.data:
            self.yellow_boundary_found_status = True
        
    def update_pos(self, msg: String):
        """Update the current position from the 'position/current' topic.
        Expected format: "x=<value> y=<value> z=<value>"."""
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
            self.get_logger().info(f'Updated position: x={self.x_}, y={self.y_}, z={self.z_}')
        except Exception as e:
            self.get_logger().error('Failed to parse current position: ' + str(e))
            
    def update_flat_areas(self, msg: String):
        """Update the flat area list from the 'position/flat_area' topic.
        Expected format: "x1=y1=z1, x2=y2=z2, ..."."""
        try:
            flat_areas = []
            entries = msg.data.split(",")
            for entry in entries:
                parts = entry.strip().split("=")
                if len(parts) == 3:
                    x, y, z = map(float, parts)
                    flat_areas.append((x, y, z))
    
            if flat_areas:
                self.flat_areas = flat_areas
                self.get_logger().info(f"Updated flat areas: {self.flat_areas}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse flat areas: {e}")
    
    def configure_phase(self, phase):
        """Set up the target list for the given phase based on the current origin.
        Also publishes the new target coordinate only once.
        Only operates when mode is automatic.
        Additionally, when phase > 2 (i.e. phase 3 or 4), the target chosen is the one nearest to the current coordinate."""
                
        if self.mode != 'auto':
            self.get_logger().info(f"Mode is set to {self.mode}. Path planner not executed.")
            if self.mode == 'hover':
                if self.hovering_status == 'ongoing':
                    self.get_logger().info('Drone is hovering.')
                elif self.hovering_status == 'completed':
                    self.phase = 5  # Ready to land.
                else:
                    self.get_logger().info("Invalid hovering status: " + str(self.hovering_status))
            elif self.mode == 'manual':
                self.get_logger().info('Drone is in manual mode.')
            return            
        
        if phase == 0:
            self.get_logger().info("Phase 0: Moving until yellow boundary is detected.")            
            if self.yellow_boundary_found_status:
                self.get_logger().info("Yellow boundary found. Moving to the next phase: Line following phase.")
                self.next_phase()     
            return            
                        
        elif phase == 1:
            if not self.yellow_boundary_found_status:
                self.get_logger().info("Critical error: Phase 1 reached without finding yellow boundary.")
                return
            self.get_logger().info("Waiting for edge coordinates from Line Follower Node...")
            if len(self.edge_coordinates) == 4:
                self.get_logger().info("Received edge coordinates. Moving to the next phase.")
                self.next_phase()                
            return
                        
        elif phase == 2:
            self.get_logger().info("Phase 2: Configuring Lawnmower Pattern inside the rectangle.")
            if self.long_side is None or self.short_side is None:
                self.get_logger().warn("Side lengths not yet computed. Cannot configure lawnmower pattern.")
                return
            x0, y0, z0 = self.origin_x, self.origin_y, self.origin_z
            points = []
            rows = int(self.short_side // self.lawn_gap) + 1
            for i in range(rows):
                y = y0 + i * self.lawn_gap
                if y > y0 + self.short_side:
                    y = y0 + self.short_side
                if i % 2 == 0:
                    points.append((x0, y, z0))
                    points.append((x0 + self.long_side, y, z0))
                else:
                    points.append((x0 + self.long_side, y, z0))
                    points.append((x0, y, z0))
            self.target_list = points
        
        elif phase == 3:
            self.get_logger().info("Phase 3: Configuring Flat Area Lander.")
            if self.flat_areas:
                self.get_logger().info(f"Using available flat areas: {self.flat_areas}")
                self.target_list = self.flat_areas
            else:
                self.get_logger().warning("No flat areas available. Waiting 5 seconds before fallback to origin.")
                self.fallback_to_origin_timer = self.create_timer(5.0, self.fallback_to_origin_once)
                return
        
        elif phase == 4:
            self.get_logger().info("Phase 4: Configuring Return to Origin.")
            self.target_list = [(self.origin_x, self.origin_y, self.origin_z)]
        
        else:
            self.get_logger().error("Unknown phase specified.")
        
        # For phases greater than 2, choose the target from the list that is nearest to the current coordinate.
        if self.phase > 2 and self.target_list:
            def distance(target):
                dx = self.x_ - target[0]
                dy = self.y_ - target[1]
                dz = self.z_ - target[2]
                return math.sqrt(dx**2 + dy**2 + dz**2)
            nearest_idx, nearest_target = min(enumerate(self.target_list), key=lambda t: distance(t[1]))
            self.target_index = nearest_idx
            self.current_target = nearest_target
        else:
            self.target_index = 0
            if self.target_list:
                self.current_target = self.target_list[0]
        
        if self.current_target:
            # Publish the new target only once when the phase is configured.
            msg = String()
            msg.data = f'x={self.current_target[0]} y={self.current_target[1]} z={self.current_target[2]}'
            self.pos_target_pub.publish(msg)
            self.get_logger().info(f'Target published: {msg.data}')
        else:
            self.get_logger().error("Target list is empty!")
            
    def next_phase(self):
        self.phase += 1
        self.get_logger().info(f'Moving to Phase {self.phase}')
        phase_msg = String()
        phase_msg.data = str(self.phase)
        self.phase_pub.publish(phase_msg)        
        
    def main_loop(self):
        """Main loop to check if the current target is reached.
        New targets are published only when the current target is reached.
        Runs only in automatic mode."""
        if self.mode != 'auto':
            self.get_logger().info(f"Mode is set to {self.mode}. Automatic path planning is paused.")
            return

        if self.current_target is None:
            return

        dx = self.x_ - self.current_target[0]
        dy = self.y_ - self.current_target[1]
        dz = self.z_ - self.current_target[2]
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        if distance < self.threshold:
            self.get_logger().info(f'Target reached: {self.current_target}')
            self.target_index += 1
            if self.target_index >= len(self.target_list):
                if self.phase < 5:
                    self.next_phase()
                    self.configure_phase(self.phase)
                else:
                    self.get_logger().info('Completed all phases.')
                    self.land_drone()
                    self.main_timer_.cancel()                
                    return
            else:
                self.current_target = self.target_list[self.target_index]
                self.get_logger().info(f'Switching to next target: {self.current_target}')
                msg = String()
                msg.data = f'x={self.current_target[0]} y={self.current_target[1]} z={self.current_target[2]}'
                self.pos_target_pub.publish(msg)
                self.get_logger().info(f'Target published: {msg.data}')
                                                            
    def fallback_to_origin_once(self):
        """Fallback method if no flat areas are received after a waiting period."""
        if self.fallback_to_origin_timer:
            self.fallback_to_origin_timer.cancel()
            self.fallback_to_origin_timer = None

        if not self.flat_areas:
            self.get_logger().warning("No flat areas received after waiting. Falling back to origin.")
            self.target_list = [(self.origin_x, self.origin_y, self.origin_z)]
        else:
            self.get_logger().info(f"Flat areas received during wait: {self.flat_areas}. Using them.")
            self.target_list = self.flat_areas

        self.target_index = 0
        if self.target_list:
            self.current_target = self.target_list[0]
            msg = String()
            msg.data = f'x={self.current_target[0]} y={self.current_target[1]} z={self.current_target[2]}'
            self.pos_target_pub.publish(msg)
            self.get_logger().info(f'Fallback target published: {msg.data}')

    def update_edges(self, msg: String):
        """Wait for four edge coordinates from the '/edge_coordinates' topic.
        Expected format for each coordinate: "x=<value> y=<value> z=<value>", separated by commas."""
        try:
            entries = msg.data.split(',')
            new_edges = []
            for entry in entries:
                entry = entry.strip()
                parts = entry.split()
                x = y = z = None
                for part in parts:
                    key, value = part.split('=')
                    if key == 'x':
                        x = float(value)
                    elif key == 'y':
                        y = float(value)
                    elif key == 'z':
                        z = float(value)
                if x is not None and y is not None and z is not None:
                    new_edges.append((x, y, z))
            if len(new_edges) == 4:
                self.edge_coordinates = new_edges
                self.get_logger().info(f"Edge coordinates updated: {self.edge_coordinates}")
                # Calculate the rectangle dimensions based on the new edge coordinates.
                self.calculate_rectangle_dimensions()
            else:
                self.get_logger().warn(f"Expected 4 edge coordinates, received {len(new_edges)}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse edge coordinates: {e}")
            
    def calculate_rectangle_dimensions(self):
        """Calculate the rectangle dimensions based on the four edge coordinates.
        Assume edges[0] and edges[2] form one pair,
        and edges[1] and edges[3] form the other."""
        if len(self.edge_coordinates) != 4:
            self.get_logger().warn("Insufficient edge coordinates to calculate dimensions.")
            return
        
        def dist(p1, p2):
            return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
        
        side1 = dist(self.edge_coordinates[0], self.edge_coordinates[2])
        side2 = dist(self.edge_coordinates[1], self.edge_coordinates[3])
        self.long_side = max(side1, side2)
        self.short_side = min(side1, side2)
        self.get_logger().info(f"Calculated rectangle dimensions: long_side={self.long_side}, short_side={self.short_side}")
            
    def update_origin(self, msg: String):
        """Update the origin coordinates from the 'position/origin' topic.
        Expected format: "x=<value> y=<value> z=<value>"."""
        try:
            parts = msg.data.split()
            for part in parts:
                key, value = part.split('=')
                value = float(value)
                if key == 'x':
                    self.origin_x = value
                elif key == 'y':
                    self.origin_y = value
                elif key == 'z':
                    self.origin_z = value
            self.get_logger().info(f'Origin updated: x={self.origin_x}, y={self.origin_y}, z={self.origin_z}')
        except Exception as e:
            self.get_logger().error('Failed to parse origin position: ' + str(e))
    
def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
