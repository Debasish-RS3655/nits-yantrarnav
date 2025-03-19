#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import random       # to determine the side length when in simulation

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner_server')
                        
        # the path planner is the publisher of the target position
        self.pos_target_pub = self.create_publisher(String, 'position/target', 10)
        
        # path planner is the subscriber of the current position
        self.pos_current_sub = self.create_subscription(String, 'position/current', self.update_pos, 10)
                
        # New subscriber to get the origin coordinate (instead of assuming 0,0,0)
        self.origin_sub = self.create_subscription(String, 'position/origin', self.update_origin, 10)
        self.origin_received = False

        # Subscriber for the flat area scanner
        self.flat_area_sub = self.create_subscription(String, 'position/flat_area', self.update_flat_areas, 10)
        self.flat_areas = []  # List to store multiple (x, y, z) flat area coordinates

        # Current position (updated continuously)
        self.x_ = 0.0
        self.y_ = 0.0
        self.z_ = 0.0

        # Origin coordinates (will be set via the 'position/origin' topic)
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_z = 0.0

        # simulation parameters
        self.simulation_sub = self.create_subscription(String, 'position/simulation', self.update_simulation, 10)
        self.simulation = False
        # Parameters for target generation
        self.side = 50.0      # Side length for square and lawnmower area
        self.lawn_gap = 2.0   # Gap between lines in lawnmower pattern
        self.threshold = 0.5  # Distance threshold to switch to the next target

        # State machine phases:
        # Phase 0: Move straight until yellow boundary reached.
        # Phase 1: Follow yellow boundary for three consecutive edges.
        # Phase 2: Lawnmower pattern inside the square.
        # Phase 3: Flat area lander (using flat areas from flat_area_sub).
        # Phase 4: Return to origin.
        
        self.phase = None  # Will be set after receiving origin.
        self.phase_pub = self.create_publisher(String, 'position/phase', 10)

        self.target_list = []
        self.target_index = 0
        self.current_target = None
        
        self.fallback_timer_ = None

        # Timer to call the send target coordinate function .. faster means smoother
        self.timer_ = self.create_timer(0.5, self.send_target_pos)
        
        self.get_logger().info('Path planner node started. Waiting for origin...')


    # !work on this update origin method
    def update_origin(self, msg: String):
        """Receive the origin from the 'position/origin' topic.
        Expected message format: "x=<value> y=<value> z=<value>".
        This is only processed once to set the starting point."""
        if self.origin_received:
            return  # Ignore subsequent origin messages.
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
            self.origin_received = True
            self.get_logger().info(f'Origin received: x={self.origin_x}, y={self.origin_y}, z={self.origin_z}')
            # Initialize the state machine starting with Phase 0
            self.phase = 0
            self.setup_phase(self.phase)
        except Exception as e:
            self.get_logger().error('Failed to parse origin: ' + str(e))
                        
    def update_simulation(self, msg: String):
        """Update the simulation status from the 'position/simulation' topic.
        Expected message format: "True" or "False"."""
        try:
            self.simulation = msg.data.strip().lower() == "true"
            self.get_logger().info(f'Simulation mode updated: {self.simulation}')
        except Exception as e:
            self.get_logger().error(f'Failed to parse simulation mode: {e}')    
            
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
        """Update the flat area list from the 'position/flat_areas' topic.
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
    
    def setup_phase(self, phase):
        """Set up the target list for the given phase based on the current origin."""
        if phase == 0:
            
            # leave this to shwetangshu's line follower
            
            # when in simulation we follow a random direction in a random length until we reach the boundary
            self.get_logger().info("Setting up Phase 0: Move straight until yellow boundary reached")            
            # we do not have any target coordinates in phase 0            
                        
            # Assume the yellow boundary is reached by moving in the positive x-direction
            self.get_logger().info("Simulating moving towards the boundary.")
            # gotta move this 
            self.target_list = [(self.origin_x + random.uniform(0, self.side), self.origin_y, self.origin_z)]
            
                
        elif phase == 1:
            self.get_logger().info("Setting up Phase 1: Follow yellow boundary for three consecutive edges")
                        
            # gotta remove this in the actual implementation
                        
            # Starting from the endpoint of Phase 0:
            x0 = self.origin_x + self.side
            y0 = self.origin_y
            z0 = self.origin_z
            self.target_list = [
                (x0, y0 + self.side, z0),         # Edge 1: move upward
                (self.origin_x, y0 + self.side, z0),# Edge 2: move left
                (self.origin_x, self.origin_y, z0)   # Edge 3: move downward (completes the square boundary)
            ]
            
            
        elif phase == 2:
            
            
            self.get_logger().info("Setting up Phase 2: Lawnmower Pattern inside the square")            
            x0, y0, z0 = self.origin_x, self.origin_y, self.origin_z
            points = []
            rows = int(self.side // self.lawn_gap) + 1
            
            for i in range(rows):
                y = y0 + i * self.lawn_gap
                if y > y0 + self.side:
                    y = y0 + self.side
                # Alternate direction for each row
                if i % 2 == 0:                    
                    points.append((x0, y, z0))
                    points.append((x0 + self.side, y, z0))
                else:
                    points.append((x0 + self.side, y, z0))
                    points.append((x0, y, z0))
            self.target_list = points
        
        
        elif phase == 3:
            self.get_logger().info("Setting up Phase 3: Flat area lander - obtaining target from flat area publisher")
            if self.flat_areas:
                self.get_logger().info(f"Using available flat areas: {self.flat_areas}")
                self.target_list = self.flat_areas
            else:
                self.get_logger().warning("No flat areas available. Waiting 5 seconds before fallback to origin.")
                self.fallback_timer_ = self.create_timer(5.0, self.fallback_to_origin_once)
                return
        
        elif phase == 4:
            self.get_logger().info("Setting up Phase 4: Return to Origin")
            # append the origin as the target to the target list now
            self.target_list = [(self.origin_x, self.origin_y, self.origin_z)]
        
        else:
            self.get_logger().error("Unknown phase specified.")
        
        self.target_index = 0
        if self.target_list:
            self.current_target = self.target_list[self.target_index]
        else:
            self.get_logger().error("Target list is empty!")
            
                        
    def send_target_pos(self):
        """Publish the current target position. When the current position overlaps the target within a threshold,
        update to the next target or phase."""
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
                if self.phase < 4:
                    self.phase += 1
                    self.get_logger().info(f'Moving to Phase {self.phase}')
                    phase_msg = String()
                    phase_msg.data = str(self.phase)
                    self.phase_pub.publish(phase_msg)
                    self.setup_phase(self.phase)
                else:
                    self.get_logger().info('Completed all phases.')
                    self.timer_.cancel()
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
        if self.fallback_timer_:
            self.fallback_timer_.cancel()
            self.fallback_timer_ = None

        if not self.flat_areas:
            self.get_logger().warning("No flat areas received after waiting. Falling back to origin.")
            self.target_list = [(self.origin_x, self.origin_y, self.origin_z)]
        else:
            self.get_logger().info(f"Flat areas received during wait: {self.flat_areas}. Using them.")
            self.target_list = self.flat_areas

        self.target_index = 0
        if self.target_list:
            self.current_target = self.target_list[0]

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
