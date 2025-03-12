#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import random
from flask import Flask, jsonify
from flask_cors import CORS
import threading

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner_server')
        # Publisher for target position and subscriber for current position
        self.pos_target_pub = self.create_publisher(String, 'position/target', 10)
        self.pos_current_sub = self.create_subscription(String, 'position/current', self.update_pos, 10)
        # subscriber for the flat area scanner
        self.flat_area_sub = self.create_subscription(String, 'position/flat_areas', self.update_flat_areas, 10)                
        self.flat_areass = []

        # Current position
        self.x_ = 0.0
        self.y_ = 0.0
        self.z_ = 0.0

        # Record the starting origin
        self.origin_x = self.x_
        self.origin_y = self.y_
        self.origin_z = self.z_

        # Parameters for target generation
        self.side = 50.0      # Side length for square and lawnmower area
        self.lawn_gap = 2.0   # Gap between lines in lawnmower pattern
        self.threshold = 0.5  # Distance threshold to switch to the next target


        # we create a publisher for the phase
        # State machine phases:
        # Phase 1: Square path
        # Phase 2: Lawnmower pattern inside the square
        # Phase 3: Random point inside the square
        # Phase 4: Return to origin        
        
        self.phase = 1

        self.phase_pub = self.create_publisher(String, 'position/phase', 10)
        
        self.target_list = []
        self.target_index = 0
        self.current_target = None
        
        self.flat_areas = []  # List to store multiple (x, y, z) flat area coordinates
        self.fallback_timer_ = None

        # Setup first phase
        self.setup_phase(self.phase)

        # Create a timer that calls send_pos at regular intervals (every second)
        self.timer_ = self.create_timer(1.0, self.send_pos)
        self.get_logger().info('Path planner node with Flask server has been started.')

    def update_pos(self, msg: String):
        # Expect message format: "x=<value> y=<value> z=<value>"
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
        try:
            # Expecting a message format: "x1=y1=z1, x2=y2=z2, ..."
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
        """Set up the target list for the given phase."""
        if phase == 1:
            # this will be replaced with the line following code
            self.get_logger().info("Setting up Phase 1: Square Path")
            x0, y0, z0 = self.origin_x, self.origin_y, self.origin_z
            # Define square vertices (clockwise)
            self.target_list = [
                (x0 + self.side, y0, z0),
                (x0 + self.side, y0 + self.side, z0),
                (x0, y0 + self.side, z0),
                (x0, y0, z0)
            ]
            
        elif phase == 2:
            self.get_logger().info("Setting up Phase 2: Lawnmower Pattern")
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
            # self.get_logger().info("Setting up Phase 3: Flat area determination")
            # x0, y0, z0 = self.origin_x, self.origin_y, self.origin_z
            # rx = random.uniform(x0, x0 + self.side)
            # ry = random.uniform(y0, y0 + self.side)
            # self.target_list = [(rx, ry, z0)]            
            
            self.get_logger().info("Setting up Phase 3: Flat area lander - obtaining target from flat area publisher")        
            if self.flat_areas:  # If we have flat areas, choose one
                self.get_logger().info(f"Using available flat areas: {self.flat_areas}")
                self.target_list = self.flat_areas  # Use all flat areas as targets
            else:
                self.get_logger().warning("No flat areas available. Waiting 5 seconds before fallback to origin.")
                self.fallback_timer_ = self.create_timer(5.0, self.fallback_to_origin_once)
                return  # Exit early; timer callback will handle the fallback
            
        elif phase == 4:
            self.get_logger().info("Setting up Phase 4: Return to Origin")
            self.target_list = [(self.origin_x, self.origin_y, self.origin_z)]
            
        else:
            self.get_logger().error("Unknown phase specified.")
        
        self.target_index = 0
        if self.target_list:
            self.current_target = self.target_list[self.target_index]
        else:
            self.get_logger().error("Target list is empty!")

    def send_pos(self):
        """Publish the current target position. When current position overlaps target within a threshold,
        update to the next target or phase."""
        if self.current_target is None:
            return

        # Compute Euclidean distance to current target
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
                    
                    # publish the new phase when it changes
                    self.phase_pub.publish(phase_msg)                    
                    self.setup_phase(self.phase)
                    
                else:
                    self.get_logger().info('Completed all phases.')
                    self.timer_.cancel()
                    return
            else:
                self.current_target = self.target_list[self.target_index]
                self.get_logger().info(f'Switching to next target: {self.current_target}')

        # Publish current target position
        msg = String()
        msg.data = f'x={self.current_target[0]} y={self.current_target[1]} z={self.current_target[2]}'
        self.pos_target_pub.publish(msg)
        self.get_logger().info(f'Target published: {msg.data}')
        
    # after 5 seconds of waiting if there are still no flat areas the drone falls back to the origin        
    def fallback_to_origin_once(self):
        if self.fallback_timer_:
            self.fallback_timer_.cancel()
            self.fallback_timer_ = None

        if not self.flat_areas:  # Still no flat areas
            self.get_logger().warning("No flat areas received after waiting. Falling back to origin.")
            self.target_list = [(self.origin_x, self.origin_y, self.origin_z)]
        else:
            self.get_logger().info(f"Flat areas received during wait: {self.flat_areas}. Using them.")
            self.target_list = self.flat_areas  # Use the newly received flat areas

        self.target_index = 0
        if self.target_list:
            self.current_target = self.target_list[0]
    

# Global variable to hold the PathPlanner node instance (used by Flask endpoints)
path_planner_node = None

# Create Flask app and enable CORS
app = Flask(__name__)
CORS(app)

@app.route('/current', methods=['GET'])
def current_position():
    """
    Returns the current position (x, y, z) of the robot.
    """
    global path_planner_node
    if path_planner_node is None:
        return jsonify({'error': 'PathPlanner node not available'}), 500
    pos = {
        'x': path_planner_node.x_,
        'y': path_planner_node.y_,
        'z': path_planner_node.z_
    }
    return jsonify(pos)

@app.route('/target', methods=['GET'])
def target_position():
    """
    Returns the current target position (x, y, z) being published.
    """
    global path_planner_node
    if path_planner_node is None:
        return jsonify({'error': 'PathPlanner node not available'}), 500
    if path_planner_node.current_target is None:
        return jsonify({'error': 'No current target available'}), 500
    pos = {
        'x': path_planner_node.current_target[0],
        'y': path_planner_node.current_target[1],
        'z': path_planner_node.current_target[2]
    }
    return jsonify(pos)

def main(args=None):
    global path_planner_node
    rclpy.init(args=args)
    node = PathPlanner()
    path_planner_node = node

    # Run the ROS2 node in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,))
    ros_thread.daemon = True
    ros_thread.start()

    # Start the Flask server (listening on port 5000)
    app.run(host='0.0.0.0', port=5000)

    # Cleanup if Flask server stops (unlikely in typical use)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
