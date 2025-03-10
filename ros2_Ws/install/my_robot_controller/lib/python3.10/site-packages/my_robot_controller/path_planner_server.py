#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, jsonify
from flask_cors import CORS
import threading

class PathPlanner(Node):
    
    def __init__(self):
        super().__init__('path_planner_server')
        self.pos_target_pub = self.create_publisher(String, 'position/target', 10)                
        
        # Initialize current position variables (these could be updated dynamically)
        self.x_ = 0
        self.y_ = 0
        self.z_ = 0
                
        # Target positions (for demonstration, we are using fixed values)
        self.target_x = 50
        self.target_y = 50
        self.target_z = 50
        
        self.pos_current_sub = self.create_subscription(String, 'position/current', self.update_pos, 10)                                
                
        self.timer_ = self.create_timer(1, self.send_pos)
        self.get_logger().info('Path planner node has been started.')


    # update the current position from the position publisher node
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

    # Publish the current target position
    def send_pos(self):
        msg = String()
        msg.data = f'x={self.target_x} y={self.target_y} z={self.target_z}'
        self.pos_target_pub.publish(msg)
        self.get_logger().info(f'Target position published: {msg.data}')

# Global variable to hold the node instance
path_planner_node = None

# Create Flask app
app = Flask(__name__)
CORS(app)
@app.route('/current', methods=['GET'])
def current_position():
    """
    Flask endpoint to extract the current target position.
    Returns a JSON with the x, y, and z values.
    """
    global path_planner_node
    if path_planner_node is None:
        return jsonify({'error': 'PathPlanner node not available'}), 500

    # Here, we choose to return the target positions.
    pos = {
        'x': path_planner_node.x_,
        'y': path_planner_node.y_,
        'z': path_planner_node.z_
    }
    return jsonify(pos)


@app.route('/target', methods=['GET'])
def target_position():
    """
    Flask endpoint to extract the current target position.
    Returns a JSON with the x, y, and z values.
    """
    global path_planner_node
    if path_planner_node is None:
        return jsonify({'error': 'PathPlanner node not available'}), 500

    # Here, we choose to return the target positions.
    pos = {
        'x': path_planner_node.target_x,
        'y': path_planner_node.target_y,
        'z': path_planner_node.target_z

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

    # Start the Flask server (accessible on port 5000)
    app.run(host='0.0.0.0', port=5000)

    node.destroy_node()  # Clean up after Flask server stops (if ever)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
