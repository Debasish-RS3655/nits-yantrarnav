#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, jsonify
from flask_cors import CORS
import threading
import time


# bridge server that subscribes to all topics and responds the current topic value over HTTP

class BridgeServer(Node):
    def __init__(self):    
        super().__init__('bridge_server_node')
        
        self.x_ = 0
        self.y_ = 0
        self.z_ = 0        
        
        self.target_x = 0
        self.target_y = 0
        self.target_z = 0
        
        self.phase = 0
        
        # subscriber list
        self.pos_current_sub = self.create_subscription(String, 'position/current', self.update_current_pos, 10)
        self.pos_target_sub = self.create_subscription(String, 'position/target', self.update_target_pos, 10)
        self.pos_phase_sub = self.create_subscription(String, 'position/phase', self.update_phase_pos, 10)                
        
    def update_current_pos(self, msg: String):
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
            # self.get_logger().info(f'Updated position: x={self.x_}, y={self.y_}, z={self.z_}')
        except Exception as e:
            self.get_logger().error('Failed to parse current position: ' + str(e))
            
    def update_target_pos(self, msg: String):
        # Expect message format: "x=<value> y=<value> z=<value>"
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
            # self.get_logger().info(f'Updated position: x={self.x_}, y={self.y_}, z={self.z_}')
        except Exception as e:
            self.get_logger().error('Failed to parse current position: ' + str(e))

    def update_phase_pos(self, msg:String):        
        try:            
            self.phase = int(msg.data)
            self.get_logger().error('Updated Bridge phase: ' + str(self.phase))
        except Exception as e:
            self.get_logger().error('Failed to parse Bridge phase: ' + str(e))
                
        
app = Flask(__name__)
CORS(app)
bridge_server_node = None
shutdown_flag = threading.Event() # signal to stop threads

# api endpoints
@app.route('/current', methods=['GET'])
def current_position():
    """
    Returns the current position (x, y, z) of the robot.
    """
    global bridge_server_node
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500
    pos = {
        'x': bridge_server_node.x_,
        'y': bridge_server_node.y_,
        'z': bridge_server_node.z_
    }
    return jsonify(pos)

@app.route('/target', methods=['GET'])
def target_position():
    """
    Returns the current target position (x, y, z) being published.
    """
    global bridge_server_node
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500
    if bridge_server_node.target_x is None or bridge_server_node.target_y is None or bridge_server_node.target_z is None:
        return jsonify({'error': 'No current target available'}), 500
    pos = {
        'x': bridge_server_node.target_x,
        'y': bridge_server_node.target_y,
        'z': bridge_server_node.target_z
    }
    return jsonify(pos)


def ros_spin(node):
    try:
        while not shutdown_flag.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()


def main(args=None):
    global bridge_server_node
    rclpy.init(args=args)
    node = BridgeServer()
    bridge_server_node = node
    
    # start ROS 2 spinning in a non-daemon thread
    ros_thread = threading.Thread(target=ros_spin, args=(node,))
    # ros_thread.daemon = True
    ros_thread.start()    
    
    try:        
        # start the Flask server by disabling the auto reloader to avoid a child process lingering
        app.run(host='0.0.0.0', port=5000, use_reloader=False)    
    except KeyboardInterrupt: 
        print('Keyboard interrupt caught. Bridge server shutting down gracefully.')
    finally: 
        # signal shutdown to ROS thread
        shutdown_flag.set()
        ros_thread.join(timeout=5) # wait for ros thread to finish
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()