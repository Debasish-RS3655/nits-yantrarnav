#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

from flask import Flask, jsonify, request
from flask_cors import CORS
import threading
import base64
import cv2
from cv_bridge import CvBridge, CvBridgeError
from urllib.parse import urljoin
import numpy as np

# Global variables to hold the latest images (in base64)
latest_image_base64 = None          # From the ROS camera subscriber
perpendicular_image_base64 = None   # From the new POST endpoint
# Lock to ensure thread-safe access to global image variables
lock = threading.Lock()

# ROS2 BridgeServer node: subscribes to position topics and camera images
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

        # Subscribers for position topics
        self.pos_current_sub = self.create_subscription(String, 'position/current', self.update_current_pos, 10)
        self.pos_target_sub = self.create_subscription(String, 'position/target', self.update_target_pos, 10)
        self.pos_phase_sub = self.create_subscription(String, 'position/phase', self.update_phase_pos, 10)
        self.mode_pub = self.create_publisher(String, 'position/mode', 10)

        # Subscriber for the camera image topic
        self.camera_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.camera_callback, 10)

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()

    def update_current_pos(self, msg: String):
        # Expected format: "x=<value> y=<value> z=<value>"
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
        except Exception as e:
            self.get_logger().error('Failed to parse current position: ' + str(e))

    def update_target_pos(self, msg: String):
        # Expected format: "x=<value> y=<value> z=<value>"
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
        except Exception as e:
            self.get_logger().error('Failed to parse target position: ' + str(e))

    def update_phase_pos(self, msg: String):
        try:
            self.phase = int(msg.data)
            self.get_logger().info('Updated Bridge phase: ' + str(self.phase))
        except Exception as e:
            self.get_logger().error('Failed to parse Bridge phase: ' + str(e))

    def camera_callback(self, msg: Image):
        global latest_image_base64
        try:
            # Convert ROS Image to OpenCV image (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Resize the image to 256x256 (as needed)
            cv_image = cv2.resize(cv_image, (256, 256))
            # Encode the image as JPEG
            ret, buffer = cv2.imencode('.jpg', cv_image)
            if ret:
                jpg_as_text = base64.b64encode(buffer).decode('utf-8')
                # Safely update the global variable for latest image
                with lock:
                    latest_image_base64 = jpg_as_text
        except CvBridgeError as e:
            self.get_logger().error('CvBridge Error: ' + str(e))

# Set up the Flask app and endpoints
app = Flask(__name__)
CORS(app)
bridge_server_node = None         # Global variable for the ROS node
shutdown_flag = threading.Event() # Signal to stop threads

# Endpoint: Return current position (from ROS node)
@app.route('/current_position', methods=['GET'])
def current_position():
    global bridge_server_node
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500
    pos = {
        'x': bridge_server_node.x_,
        'y': bridge_server_node.y_,
        'z': bridge_server_node.z_
    }
    return jsonify(pos)

# Endpoint: Return target position (from ROS node)
@app.route('/target_position', methods=['GET'])
def target_position():
    global bridge_server_node
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500
    pos = {
        'x': bridge_server_node.target_x,
        'y': bridge_server_node.target_y,
        'z': bridge_server_node.target_z
    }
    return jsonify(pos)

# Endpoint: Set mode (automatic, manual, or hover)
@app.route('/mode', methods=['POST'])
def set_mode():
    global bridge_server_node
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500

    data = request.get_json()
    if not data or 'mode' not in data:
        return jsonify({'error': 'Please provide a mode (automatic, manual, or hover)'}), 400

    mode = data['mode']
    if mode not in ['automatic', 'manual', 'hover']:
        return jsonify({'error': 'Mode must be either "automatic", "manual", or "hover"'}), 400

    msg = String()
    msg.data = mode
    bridge_server_node.mode_pub.publish(msg)
    bridge_server_node.get_logger().info(f'Published mode from Bridge server: {mode}')
    return jsonify({'mode': mode, 'status': 'published'}), 200

# Endpoint: Return the latest camera image (from ROS) in base64
@app.route('/latest_image', methods=['GET'])
def get_latest_image():
    global latest_image_base64
    with lock:
        if latest_image_base64 is not None:
            return jsonify({'image': latest_image_base64})
        else:
            return jsonify({'error': 'No image received yet'}), 404

# New Endpoint: Receive a perpendicular camera image via POST (without saving)
@app.route('/perpendicular_cam', methods=['POST'])
def perpendicular_cam():
    global perpendicular_image_base64
    data = request.get_json(force=True)
    if 'image' not in data:
        return jsonify({'error': 'No image provided'}), 400

    image_b64 = data['image']

    # Validate that the base64 string decodes to an image (without saving)
    try:
        im_bytes = base64.b64decode(image_b64)
        np_arr = np.frombuffer(im_bytes, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError("Decoded image is None")
    except Exception as e:
        print("Error decoding image:", e)
        return jsonify({'error': 'Invalid image data'}), 400

    with lock:
        perpendicular_image_base64 = image_b64

    print("Perpendicular camera image received and stored in memory")
    return jsonify({'status': 'success'}), 200

# Function to spin the ROS node in a separate thread
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

    # Start ROS 2 spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin, args=(node,))
    ros_thread.start()

    try:
        # Start the Flask server (disable auto reloader to avoid lingering processes)
        app.run(host='0.0.0.0', port=5000, use_reloader=False)
    except KeyboardInterrupt:
        print('Keyboard interrupt caught. Bridge server shutting down gracefully.')
    finally:
        shutdown_flag.set()
        ros_thread.join(timeout=5)  # wait for ROS thread to finish
        rclpy.shutdown()

if __name__ == '__main__':
    main()
