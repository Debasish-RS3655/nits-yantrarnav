#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import Point

# NEW: Import QoS-related classes
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS
import threading
import base64
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math

# NEW: Import the OpticalFlow message type from mavros_msgs
from mavros_msgs.msg import OpticalFlow

# Global variables to hold the latest images (in base64)
latest_image_base64 = None          # From the ROS camera subscriber
perpendicular_image_base64 = None   # From the perpendicular webcam publisher
latest_velocity = None  # Will hold a dictionary with flow_rate data from optical flow sensor.
latest_z_velocity = None  # Will hold the latest z_velocity from the /z_velocity topic.

# Lock to ensure thread-safe access to global image and velocity variables
lock = threading.Lock()

# ROS2 BridgeServer node: subscribes to various topics
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
        self.drone_launch_status = None

        # Subscribers for position topics
        self.pos_current_sub = self.create_subscription(String, 'position/current', self.update_current_pos, 10)
        self.pos_target_sub = self.create_subscription(String, 'position/target', self.update_target_pos, 10)
        self.pos_phase_sub = self.create_subscription(String, 'position/phase', self.update_phase_pos, 10)
        self.mode_pub = self.create_publisher(String, 'position/mode', 10)
        self.land_launch_pub = self.create_publisher(String, '/launch_commands', 10)
        
        # ---------------------------
        # OLD velocity subscriber removed:
        # self.vel_sub = self.create_subscription(
        #     TwistStamped,
        #     '/mavros/local_position/velocity_local',
        #     self.velocity_callback,
        #     10  # QoS depth
        # )
        # ---------------------------
        
        # NEW: Subscribe to the optical flow sensor topic to update flow_rate (x and y)
        self.flow_sub = self.create_subscription(
            OpticalFlow,
            '/mavros/optical_flow/raw/optical_flow',
            self.velocity_callback,
            10
        )
        
        # NEW: Subscribe to the /z_velocity topic to get the calculated vertical speed
        self.z_velocity_sub = self.create_subscription(
            Float32,
            '/z_velocity',
            self.z_velocity_callback,
            10
        )
        
        # Subscriber for launch/land status (String)
        self.drone_launch_land_sub = self.create_subscription(String, '/launch_land_status', self.update_launch_status, 10)
        
        # Subscriber for system ready status (Bool)
        self.system_ready_sub = self.create_subscription(Bool, '/system_launch_status', self.system_ready_status_callback, 10)
        self.system_ready_status = False

        # Subscriber for the camera image topic
        self.camera_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.camera_callback, 10)
        
        # ---------------------------
        # Modified Battery subscriber:
        # Create a QoS profile for the battery topic with BEST_EFFORT reliability.
        qos_profile_battery = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.battery_sub = self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, qos_profile_battery)
        # ---------------------------
        
        self.battery_percent = None
        self.battery_voltage = None

        # Subscriber for flat area points (assumed type Point published on /landing_spots)
        self.flat_area_point_sub = self.create_subscription(Point, '/landing_spots', self.flat_area_point_callback, 10)
        self.flat_area_points = []
        self.flat_area_gap_threshold = 0.5

        # Array to hold {coordinate, image} pairs when near a flat area
        self.ml_check_area_array = []
        self.ml_predicted_area_pub = self.create_publisher(String, '/ml_predicted_area', 10)

        # NEW: Subscriber for perpendicular webcam image (String message)
        self.perpendicular_cam_sub = self.create_subscription(String, '/perpendicular_cam', self.perpendicular_cam_callback, 10)

        # NEW: Subscriber for current height (String message published by the range_to_height node)
        self.height_sub = self.create_subscription(String, '/current_height', self.update_current_height, 10)
        self.current_height = None

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()        

    # Updated velocity callback using optical flow sensor's flow_rate for x and y
    def velocity_callback(self, msg: OpticalFlow):
        global latest_velocity, lock
        # Extract the flow_rate from the optical flow message.
        flow_rate = msg.flow_rate
        
        # Build a dictionary to store the flow rate data (x and y).
        velocity_data = {
            'header': {
                'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'frame_id': msg.header.frame_id
            },
            'flow_rate': {
                'x': flow_rate.x,
                'y': flow_rate.y,
                # The z component will be replaced from the /z_velocity topic.
                'z': 0.0
            }
        }
        with lock:
            latest_velocity = velocity_data
        self.get_logger().info("Updated optical flow rate (x,y): " + str(velocity_data))
    
    # New callback to update the z_velocity from the /z_velocity topic.
    def z_velocity_callback(self, msg: Float32):
        global latest_z_velocity, lock
        with lock:
            latest_z_velocity = msg.data
        self.get_logger().info("Updated z velocity from /z_velocity: " + str(msg.data))
    
    def system_ready_status_callback(self, msg: Bool):
        self.system_ready_status = msg.data
        self.get_logger().info(f"System ready status updated to: {self.system_ready_status}")

    def update_current_pos(self, msg: String):
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

    def update_launch_status(self, msg: String):
        new_status = msg.data
        if new_status not in ['launched', 'landed', 'launching', 'landing']:
            self.get_logger().info("Invalid launch status received: " + new_status)
            return
        self.drone_launch_status = new_status        
        self.get_logger().info(f"Launch status updated to: {self.drone_launch_status}")

    def update_current_height(self, msg: String):
        try:
            # Convert height string to a float value
            self.current_height = float(msg.data)
            self.get_logger().info(f"Updated current height: {self.current_height}")
        except Exception as e:
            self.get_logger().error("Failed to parse current height: " + str(e))

    def camera_callback(self, msg: Image):
        global latest_image_base64
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (512, 512))
            ret, buffer = cv2.imencode('.jpg', cv_image)
            if ret:
                jpg_as_text = base64.b64encode(buffer).decode('utf-8')
                with lock:
                    latest_image_base64 = jpg_as_text
        except CvBridgeError as e:
            self.get_logger().error('CvBridge Error: ' + str(e))

    def battery_callback(self, msg: BatteryState):
        self.battery_percent = msg.percentage
        self.battery_voltage = msg.voltage
        self.get_logger().info(f"Battery: {self.battery_percent*100:.1f}% | Voltage: {self.battery_voltage:.2f}V")

    def flat_area_point_callback(self, msg: Point):
        new_point = {'x': msg.x, 'y': msg.y, 'z': msg.z}
        unique = True
        for p in self.flat_area_points:
            dx = new_point['x'] - p['x']
            dy = new_point['y'] - p['y']
            dz = new_point['z'] - p['z']
            if math.sqrt(dx**2 + dy**2 + dz**2) < self.flat_area_gap_threshold:
                unique = False
                break
        if unique:
            self.flat_area_points.append(new_point)
            self.get_logger().info(f"Added new flat area point: {new_point}")
        else:
            self.get_logger().info("Received flat area point is not unique; ignoring.")

    def perpendicular_cam_callback(self, msg: String):
        global perpendicular_image_base64
        # Simply update the global variable with the latest perpendicular webcam image
        with lock:
            perpendicular_image_base64 = msg.data
        self.get_logger().info("Updated perpendicular webcam image.")

# Set up the Flask app and endpoints
app = Flask(__name__, static_folder='static')
CORS(app)
bridge_server_node = None  # Global variable for the ROS node
shutdown_flag = threading.Event()  # Signal to stop threads

# Endpoint: Return current position
@app.route('/current_position', methods=['GET'])
def current_position():
    global bridge_server_node
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500
    pos = {
        'x': bridge_server_node.x_,
        'y': bridge_server_node.y_,
        'z': bridge_server_node.current_height          # changed to data from the height sensor
    }
    return jsonify(pos)

# Endpoint: Return target position
@app.route('/target_position', methods=['GET'])
def target_position():
    global bridge_server_node
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500
    pos = {
        'x': bridge_server_node.target_x * 100,
        'y': bridge_server_node.target_y * 100,
        # 'z': bridge_server_node.target_z
        'z': 3 * 100
    }
    return jsonify(pos)

# Endpoint: Set mode
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

# Endpoint: Return the latest camera image in base64
@app.route('/depth_cam_rgb', methods=['GET'])
def get_latest_image():
    global latest_image_base64
    with lock:
        if latest_image_base64 is not None:
            return jsonify({'image': latest_image_base64})
        else:
            return jsonify({'error': 'No image received yet'}), 404

# Modified Endpoint: Return the latest perpendicular webcam image as base64
@app.route('/perpendicular_cam_rgb', methods=['GET'])
def get_perpendicular_cam():
    global perpendicular_image_base64
    with lock:
        if perpendicular_image_base64 is not None:
            return jsonify({'image': perpendicular_image_base64})
        else:
            return jsonify({'error': 'No perpendicular webcam image received yet'}), 404

# New Endpoint: Return the current height from the rangefinder
@app.route('/current_height', methods=['GET'])
def current_height_endpoint():
    global bridge_server_node
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500
    if bridge_server_node.current_height is None:
        return jsonify({'error': 'Current height not available'}), 404
    return jsonify({'current_height': bridge_server_node.current_height})

# New Endpoint: Serve static HTML files from a directory via /home
@app.route('/home', methods=['GET'])
def home():
    # Serve the index.html file from the static directory
    return send_from_directory(app.static_folder, 'interface.html')

# NEW: Endpoint to serve any file from the static directory
@app.route('/<path:filename>', methods=['GET'])
def serve_static(filename):
    return send_from_directory(app.static_folder, filename)

# Endpoint: Return battery status
@app.route('/battery_status', methods=['GET'])
def battery_status():
    global bridge_server_node
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500
    battery = {
        'percentage': bridge_server_node.battery_percent * 100 if bridge_server_node.battery_percent is not None else None,
        'voltage': bridge_server_node.battery_voltage
    }
    return jsonify(battery)

# Endpoint: Return system ready status
@app.route('/system_ready', methods=['GET'])
def system_ready():
    global bridge_server_node
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500
    return jsonify({'system_ready': bridge_server_node.system_ready_status})

# Endpoint: /ml_check_area (unchanged)
@app.route('/ml_check_area', methods=['GET'])
def ml_check_area():
    global bridge_server_node, perpendicular_image_base64
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500
    near_threshold = 1.0  # meters
    current_coord = {'x': bridge_server_node.x_, 'y': bridge_server_node.y_, 'z': bridge_server_node.z_}
    found = None
    for point in bridge_server_node.flat_area_points:
        dx = current_coord['x'] - point['x']
        dy = current_coord['y'] - point['y']
        dz = current_coord['z'] - point['z']
        if math.sqrt(dx**2 + dy**2 + dz**2) < near_threshold:
            found = point
            break
    if found and perpendicular_image_base64 is not None:
        entry = {'coordinate': current_coord, 'image': perpendicular_image_base64}
        exists = False
        for e in bridge_server_node.ml_check_area_array:
            ec = e['coordinate']
            if (abs(ec['x'] - current_coord['x']) < 0.1 and 
                abs(ec['y'] - current_coord['y']) < 0.1 and 
                abs(ec['z'] - current_coord['z']) < 0.1):
                exists = True
                break
        if not exists:
            bridge_server_node.ml_check_area_array.append(entry)
            bridge_server_node.get_logger().info(f"Added ml_check_area entry: {entry}")
        return jsonify(entry)
    else:
        return jsonify({'error': 'No flat area nearby or no perpendicular image available'}), 404

# New Endpoint: /ml_check_area_temp
# Always returns the current coordinates and the latest depth camera RGB image
@app.route('/ml_check_area_temp', methods=['GET'])
def ml_check_area_temp():
    global bridge_server_node, latest_image_base64
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500

    current_coord = {
        'x': bridge_server_node.x_,
        'y': bridge_server_node.y_,
        'z': bridge_server_node.z_
    }

    with lock:
        if latest_image_base64 is not None:
            response = {'coordinate': current_coord, 'image': latest_image_base64}
            return jsonify(response)
        else:
            return jsonify({'error': 'No image received yet'}), 404

# Endpoint: /predicted_area (modified publisher format)
@app.route('/predicted_area', methods=['POST'])
def predicted_area():
    global bridge_server_node
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500

    data = request.get_json(force=True)
    required_keys = ['status', 'class', 'accuracy', 'coordinate']
    if not all(k in data for k in required_keys):
        return jsonify({'error': 'Missing keys in JSON body'}), 400

    # Ensure that 'class' is present (already checked above) and not empty
    if not data['class']:
        return jsonify({'error': 'Missing terrain class in request'}), 400

    # Extract and parse the coordinate string into individual values
    coord_str = data['coordinate']
    try:
        parts = coord_str.split()
        coord = {}
        for part in parts:
            key, value = part.split('=')
            coord[key] = float(value)
    except Exception as e:
        return jsonify({'error': 'Failed to parse coordinate in request'}), 400

    # Check that the required coordinate keys exist
    if not all(k in coord for k in ['x', 'y', 'z']):
        return jsonify({'error': 'Missing one or more coordinate values (x, y, z)'}), 400

    # Build the publisher message in the desired format:
    # "class <terrain> coordinate x=<x_val> y=<y_val> z=<z_val>"
    msg_data = f"class {data['class']} coordinate x={coord['x']} y={coord['y']} z={coord['z']}"
    msg = String()
    msg.data = msg_data
    bridge_server_node.ml_predicted_area_pub.publish(msg)
    bridge_server_node.get_logger().info(f"Published predicted area: {msg.data}")

    new_array = []
    removed = False
    for entry in bridge_server_node.ml_check_area_array:
        ec = entry['coordinate']
        if (abs(ec['x'] - coord['x']) < 0.1 and 
            abs(ec['y'] - coord['y']) < 0.1 and 
            abs(ec['z'] - coord['z']) < 0.1):
            removed = True
            continue
        new_array.append(entry)
    bridge_server_node.ml_check_area_array = new_array
    if removed:
        return jsonify({'status': 'Entry removed and published'}), 200
    else:
        return jsonify({'error': 'No matching entry found'}), 404
        
@app.route('/launch', methods=['GET'])
def launch():
    global bridge_server_node
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500

    # Create and publish a "launch" command message.
    msg = String()
    msg.data = "launch"
    bridge_server_node.land_launch_pub.publish(msg)
    bridge_server_node.get_logger().info("Published launch command.")
    return jsonify({'status': 'launch command published'}), 200

# Endpoint: /drone_status - returns current phase and launch_land_status
@app.route('/drone_status', methods=['GET'])
def drone_status():
    global bridge_server_node
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500
    status = {
        'phase': bridge_server_node.phase,
        'launch_land_status': bridge_server_node.drone_launch_status
    }
    return jsonify(status)

# Updated Endpoint: /velocities now returns optical flow x,y and replaces z with value from /z_velocity
@app.route('/velocities', methods=['GET'])
def get_velocities():
    global latest_velocity, latest_z_velocity, lock
    with lock:
        if latest_velocity is None:
            return jsonify({'error': 'No velocity data received yet'}), 404
        # Replace the z component with the latest value from /z_velocity if available
        velocity_data = latest_velocity.copy()
        if latest_z_velocity is not None:
            velocity_data['flow_rate']['z'] = latest_z_velocity
        else:
            velocity_data['flow_rate']['z'] = 0.0
        return jsonify(velocity_data)

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
    # Initialize arrays that need to be set up (do not recreate the publisher; it's already created in __init__)
    node.ml_check_area_array = []
    node.flat_area_points = []
    bridge_server_node = node

    ros_thread = threading.Thread(target=ros_spin, args=(node,))
    ros_thread.start()

    try:
        app.run(host='0.0.0.0', port=5000, use_reloader=False)
    except KeyboardInterrupt:
        print('Keyboard interrupt caught. Bridge server shutting down gracefully.')
    finally:
        shutdown_flag.set()
        ros_thread.join(timeout=5)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
