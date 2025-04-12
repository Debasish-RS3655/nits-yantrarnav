#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import Point

from flask import Flask, jsonify
from flask_cors import CORS
import threading
import base64
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math

# Global variables to hold the latest images (in base64)
latest_image_base64 = None          # From the ROS camera subscriber
perpendicular_image_base64 = None   # From the perpendicular webcam publisher

# Lock to ensure thread-safe access to global image variables
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

        # Subscriber for launch/land status
        self.drone_launch_land_sub = self.create_subscription(String, '/launch_land_status', self.update_launch_status, 10)

        # Subscriber for system ready status
        self.system_ready_sub = self.create_subscription(Bool, '/system_launch_status', self.system_ready_status_callback, 10)
        self.system_ready_status = False

        # Subscriber for the camera image topic
        self.camera_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.camera_callback, 10)

        # Battery status subscriber
        self.battery_sub = self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, 10)
        self.battery_percent = None
        self.battery_voltage = None

        # Subscriber for flat area points (assumed type Point published on /landing_spots)
        self.flat_area_point_sub = self.create_subscription(Point, '/landing_spots', self.flat_area_point_callback, 10)
        self.flat_area_points = []
        self.flat_area_gap_threshold = 0.5

        # Array to hold {coordinate, image} pairs when near a flat area
        self.ml_check_area_array = []
        self.ml_predicted_area_pub = self.create_publisher(String, '/ml_predicted_area', 10)

        # Subscriber for perpendicular webcam image
        self.perpendicular_cam_sub = self.create_subscription(String, '/perpendicular_cam', self.perpendicular_cam_callback, 10)

        # Subscriber for current height (published by the range_to_height node)
        self.height_sub = self.create_subscription(String, '/current_height', self.update_current_height, 10)
        self.current_height = None

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()

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
        with lock:
            perpendicular_image_base64 = msg.data
        self.get_logger().info("Updated perpendicular webcam image.")

# Set up the Flask app and endpoints
app = Flask(__name__, static_folder='static')
CORS(app)
bridge_server_node = None  # Global variable for the ROS node
shutdown_flag = threading.Event()  # Signal to stop threads

# Single GET endpoint for all current parameters
@app.route('/all', methods=['GET'])
def get_all_parameters():
    global bridge_server_node, latest_image_base64, perpendicular_image_base64
    if bridge_server_node is None:
        return jsonify({'error': 'BridgeServer node not available'}), 500

    with lock:
        response = {
            'current_position': {
                'x': bridge_server_node.x_,
                'y': bridge_server_node.y_,
                'z': bridge_server_node.z_
            },
            'target_position': {
                'x': bridge_server_node.target_x,
                'y': bridge_server_node.target_y,
                'z': bridge_server_node.target_z
            },
            'phase': bridge_server_node.phase,
            'launch_status': bridge_server_node.drone_launch_status,
            'system_ready': bridge_server_node.system_ready_status,
            'battery': {
                'percentage': (bridge_server_node.battery_percent * 100)
                               if bridge_server_node.battery_percent is not None else None,
                'voltage': bridge_server_node.battery_voltage
            },
            'current_height': bridge_server_node.current_height,
            'flat_area_points': bridge_server_node.flat_area_points,
            'latest_image': latest_image_base64,
            'perpendicular_image': perpendicular_image_base64,
            'ml_check_area_array': bridge_server_node.ml_check_area_array
        }
    return jsonify(response)

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
