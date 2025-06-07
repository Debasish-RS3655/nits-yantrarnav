#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import cv2
import numpy as np
import math
import time
import requests
import base64

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        # --- Publishers ---
        self.drone_commands_pub = self.create_publisher(String, 'drone_commands', 10)
        self.yellow_boundary_found_pub = self.create_publisher(Bool, 'yellow_boundary_found', 10)
        self.yellow_boundary_map_status_pub = self.create_publisher(String, 'yello_boundary_map_status', 10)
        self.edge_coordinates_pub = self.create_publisher(String, 'edge_coordinates', 10)
        
        # --- Subscriber ---
        self.phase_sub = self.create_subscription(String, 'position/phase', self.phase_callback, 10)
        
        # --- Parameters ---
        self.current_phase = "0"   # initial phase; updated by the subscriber
        self.center_tolerance = 20  # allowed horizontal deviation in pixels
        self.angle_threshold = 20   # degrees threshold for detecting a corner/edge
        self.min_area = 100         # minimum area for a valid yellow tape contour
        
        # --- Internal state ---
        self.edge_count = 0  # to count detected edges
        
        # Create a timer that runs the main loop (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Line Follower Node started, waiting for image and phase info...")
        
    def phase_callback(self, msg: String):
        self.current_phase = msg.data
        self.get_logger().info("Phase updated: " + self.current_phase)
        self.get_logger().debug("Received phase message: " + msg.data)

    def get_image_from_server(self):
        """
        Sends a GET request to http://localhost:5000/depth_cam_rgb
        and returns the decoded OpenCV image if successful.
        """
        try:
            response = requests.get("http://localhost:5000/depth_cam_rgb", timeout=1.0)
            if response.status_code == 200:
                data = response.json()
                image_b64 = data.get("image", "")
                if image_b64:
                    image_data = base64.b64decode(image_b64)
                    np_arr = np.frombuffer(image_data, np.uint8)
                    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if img is None:
                        self.get_logger().error("cv2.imdecode returned None. Check the image format!")
                    else:
                        self.get_logger().debug("Image decoded successfully from server.")
                    return img
                else:
                    self.get_logger().error("No 'image' key found in server response.")
            else:
                self.get_logger().error("Failed to get image from server (HTTP status: " + str(response.status_code) + ")")
        except Exception as e:
            self.get_logger().error("Exception in get_image_from_server: " + str(e))
        return None

    def process_image(self, frame):
        """
        Process the frame to detect yellow tape and compute:
         - status_message: "Tape centered", "Error: Tape off center!", or "Corner reached!"
         - command_message: e.g. "Forward Command", "Maintain Command", or "No Command"
         - detailed_info: string with computed details (centroid, deviation, angle)
        Returns a tuple: (status_message, command_message, detailed_info)
        """
        height, width = frame.shape[:2]
        horizontal_center = width // 2

        # Convert to HSV and threshold for yellow color.
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv_frame, yellow_lower, yellow_upper)
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(yellow_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.get_logger().debug("Detected " + str(len(contours)) + " yellow contours.")

        status_message = ""
        command_message = ""
        detailed_info = ""

        if not contours:
            status_message = "Error: No yellow tape detected"
            command_message = "No Command"
        else:
            # Select the largest contour
            main_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(main_contour)
            self.get_logger().debug("Largest contour area: " + str(area))
            if area < self.min_area:
                status_message = "Error: Yellow tape too small"
                command_message = "No Command"
            else:
                # Calculate centroid and bounding box
                x, y, w, h = cv2.boundingRect(main_contour)
                M = cv2.moments(main_contour)
                if M["m00"] != 0:
                    centroid_x = int(M["m10"] / M["m00"])
                    centroid_y = int(M["m01"] / M["m00"])
                else:
                    centroid_x = x + w // 2
                    centroid_y = y + h // 2

                deviation = horizontal_center - centroid_x

                # Fit a line to the contour to get orientation.
                line = cv2.fitLine(main_contour, cv2.DIST_L2, 0, 0.01, 0.01)
                vx, vy = line[0][0], line[1][0]
                angle_deg = math.degrees(math.atan2(vy, vx))
                angle_deviation = abs(abs(angle_deg) - 90)

                self.get_logger().debug(
                    f"Centroid: ({centroid_x},{centroid_y}), Deviation: {deviation}, Angle: {angle_deg:.1f}, AngleDev: {angle_deviation:.1f}"
                )

                if angle_deviation > self.angle_threshold:
                    status_message = "Corner reached!"
                elif abs(deviation) > self.center_tolerance:
                    status_message = "Error: Tape off center!"
                else:
                    status_message = "Tape centered"

                # Modified command logic:
                # If tape is off center, send a "Forward Command"
                # If tape is centered, send a "Maintain Command"
                # Otherwise, send "No Command"
                if "off center" in status_message:
                    command_message = "Forward Command"
                elif "Tape centered" in status_message:
                    command_message = "Maintain Command"
                else:
                    command_message = "No Command"

                detailed_info = f"Centroid=({centroid_x},{centroid_y}), Deviation={deviation}, Angle={angle_deg:.1f}, AngleDev={angle_deviation:.1f}"
        return status_message, command_message, detailed_info

    def timer_callback(self):
        frame = self.get_image_from_server()
        if frame is not None:
            self.get_logger().info(f"Received frame: shape={frame.shape}, dtype={frame.dtype}")
            self.get_logger().debug("Sample pixel data (top-left 5x5):\n" + str(frame[:5, :5]))
            
            status_msg, command_msg, details = self.process_image(frame)
            self.get_logger().info("[STATUS] " + status_msg)
            self.get_logger().info("[COMMAND] " + command_msg)
            self.get_logger().info("[DETAIL] " + details)
        else:
            self.get_logger().error("No frame received from server!")
            dummy_cmd = String()
            dummy_cmd.data = "Dummy Command: No Frame"
            self.drone_commands_pub.publish(dummy_cmd)
            self.get_logger().info("Published dummy command on /drone_commands.")
            return

        # Publish based on the current phase.
        if self.current_phase == "0":
            if "Corner reached!" in status_msg:
                yellow_msg = Bool()
                yellow_msg.data = True
                self.yellow_boundary_found_pub.publish(yellow_msg)
                self.get_logger().info("Published yellow boundary found signal on /yellow_boundary_found.")
                self.current_phase = "2"
            else:
                cmd_msg = String()
                cmd_msg.data = command_msg
                self.drone_commands_pub.publish(cmd_msg)
                self.get_logger().debug("Published drone command: " + command_msg)

        elif self.current_phase == "1":
            map_msg = String()
            map_msg.data = "mapping"
            self.yellow_boundary_map_status_pub.publish(map_msg)
            self.get_logger().debug("Published mapping status on /yello_boundary_map_status: mapping")
            
            cmd_msg = String()
            cmd_msg.data = command_msg
            self.drone_commands_pub.publish(cmd_msg)
            self.get_logger().debug("Published drone command in mapping phase: " + command_msg)

            if "Corner reached!" in status_msg:
                self.edge_count += 1
                edge_msg = String()
                edge_msg.data = "x=0.0 y=0.0 z=0.0"  # Replace with actual coordinates if available.
                self.edge_coordinates_pub.publish(edge_msg)
                self.get_logger().info(f"Edge {self.edge_count} detected; published coordinates on /edge_coordinates.")
                if self.edge_count == 3:
                    comp_msg = String()
                    comp_msg.data = "completed"
                    self.yellow_boundary_map_status_pub.publish(comp_msg)
                    self.get_logger().info("Mapping completed: published 'completed' on /yello_boundary_map_status.")
                    fourth_edge_msg = String()
                    fourth_edge_msg.data = "x=0.0 y=0.0 z=0.0"
                    self.edge_coordinates_pub.publish(fourth_edge_msg)
                    self.get_logger().info("Published fourth edge coordinates on /edge_coordinates.")

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
