#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
import time
from enum import Enum

class DroneState(Enum):
    INITIALIZING = 0
    WAITING_FOR_ARM = 1
    SEARCHING_LINE = 2
    ORIENTING = 3
    FOLLOWING_LINE = 4
    TURNING = 5
    LINE_LOST = 6 # Optional: Add a landing state

class DroneLineFollowerNode(Node):
    def __init__(self):
        super().__init__('drone_line_follower_node')

        # --- Parameters ---
        self.declare_parameter('image_topic', '/camera/image_raw') # Adjust topic name as needed
        self.declare_parameter('forward_velocity', 0.2) # m/s
        self.declare_parameter('yaw_rate_gain', 0.1)   # P gain for yaw control based on line angle
        self.declare_parameter('roll_rate_gain', 0.005) # P gain for roll control based on deviation (adjust sign based on frame)
        self.declare_parameter('turn_yaw_rate', 0.5)   # rad/s for 90-degree turn
        self.declare_parameter('turn_duration_factor', 1.6) # Multiplier for calculated turn time (pi/2 / rate) to ensure completion
        self.declare_parameter('center_tolerance', 30)  # allowed horizontal deviation in pixels
        self.declare_parameter('angle_tolerance', 10)   # degrees threshold for orientation
        self.declare_parameter('corner_angle_threshold', 45) # degrees threshold to consider a corner
        self.declare_parameter('min_area', 500)         # minimum area for a valid yellow tape contour
        self.declare_parameter('search_yaw_rate', 0.3)  # rad/s while searching
        self.declare_parameter('search_forward_vel', 0.1) # m/s while searching
        self.declare_parameter('line_lost_timeout', 2.0) # seconds before declaring line lost

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.forward_velocity = self.get_parameter('forward_velocity').get_parameter_value().double_value
        self.yaw_rate_gain = self.get_parameter('yaw_rate_gain').get_parameter_value().double_value
        self.roll_rate_gain = self.get_parameter('roll_rate_gain').get_parameter_value().double_value
        self.turn_yaw_rate = self.get_parameter('turn_yaw_rate').get_parameter_value().double_value
        self.turn_duration = (math.pi / 2.0) / self.turn_yaw_rate * self.get_parameter('turn_duration_factor').get_parameter_value().double_value
        self.center_tolerance = self.get_parameter('center_tolerance').get_parameter_value().integer_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        self.corner_angle_threshold = self.get_parameter('corner_angle_threshold').get_parameter_value().double_value
        self.min_area = self.get_parameter('min_area').get_parameter_value().integer_value
        self.search_yaw_rate = self.get_parameter('search_yaw_rate').get_parameter_value().double_value
        self.search_forward_vel = self.get_parameter('search_forward_vel').get_parameter_value().double_value
        self.line_lost_timeout = self.get_parameter('line_lost_timeout').get_parameter_value().double_value


        # --- MAVROS Communication ---
        # Define QoS profile for MAVROS compatibility
        qos_profile_mavros = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile_mavros)
        self.vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10) # Use unstamped for body frame

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting again...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set mode service not available, waiting again...')

        # --- Image Processing ---
        self.bridge = CvBridge()
        # Define QoS profile for sensor data
        qos_profile_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data)

        # --- Internal State ---
        self.current_mavros_state = State()
        self.current_drone_state = DroneState.INITIALIZING
        self.cv_image = None
        self.last_image_time = self.get_clock().now()
        self.last_line_detection_time = self.get_clock().now()
        self.turn_start_time = None
        self.orientation_attempts = 0 # Counter for orientation stabilization
        self.last_deviation = 0.0 # Store last known deviation for searching

        # --- Main Control Loop Timer ---
        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz

        self.get_logger().info("Drone Line Follower Node started.")
        self.get_logger().info(f"Parameters: fwd_vel={self.forward_velocity}, "
                               f"yaw_gain={self.yaw_rate_gain}, roll_gain={self.roll_rate_gain}, "
                               f"turn_rate={self.turn_yaw_rate}, turn_dur={self.turn_duration:.2f}, "
                               f"center_tol={self.center_tolerance}, angle_tol={self.angle_tolerance}, "
                               f"corner_thresh={self.corner_angle_threshold}, min_area={self.min_area}")


    def state_callback(self, msg: State):
        """Stores the current MAVROS state."""
        if self.current_mavros_state.mode != msg.mode:
            self.get_logger().info(f"Mode changed: {self.current_mavros_state.mode} -> {msg.mode}")
        if self.current_mavros_state.armed != msg.armed:
             self.get_logger().info(f"Arming status changed: {self.current_mavros_state.armed} -> {msg.armed}")
        self.current_mavros_state = msg

    def image_callback(self, msg: Image):
        """Converts ROS Image message to OpenCV image."""
        self.last_image_time = self.get_clock().now()
        try:
            # Ensure encoding is suitable for color detection
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            self.cv_image = None
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
            self.cv_image = None

    def process_image(self, frame):
        """
        Detects the yellow line and returns its properties.
        Returns a dictionary: {'detected': bool, 'deviation': float, 'angle_deg': float, 'is_corner': bool}
        deviation: Pixels from image center (+ve means line is to the right).
        angle_deg: Angle of the line segment (0=horizontal right, 90=vertical up).
        is_corner: True if the angle suggests a corner.
        """
        if frame is None:
            return {'detected': False}

        height, width = frame.shape[:2]
        horizontal_center = width // 2

        # Convert image to HSV and threshold for yellow color.
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_lower = np.array([20, 100, 100]) # Adjust these values based on lighting
        yellow_upper = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv_frame, yellow_lower, yellow_upper)

        # Optional: Apply morphological operations
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(yellow_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return {'detected': False}

        # Select the largest contour
        main_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(main_contour)

        if area < self.min_area:
            return {'detected': False}

        # Calculate centroid
        M = cv2.moments(main_contour)
        if M["m00"] != 0:
            centroid_x = int(M["m10"] / M["m00"])
            # centroid_y = int(M["m01"] / M["m00"]) # Not used for control here
        else:
             return {'detected': False} # Avoid division by zero

        deviation = centroid_x - horizontal_center # Positive if line is right of center

        # Fit a line to the contour to determine orientation.
        if len(main_contour) >= 5: # fitLine requires at least 5 points
            try:
                line = cv2.fitLine(main_contour, cv2.DIST_L2, 0, 0.01, 0.01)
                vx, vy = line[0][0], line[1][0]
                # Calculate angle respecting OpenCV's coordinate system (y increases downwards)
                # We want angle relative to horizontal x-axis
                angle_rad = math.atan2(-vy, vx) # Negate vy for standard angle convention
                angle_deg = math.degrees(angle_rad) # Range [-180, 180]
                # Normalize angle to be primarily upward (e.g., near 90 degrees)
                if angle_deg < -90:
                     angle_deg += 180
                elif angle_deg > 90:
                    angle_deg -= 180
                # Now angle_deg should be roughly in [-90, 90], representing deviation from vertical
            except Exception as e:
                 self.get_logger().warn(f"fitLine failed: {e}")
                 angle_deg = 0 # Assume horizontal if fitLine fails? Or maybe return undetected?
                 return {'detected': False} # Safer to return undetected
        else:
            self.get_logger().warn("Contour too small for fitLine.")
            return {'detected': False}

        # Determine if it's a corner based on how far the angle is from vertical (90 or -90)
        # angle_deviation_from_vertical = abs(abs(angle_deg) - 90) # Old calculation
        angle_deviation_from_vertical = abs(angle_deg) # Angle relative to horizontal X axis, 0 means horizontal line (corner)

        is_corner = angle_deviation_from_vertical < self.corner_angle_threshold or angle_deviation_from_vertical > (180 - self.corner_angle_threshold)
        # is_corner = angle_deviation_from_vertical > self.corner_angle_threshold

        self.last_line_detection_time = self.get_clock().now()
        self.last_deviation = deviation # Store last good deviation

        return {
            'detected': True,
            'deviation': deviation,
            'angle_deg': angle_deg, # Angle relative to horizontal (0 right, 90 up, -90 down)
            'is_corner': is_corner
        }

    def set_mode(self, mode):
        """Sets the MAVROS flight mode."""
        if self.current_mavros_state.mode == mode:
            return True
        self.get_logger().info(f"Attempting to set mode to {mode}...")
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        # We don't explicitly wait here in the callback, handle transitions based on state updates

    def arm_vehicle(self):
        """Arms the vehicle via MAVROS."""
        if self.current_mavros_state.armed:
            return True
        self.get_logger().info("Attempting to arm vehicle...")
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        # We don't explicitly wait here

    def disarm_vehicle(self):
        """Disarms the vehicle via MAVROS."""
        if not self.current_mavros_state.armed:
            return True
        self.get_logger().info("Attempting to disarm vehicle...")
        req = CommandBool.Request()
        req.value = False
        future = self.arming_client.call_async(req)

    def control_loop(self):
        """Main state machine and control logic."""
        cmd_vel = Twist() # Initialize zero velocity

        # --- State Machine Logic ---
        if self.current_drone_state == DroneState.INITIALIZING:
            self.get_logger().info("State: INITIALIZING - Waiting for MAVROS connection and services.")
            # Check if services are ready (already done in __init__)
            # Check for MAVROS connection via state callback
            if self.current_mavros_state.connected:
                 self.get_logger().info("MAVROS connected.")
                 self.set_mode("OFFBOARD") # Request OFFBOARD mode
                 self.current_drone_state = DroneState.WAITING_FOR_ARM
            else:
                # Keep waiting
                pass

        elif self.current_drone_state == DroneState.WAITING_FOR_ARM:
            self.get_logger().info("State: WAITING_FOR_ARM")
            if self.current_mavros_state.mode == "OFFBOARD" and not self.current_mavros_state.armed:
                self.arm_vehicle()
            elif self.current_mavros_state.mode == "OFFBOARD" and self.current_mavros_state.armed:
                self.get_logger().info("Armed and in OFFBOARD mode. Transitioning to SEARCHING_LINE.")
                self.current_drone_state = DroneState.SEARCHING_LINE
                self.last_line_detection_time = self.get_clock().now() # Reset timer
            else:
                # Still waiting for OFFBOARD mode confirmation or arming success
                self.set_mode("OFFBOARD") # Keep requesting OFFBOARD


        elif self.current_drone_state == DroneState.SEARCHING_LINE:
            self.get_logger().info("State: SEARCHING_LINE")
            line_info = self.process_image(self.cv_image)

            if line_info['detected']:
                self.get_logger().info("Line detected during search. Transitioning to ORIENTING.")
                self.current_drone_state = DroneState.ORIENTING
                self.orientation_attempts = 0 # Reset counter
            else:
                # Command a slow forward movement and maybe a slow yaw to find the line
                # This assumes the drone starts somewhat facing the line area
                cmd_vel.linear.x = self.search_forward_vel
                # Optional: Add a slow rotation if line isn't found quickly
                # cmd_vel.angular.z = self.search_yaw_rate
                self.get_logger().info(f"Searching: Moving forward at {self.search_forward_vel} m/s")


        elif self.current_drone_state == DroneState.ORIENTING:
            self.get_logger().info("State: ORIENTING")
            line_info = self.process_image(self.cv_image)

            if not line_info['detected']:
                self.get_logger().warn("Line lost during orientation. Reverting to SEARCHING_LINE.")
                self.current_drone_state = DroneState.SEARCHING_LINE
            else:
                 # Target angle is 90 degrees (vertical up in image)
                 angle_error_deg = line_info['angle_deg'] - 90.0
                 # Normalize angle error to [-180, 180]
                 if angle_error_deg > 180: angle_error_deg -= 360
                 if angle_error_deg < -180: angle_error_deg += 360

                 angle_error_rad = math.radians(angle_error_deg)

                 self.get_logger().info(f"Orienting: Angle={line_info['angle_deg']:.1f}, Error={angle_error_deg:.1f} deg")

                 if abs(angle_error_deg) < self.angle_tolerance:
                     self.orientation_attempts += 1
                     if self.orientation_attempts >= 3: # Require stable orientation for a few cycles
                        self.get_logger().info("Orientation achieved. Transitioning to FOLLOWING_LINE.")
                        self.current_drone_state = DroneState.FOLLOWING_LINE
                        cmd_vel.angular.z = 0.0 # Stop rotation
                     else:
                         self.get_logger().info(f"Orientation close, stabilizing ({self.orientation_attempts}/3)...")
                         # Apply small correction or hold zero if very close
                         cmd_vel.angular.z = -self.yaw_rate_gain * angle_error_rad # Corrective yaw
                 else:
                     self.orientation_attempts = 0 # Reset counter if error increases
                     cmd_vel.angular.z = -self.yaw_rate_gain * angle_error_rad # Corrective yaw
                     self.get_logger().info(f"Applying yaw correction: {cmd_vel.angular.z:.2f} rad/s")
                 # Hold position or move very slowly forward during orientation
                 cmd_vel.linear.x = 0.05


        elif self.current_drone_state == DroneState.FOLLOWING_LINE:
            self.get_logger().info("State: FOLLOWING_LINE")
            line_info = self.process_image(self.cv_image)

            if not line_info['detected']:
                 time_since_last_detection = (self.get_clock().now() - self.last_line_detection_time).nanoseconds / 1e9
                 if time_since_last_detection > self.line_lost_timeout:
                     self.get_logger().warn("Line lost while following. Transitioning to LINE_LOST.")
                     self.current_drone_state = DroneState.LINE_LOST
                 else:
                     self.get_logger().warn(f"Line temporarily lost ({time_since_last_detection:.1f}s), continuing straight...")
                     # Continue with previous commands or just move forward
                     cmd_vel.linear.x = self.forward_velocity
                     # Optionally use last known deviation to try and correct:
                     # cmd_vel.linear.y = -self.roll_rate_gain * self.last_deviation
            else:
                # Check for corner first
                if line_info['is_corner']:
                    self.get_logger().info("Corner detected! Transitioning to TURNING.")
                    self.current_drone_state = DroneState.TURNING
                    self.turn_start_time = self.get_clock().now()
                    # Stop forward/lateral motion before turning
                    cmd_vel.linear.x = 0.0
                    cmd_vel.linear.y = 0.0
                    cmd_vel.angular.z = self.turn_yaw_rate # Start turning immediately
                else:
                    # --- Calculate Control Commands ---
                    deviation = line_info['deviation']
                    angle_error_deg = line_info['angle_deg'] - 90.0
                    if angle_error_deg > 180: angle_error_deg -= 360
                    if angle_error_deg < -180: angle_error_deg += 360
                    angle_error_rad = math.radians(angle_error_deg)

                    # Roll command based on horizontal deviation (adjust sign based on camera/frame)
                    # Linear Y in MAVROS body frame corresponds to roll/strafe
                    cmd_vel.linear.y = -self.roll_rate_gain * deviation # (-) if right dev needs left roll

                    # Yaw command based on line angle deviation from vertical
                    # Angular Z in MAVROS body frame is yaw rate
                    cmd_vel.angular.z = -self.yaw_rate_gain * angle_error_rad # (-) if line angled left needs right yaw

                    # Constant forward velocity
                    # Linear X in MAVROS body frame is forward velocity
                    cmd_vel.linear.x = self.forward_velocity

                    self.get_logger().info(f"Following: Dev={deviation:.1f}px, AngleErr={angle_error_deg:.1f}deg -> "
                                           f"RollCmd={cmd_vel.linear.y:.2f}, YawCmd={cmd_vel.angular.z:.2f}, Fwd={cmd_vel.linear.x:.2f}")


        elif self.current_drone_state == DroneState.TURNING:
            self.get_logger().info("State: TURNING")
            if self.turn_start_time is None:
                 self.get_logger().error("Turn started without setting start time! Reverting to search.")
                 self.current_drone_state = DroneState.SEARCHING_LINE
            else:
                elapsed_turn_time = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
                self.get_logger().info(f"Turning for {elapsed_turn_time:.2f} / {self.turn_duration:.2f} seconds.")

                if elapsed_turn_time >= self.turn_duration:
                    self.get_logger().info("Turn completed. Transitioning to SEARCHING_LINE.")
                    self.current_drone_state = DroneState.SEARCHING_LINE
                    self.turn_start_time = None
                    cmd_vel.angular.z = 0.0 # Stop turning
                else:
                    # Command constant yaw rate
                    cmd_vel.angular.z = self.turn_yaw_rate
                    # Ensure no linear motion during turn
                    cmd_vel.linear.x = 0.0
                    cmd_vel.linear.y = 0.0


        elif self.current_drone_state == DroneState.LINE_LOST:
            self.get_logger().warn("State: LINE_LOST - Stopping.")
            # Stop the drone
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.z = 0.0
            # Optional: Implement a search pattern here or transition to LANDING
            # For now, just stop and wait. Maybe revert to SEARCHING after a delay?


        # --- Safety Checks ---
        # Check if image feed is stale
        time_since_last_image = (self.get_clock().now() - self.last_image_time).nanoseconds / 1e9
        if time_since_last_image > 2.0 and self.current_drone_state not in [DroneState.INITIALIZING, DroneState.WAITING_FOR_ARM]:
            self.get_logger().error(f"Stale image feed ({time_since_last_image:.1f}s). Stopping.")
            cmd_vel = Twist() # Zero velocity
            # Consider changing state to LINE_LOST or a dedicated failsafe state

        # Ensure we are in OFFBOARD mode and armed when trying to move
        if self.current_drone_state not in [DroneState.INITIALIZING, DroneState.WAITING_FOR_ARM]:
             if not self.current_mavros_state.armed or self.current_mavros_state.mode != "OFFBOARD":
                  self.get_logger().warn(f"Not armed or not in OFFBOARD mode (State: {self.current_mavros_state.mode}, Armed: {self.current_mavros_state.armed}). Commanding zero velocity.")
                  cmd_vel = Twist() # Zero velocity
                  # Consider transitioning back to WAITING_FOR_ARM or INITIALIZING
                  if not self.current_mavros_state.armed:
                       self.current_drone_state = DroneState.WAITING_FOR_ARM
                  else: # Not in offboard
                      self.set_mode("OFFBOARD") # Try to re-enable offboard


        # --- Publish Commands ---
        # Only publish if MAVROS is connected
        if self.current_mavros_state.connected:
            self.vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = DroneLineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
        # Attempt to disarm on shutdown
        node.disarm_vehicle()
        # Set velocity to zero
        node.vel_pub.publish(Twist())

    finally:
        # Ensure node is destroyed and shutdown is called
        if rclpy.ok():
            if node:
                 node.vel_pub.publish(Twist()) # Final zero command
                 node.get_logger().info("Destroying node...")
                 node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
