#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import base64

# Gets the webcam feed from OpenCV and publishes it to the perpendicular_cam topic

class PerpendicularWebcamPublisher(Node):
    def __init__(self):
        super().__init__('perpendicular_webcam_publisher')
        self.publisher_ = self.create_publisher(String, '/perpendicular_cam', 10)
        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        # Open the default webcam (device index 0)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Webcam could not be opened.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame from webcam")
            return
        
        # define the resolution her
        # Optionally, you may resize the frame
        frame = cv2.resize(frame, (256, 256))
        # Encode the frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            self.get_logger().error("Failed to encode frame")
            return
        # Convert the JPEG buffer to a base64 encoded string
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
        msg = String()
        msg.data = jpg_as_text
        self.publisher_.publish(msg)
        self.get_logger().info("Published a webcam frame.")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
