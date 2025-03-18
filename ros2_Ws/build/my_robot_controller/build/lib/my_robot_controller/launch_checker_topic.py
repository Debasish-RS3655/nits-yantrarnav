#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class LaunchTopicMonitor(Node):
    def __init__(self):
        super().__init__('launch_topic_monitor')

        # List of required topics to check for.
        self.topic_list = ['/delayed_topic1', '/delayed_topic2']
        
        self.displayed = False

        # Publisher for the system launch status.
        self.publisher = self.create_publisher(Bool, '/system_launch_status', 10)

        # Timer to periodically check if all required topics exist.
        self.timer = self.create_timer(1.0, self.state_updater)

    def state_updater(self):
        # Get the list of all currently active topics.
        topics_and_types = self.get_topic_names_and_types()
        # Extract the topic names from the returned list of tuples.
        existing_topics = [topic for topic, types in topics_and_types]

        # Check if all required topics exist.
        if all(topic in existing_topics for topic in self.topic_list):
            out_msg = Bool(data=True)
            if not self.displayed:
                self.get_logger().info("All required topics are available. System is ready.")
                self.displayed = True
        else:
            out_msg = Bool(data=False)

        self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaunchTopicMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
