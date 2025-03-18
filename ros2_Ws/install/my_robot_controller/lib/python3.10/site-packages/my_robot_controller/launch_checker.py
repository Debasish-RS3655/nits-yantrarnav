#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# subscribes to the important launch topics and publishes true when all topics are active

class LaunchTopicMonitor(Node):
    def __init__(self):
        super().__init__('launch_topic_monitor')

        # List of topics to subscribe to.
        self.topic_list = ['/delayed_topic1', '/delayed_topic2']
        
        self.displayed = False

        # Dictionary to track if each topic has received a message in the current cycle.
        self.active_flags = {topic: False for topic in self.topic_list}

        # Create a subscriber for each topic.
        for topic in self.topic_list:
            self.create_subscription(
                String,
                topic,
                lambda msg, t=topic: self.topic_callback(msg, t),
                10)
            self.get_logger().info(f"Subscribed to {topic}")

        # Publisher for the output topic.
        self.publisher = self.create_publisher(Bool, '/system_launch_status', 10)

        # Timer to periodically check the active status.
        self.timer = self.create_timer(1.0, self.state_updater)

    def topic_callback(self, msg, topic):
        # Mark the topic as active when a message is received.
        self.active_flags[topic] = True
        # self.get_logger().info(f"Received from {topic}: {msg.data}")

    def state_updater(self):
        # Check the active status for all topics.
        if all(self.active_flags.values()):
            out_msg = Bool(data=True)
            # display the launch message
            if not self.displayed:
                self.get_logger().info("All launch topics have started. System is ready.")
                self.displayed = True
        else:
            out_msg = Bool(data=False)
            # self.get_logger().info("At least one topic inactive: publishing False")

        self.publisher.publish(out_msg)
        
        # Reset active flags for the next cycle.
        for topic in self.active_flags:
            self.active_flags[topic] = False

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
