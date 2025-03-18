import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DelayedPublisher(Node):
    def __init__(self):
        super().__init__('delayed_publisher1')
        self.publisher_ = self.create_publisher(String, '/delayed_topic1', 10)
        
        # Start a timer that triggers after 5 seconds.
        # Once the timer_callback is called, we cancel the timer to ensure one-shot behavior.
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info('Timer started: will publish message after 5 seconds.')

    def timer_callback(self):
        msg = String()
        msg.data = 'This is a delayed message.'
        self.publisher_.publish(msg)
        self.get_logger().info('Published delayed message to /delayed_topic1')
        
        # Cancel the timer so that the callback doesn't repeat.
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = DelayedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
