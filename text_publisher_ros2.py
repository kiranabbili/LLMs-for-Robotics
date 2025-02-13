#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TextPublisher(Node):
    def __init__(self):
        super().__init__('text_publisher')

        # Create a publisher for the /text_input topic
        self.publisher = self.create_publisher(String, '/text_input', 10)

        # Set the publishing rate (e.g., 1 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Get user input
        user_input = input("Enter a command (or 'q' to quit): ")

        if user_input.lower() == 'q':
            self.get_logger().info("Shutting down...")
            raise SystemExit

        # Create and publish the message
        msg = String()
        msg.data = user_input
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    text_publisher = TextPublisher()

    try:
        rclpy.spin(text_publisher)
    except SystemExit:
        pass  # Handle the shutdown gracefully
    finally:
        text_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
