#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from transformers import pipeline

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')

        # Initialize the LLM
        self.llm = pipeline("text-generation", model="distilgpt2")

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.text_input_sub = self.create_subscription(
            String, '/text_input', self.text_input_callback, 10)

    def text_input_callback(self, msg):
        # Process the input text using the LLM
        input_text = msg.data
        self.get_logger().info(f"Received input: {input_text}")

        # Generate a response using the LLM
        response = self.llm(input_text, max_length=50, num_return_sequences=1)[0]['generated_text']
        self.get_logger().info(f"LLM Response: {response}")

        # Perform actions based on the response
        self.execute_action(response)

    def execute_action(self, response):
        vel_cmd = Twist()
        if "move forward" in response.lower():
            vel_cmd.linear.x = 0.2  # Move forward
        elif "turn left" in response.lower():
            vel_cmd.angular.z = 0.5  # Turn left
        elif "turn right" in response.lower():
            vel_cmd.angular.z = -0.5  # Turn right
        elif "stop" in response.lower():
            vel_cmd.linear.x = 0.0  # Stop
            vel_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(vel_cmd)

def main(args=None):
    rclpy.init(args=args)
    llm_node = LLMNode()
    rclpy.spin(llm_node)
    llm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
