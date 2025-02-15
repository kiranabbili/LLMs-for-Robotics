#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from transformers import pipeline

class LLMNavNode(Node):
    def __init__(self):
        super().__init__('llm_nav_node')

        # Initialize the LLM
        self.llm = pipeline("text-generation", model="distilgpt2")

        # Predefined locations (x, y, theta)
        self.locations = {
            "move to location1": (1.3, 2.2, 0.0),
            "move to location2": (3.0, 2.1, 0.0),
            "move to location3": (3.3, 1.3, 0.0),
        }

        # Subscriber for text input
        self.text_input_sub = self.create_subscription(
            String, '/text_input', self.text_input_callback, 10)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def text_input_callback(self, msg):
        # Process the input text using the LLM
        input_text = msg.data
        self.get_logger().info(f"Received input: {input_text}")

        # Generate a response using the LLM
        response = self.llm(input_text, max_length=50, num_return_sequences=1)[0]['generated_text']
        self.get_logger().info(f"LLM Response: {response}")

        # Parse the response to get the goal location
        goal_location = self.parse_goal(response)
        if goal_location:
            self.send_goal(goal_location)

    def parse_goal(self, response):
        # Check if the response contains a predefined location
        for location, coordinates in self.locations.items():
            if location in response.lower():
                self.get_logger().info(f"Goal location found: {location}")
                return coordinates
        self.get_logger().error("Unknown goal location.")
        return None

    def send_goal(self, goal_location):
        # Create a goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = goal_location[0]
        goal_msg.pose.pose.position.y = goal_location[1]
        goal_msg.pose.pose.orientation.z = goal_location[2]

        # Send the goal to the Nav2 action server
        self.nav_client.wait_for_server()
        self.goal_handle = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.goal_handle.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Handle the response from the action server
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by the action server.")
            return

        self.get_logger().info("Goal accepted. Navigating to the goal...")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        # Handle the result of the navigation action
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached successfully!")
        else:
            self.get_logger().error("Failed to reach the goal. Status: {status}")

    def feedback_callback(self, feedback_msg):
        # Handle feedback from the Nav2 stack
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Distance to goal: {feedback.distance_remaining:.2f} meters")

def main(args=None):
    rclpy.init(args=args)
    llm_nav_node = LLMNavNode()
    rclpy.spin(llm_nav_node)
    llm_nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
