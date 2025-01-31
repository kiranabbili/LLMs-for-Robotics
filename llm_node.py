#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from transformers import pipeline

class LLMNode:
    def __init__(self):
        rospy.init_node('llm_node', anonymous=True)

        # Initialize the LLM (using a text generation pipeline)
        self.llm = pipeline("text-generation", model="gpt2")

        # Subscribers and publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.text_input_sub = rospy.Subscriber('/text_input', String, self.text_input_callback)

    def text_input_callback(self, msg):
        # Process the input text using the LLM
        input_text = msg.data
        rospy.loginfo(f"Received input: {input_text}")

        # Generate a response using the LLM
        response = self.llm(input_text, max_length=50, num_return_sequences=1)[0]['generated_text']
        rospy.loginfo(f"LLM Response: {response}")

        # Perform actions based on the response
        self.execute_action(response)

    def execute_action(self, response):
        # Example: Simple command parsing
        if "move forward" in response.lower():
            self.move_robot(linear=0.2, angular=0.0)
        elif "turn left" in response.lower():
            self.move_robot(linear=0.0, angular=0.5)
        elif "turn right" in response.lower():
            self.move_robot(linear=0.0, angular=-0.5)
        elif "stop" in response.lower():
            self.move_robot(linear=0.0, angular=0.0)
        else:
            rospy.loginfo("No valid action detected.")

    def move_robot(self, linear, angular):
        # Publish velocity commands to TurtleBot3
        vel_cmd = Twist()
        vel_cmd.linear.x = linear
        vel_cmd.angular.z = angular
        self.cmd_vel_pub.publish(vel_cmd)

if __name__ == '__main__':
    try:
        llm_node = LLMNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
