#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from transformers import pipeline
import time
import re

class LLMNode:
    def __init__(self):
        rospy.init_node('llm_node', anonymous=True)

        # Initialize the LLM (using a text generation pipeline)
        self.llm = pipeline("text-generation", model="gpt2")

        # Subscribers and publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.text_input_sub = rospy.Subscriber('/text_input', String, self.text_input_callback)

    def text_input_callback(self, msg):
        input_text = msg.data
        rospy.loginfo(f"Received input: {input_text}")

        # Generate a response using the LLM
        response = self.llm(input_text, max_length=50, num_return_sequences=1)[0]['generated_text']
        rospy.loginfo(f"LLM Response: {response}")

        # Perform actions based on the response
        self.execute_action(response)

    def execute_action(self, response):
        # Check for movement commands with specific distance/angle
        move_match = re.search(r"move (\d+(?:\.\d+)?) meters?", response, re.IGNORECASE)
        turn_match = re.search(r"turn (left|right) (\d+(?:\.\d+)?) degrees?", response, re.IGNORECASE)

        if move_match:
            distance = float(move_match.group(1))
            self.move_distance(distance, speed=0.2)
        elif turn_match:
            direction = turn_match.group(1).lower()
            angle = float(turn_match.group(2))
            self.turn_angle(angle, speed=0.5 if direction == 'left' else -0.5)
        elif "stop" in response.lower():
            self.move_robot(0.0, 0.0)
        else:
            rospy.loginfo("No valid action detected.")

    def move_distance(self, distance, speed):
        duration = distance / speed  # Time to move given speed
        self.move_robot(speed, 0.0)
        time.sleep(duration)
        self.move_robot(0.0, 0.0)

    def turn_angle(self, angle, speed):
        angular_speed = abs(speed)
        duration = (angle / 360.0) * (2 * 3.1416 / angular_speed)  # Approximation
        self.move_robot(0.0, speed)
        time.sleep(duration)
        self.move_robot(0.0, 0.0)

    def move_robot(self, linear, angular):
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
