#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def text_publisher():
    # Initialize the node
    rospy.init_node('text_publisher', anonymous=True)

    # Create a publisher for the /text_input topic
    pub = rospy.Publisher('/text_input', String, queue_size=10)

    while not rospy.is_shutdown():
        # Get user input
        message = input("Enter a command: ")

        # Publish the message
        rospy.loginfo(f"Publishing: {message}")
        pub.publish(message)

if __name__ == '__main__':
    try:
        text_publisher()
    except rospy.ROSInterruptException:
        pass
