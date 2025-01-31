# LLMs-for-Robotics
LLM frame work for Turtlebot3

Steps to setup LLM for Turtlebot3
1. Install TurtleBot3 packages:
   
sudo apt-get install ros-<ros_distro>-turtlebot3-*

3. Set up the TurtleBot3 simulation in Gazebo:
   
export TURTLEBOT3_MODEL=burger

roslaunch turtlebot3_gazebo turtlebot3_world.launch

----Select an LLM framework that suits your needs. Popular options include:

OpenAI GPT (e.g., GPT-4): Requires API access and internet connectivity.

Hugging Face Transformers: Open-source models like GPT-2, GPT-Neo, or BERT that can be run locally.

LLaMA (Meta): Lightweight and efficient models for edge devices----

For simplicity, we'll use Hugging Face Transformers in this guide.

3. Install the necessary Python libraries for LLMs and ROS integration:
   
     pip install transformers torch rospy
   
5. Create a ROS node that interacts with the LLM and TurtleBot3. This node will: llm_node.py
   
7. Make it executable: chmod +x llm_node.py
   
9. Run the node: rosrun your_package_name llm_node.py
    
11. Create a simple ROS node that publishes messages to the /text_input topic. This node will be: text_publisher.py

12. Make it executable, and run it:
   
     chmod +x text_publisher.py
   
     rosrun your_package_name text_publisher.py

   
