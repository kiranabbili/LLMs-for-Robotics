#!/usr/bin/env python3
"""
ROSA + LLM Controller for TurtleBot3 Simulation (ROS 2 Humble)
Author: Kiran Kumar
Description:
  Use an LLM (Anthropic Claude or Ollama) to control a TurtleBot3 in Gazebo simulation.
  Supports navigation, velocity control, stopping, and map saving.

Usage:
  1. Start TurtleBot3 Gazebo + Nav2:
       export TURTLEBOT3_MODEL=waffle_pi
       ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
       ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

  2. Run this script:
       python3 rosa_turtlebot3.py

  3. Example Commands:
       navigate to kitchen
       go to x: 1.0 y: 0.5
       move forward
       stop
       save map my_home
"""

from langchain_anthropic import ChatAnthropic
from langchain_ollama import ChatOllama
from langchain.agents import tool
from rosa import ROSA
from rosa.prompts import RobotSystemPrompts
import os
import pathlib
import time
import subprocess
from typing import Tuple
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

node = None
vel_publisher = None
navigate_to_pose_action_client = None


# =============================
# Utility Functions
# =============================
def execute_ros_command(command: str) -> Tuple[bool, str]:
    """Execute a ROS2 command and return success + output."""
    if not command.startswith("ros2 "):
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    try:
        output = subprocess.check_output(command, shell=True).decode()
        return True, output
    except Exception as e:
        return False, str(e)


def _get_maps_dir() -> str:
    """Ensure and return path to 'maps' directory."""
    maps_dir = os.path.expanduser("~/turtlebot3_maps")
    pathlib.Path(maps_dir).mkdir(parents=True, exist_ok=True)
    return maps_dir


# =============================
# Predefined Locations (Gazebo)
# =============================
LOCATIONS = {
    "home": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"z": 0.0, "w": 1.0}},
    "kitchen": {"position": {"x": 1.5, "y": 0.5, "z": 0.0}, "orientation": {"z": 0.0, "w": 1.0}},
    "living room": {"position": {"x": -1.0, "y": 0.5, "z": 0.0}, "orientation": {"z": 0.0, "w": 1.0}},
    "office": {"position": {"x": 0.5, "y": -1.0, "z": 0.0}, "orientation": {"z": 0.0, "w": 1.0}},
}


# =============================
# ROS Tools (for LLM)
# =============================

@tool
def send_vel(velocity: float) -> str:
    """Move the TurtleBot3 forward/backward."""
    global vel_publisher
    twist = Twist()
    twist.linear.x = velocity
    vel_publisher.publish(twist)
    return f"Velocity set to {velocity:.2f} m/s"


@tool
def stop() -> str:
    """Stop the TurtleBot3."""
    global vel_publisher
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    vel_publisher.publish(twist)
    return "Robot stopped."


@tool
def navigate_to_pose(x: float, y: float, z_orientation: float, w_orientation: float) -> str:
    """Navigate the TurtleBot3 to a specific coordinate on the map."""
    global navigate_to_pose_action_client, node
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.orientation.z = z_orientation
    goal_msg.pose.pose.orientation.w = w_orientation

    navigate_to_pose_action_client.send_goal_async(goal_msg)
    return f"Navigation goal sent to (x={x:.2f}, y={y:.2f})."


@tool
def save_map(map_name: str) -> str:
    """Save the current map to ~/turtlebot3_maps."""
    maps_dir = _get_maps_dir()
    filepath_prefix = os.path.join(maps_dir, map_name)
    cmd = f"ros2 run nav2_map_server map_saver_cli -f '{filepath_prefix}'"
    success, output = execute_ros_command(cmd)
    if success:
        return f"Map '{map_name}' saved in {maps_dir}"
    return f"Failed to save map: {output}"


@tool
def list_saved_maps() -> str:
    """List saved maps."""
    maps_dir = _get_maps_dir()
    files = [f[:-5] for f in os.listdir(maps_dir) if f.endswith(".yaml")]
    return f"Available maps: {', '.join(files) if files else 'No maps found.'}"


@tool
def get_location_names() -> str:
    """Return all predefined location names."""
    return f"Available locations: {', '.join(LOCATIONS.keys())}"


@tool
def navigate_to_location_by_name(location_name: str) -> str:
    """Navigate to a predefined location."""
    global navigate_to_pose_action_client, node
    loc = LOCATIONS.get(location_name.lower())
    if not loc:
        return f"Location '{location_name}' not found. Try: {', '.join(LOCATIONS.keys())}"
    pos, orient = loc["position"], loc["orientation"]

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = pos["x"]
    goal_msg.pose.pose.position.y = pos["y"]
    goal_msg.pose.pose.orientation.z = orient["z"]
    goal_msg.pose.pose.orientation.w = orient["w"]

    navigate_to_pose_action_client.send_goal_async(goal_msg)
    return f"Navigating to {location_name}..."


# =============================
# Main Entry
# =============================

def main():
    global node, vel_publisher, navigate_to_pose_action_client
    print("🤖 Hi from ROSA TurtleBot3 Controller!")

    # --- Initialize ROS2 ---
    rclpy.init()
    sim_time_param = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
    node = rclpy.create_node("turtlebot3_rosa_node", parameter_overrides=[sim_time_param])

    vel_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    navigate_to_pose_action_client = ActionClient(node, NavigateToPose, "/navigate_to_pose")

    # --- Initialize LLM ---
    try:
        user_name = os.getenv("USER")
        if user_name == "ros":
            print("Using remote Ollama instance...")
            llm = ChatOllama(
                model="hhao/qwen2.5-coder-tools:latest",
                temperature=0,
                base_url="http://160.85.252.236:11434",
                num_ctx=32192,
            )
        else:
            print("Using Anthropic API with Claude 3.5 Sonnet...")
            api_key=os.getenv("ANTHROPIC_API_KEY")
            if not api_key:
                raise ValueError("ANTHROPIC_API_KEY is not set in environment variables. Use `export ANTHROPIC_API_KEY=your_key`.")
            llm = ChatAnthropic(
                model="claude-3-5-sonnet-20240620",
                temperature=0,
                anthropic_api_key=api_key,
                max_tokens=4096,
            )
    except Exception as e:
        print(f"LLM initialization failed: {e}")
        return

    prompt = RobotSystemPrompts()
    prompt.embodiment = (
        "You are Summit, a helpful TurtleBot3 robot assistant in simulation. "
        "You can move, navigate, stop, and save maps using ROS 2 tools."
    )

    agent = ROSA(
        ros_version=2,
        llm=llm,
        tools=[
            send_vel,
            stop,
            navigate_to_pose,
            save_map,
            list_saved_maps,
            get_location_names,
            navigate_to_location_by_name,
        ],
        prompts=prompt,
    )

    print("🧠 Type natural commands (e.g., 'navigate to kitchen', 'move forward') or 'exit' to quit.")

    try:
        while True:
            msg = input("\nCommand ➜ ")
            if msg.lower() in ["exit", "quit"]:
                break
            print("Processing...")
            try:
                res = agent.invoke(msg)[0]
                if isinstance(res, dict) and "text" in res:
                    print(res["text"])
                else:
                    print(res)
            except Exception as e:
                print(f"Error during request: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    agent.shutdown()
    print("👋 Shutting down ROSA TurtleBot3 Controller.")


if __name__ == "__main__":
    main()
