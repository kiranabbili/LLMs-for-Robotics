import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
import requests  # For making API calls to DeepSeek

class LLMNavigationNode(Node):
    def __init__(self):
        super().__init__('llm_navigation_node')
        self.get_logger().info("Initializing LLMNavigationNode...")
        
        # Publisher for velocity commands (fallback)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for odometry (robot's current position)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Subscriber for navigation commands
        self.command_subscription = self.create_subscription(String, '/command', self.command_callback, 10)
        
        # Subscriber for LIDAR data (obstacle detection)
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # Action client for Navigation2's NavigateToPose
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Robot's current pose
        self.current_pose = None
        
        # Obstacle detection flag
        self.obstacle_detected = False
        
        # Set up DeepSeek API key
        self.deepseek_api_key = "place your API Key"  # Replace with your actual API key
        self.deepseek_api_url = "https://api.deepseek.com/v1/chat/completions"  # Example endpoint (replace with actual URL)

    def odom_callback(self, msg):
        # Update the robot's current position
        self.current_pose = msg.pose.pose
        self.get_logger().info(f"Updated current pose: {self.current_pose}")

    def lidar_callback(self, msg):
        # Check for obstacles in the LIDAR data
        try:
            min_distance = min(msg.ranges)  # Find the closest obstacle
            if min_distance < 0.5:  # Threshold for obstacle detection (0.5 meters)
                self.obstacle_detected = True
                self.get_logger().warn("Obstacle detected! Stopping the robot.")
                self.stop_robot()
            else:
                self.obstacle_detected = False
        except Exception as e:
            self.get_logger().error(f"Error in lidar_callback: {e}")

    def stop_robot(self):
        # Stop the robot by sending zero velocity
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)
        self.get_logger().info("Robot stopped.")

    def command_callback(self, msg):
        # Process the navigation command
        try:
            user_command = msg.data
            self.get_logger().info(f'Received command: {user_command}')
            
            # Generate waypoints using the DeepSeek API
            waypoints = self.get_waypoints_from_deepseek(user_command)
            
            if waypoints:
                # Navigate to the first waypoint using Navigation2
                self.navigate_to_waypoint(waypoints[0])
        except Exception as e:
            self.get_logger().error(f"Error in command_callback: {e}")

    def get_waypoints_from_deepseek(self, user_command):
        # Call the DeepSeek API to generate waypoints
        try:
            headers = {
                "Authorization": f"Bearer {self.deepseek_api_key}",
                "Content-Type": "application/json"
            }
            data = {
                "model": "deepseek-chat",  # Replace with the correct model name
                "messages": [
                    {"role": "system", "content": "You are a helpful assistant that generates waypoints for robot navigation."},
                    {"role": "user", "content": f"Generate a list of waypoints (x, y coordinates) for the command: {user_command}. Format: [x1, y1], [x2, y2], ..."}
                ],
                "max_tokens": 50
            }
            response = requests.post(self.deepseek_api_url, headers=headers, json=data)
            response.raise_for_status()  # Raise an error for bad status codes
            waypoints_str = response.json()["choices"][0]["message"]["content"].strip()
            waypoints = eval(waypoints_str)  # Convert string to list of waypoints
            self.get_logger().info(f"Generated waypoints: {waypoints}")
            return waypoints
        except Exception as e:
            self.get_logger().error(f"Failed to generate waypoints: {e}")
            return None

    def navigate_to_waypoint(self, waypoint):
        # Create a PoseStamped message for the target waypoint
        try:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = waypoint[0]
            goal_pose.pose.position.y = waypoint[1]
            goal_pose.pose.orientation.w = 1.0  # Default orientation
            
            # Create a NavigateToPose goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose
            
            # Send the goal to the Navigation2 action server
            self.nav_to_pose_client.wait_for_server()
            self.send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
            self.send_goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info("Goal sent to Navigation2 server.")
        except Exception as e:
            self.get_logger().error(f"Error in navigate_to_waypoint: {e}")

    def goal_response_callback(self, future):
        # Handle the response from the Navigation2 action server
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal was rejected by the Navigation2 server.")
                return
            
            self.get_logger().info("Goal accepted! Navigating to the waypoint...")
            self.result_future = goal_handle.get_result_async()
            self.result_future.add_done_callback(self.goal_result_callback)
        except Exception as e:
            self.get_logger().error(f"Error in goal_response_callback: {e}")

    def goal_result_callback(self, future):
        # Handle the result of the navigation action
        try:
            result = future.result().result
            if result:
                self.get_logger().info("Navigation completed successfully!")
            else:
                self.get_logger().error("Navigation failed.")
        except Exception as e:
            self.get_logger().error(f"Error in goal_result_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        llm_navigation_node = LLMNavigationNode()
        rclpy.spin(llm_navigation_node)
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        llm_navigation_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
