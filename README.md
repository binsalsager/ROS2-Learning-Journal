🤖 Sager's ROS 2 Learning Journal 🚀
Welcome to my personal knowledge base and public record of my journey learning the Robot Operating System 2 (ROS 2). As a B.Tech student in Robotics and Automation, my goal is to master the tools required to build the future of robotics. This repository documents my progress, one commit at a time.

📜 Table of Contents
Core Concepts

Workspace Commands

Python Node Structure

Design Patterns

📝 Entry Date: 2025-09-10
Today's Focus: Deep dive into the fundamentals of a ROS 2 Publisher node.

🧠 1. Core Concepts (The Physics)
Class: The blueprint or schematic. It defines what an object is and what it can do.

Object (Instance): The actual, working thing built from the blueprint. It has its own independent variables and state.

Node: A single, running ROS 2 program. In code, it's an object created from a Node class.

Topic: A named channel for messages, like a public radio frequency (e.g., /chatter).

Publisher: A node that sends messages to a Topic.

Subscriber: A node that listens for messages from a Topic.

Callback: A function that runs automatically when a specific event happens (e.g., a timer ticks or a message arrives).

⌨️ 2. Essential Workspace Commands
mkdir -p ws_name/src

Creates a new workspace directory and the essential src subfolder.

ros2 pkg create <package_name> --build-type ament_python

Creates a new Python package inside the src folder.

colcon build

Compiles your workspace. Must be run from the workspace root (e.g., ~/bumperbot_ws).

source install/setup.bash

Makes the current terminal aware of your workspace's packages. Must be run in every new terminal.

ros2 pkg list

Lists all ROS 2 packages your terminal is aware of.

🏗️ 3. Python Node Structure (The Basic Blueprint)
# 1. Imports (The ingredients)
import rclpy
from rclpy.node import Node
# (Import the specific message types you need)

# 2. Class Definition (The blueprint)
class MyNode(Node):
    # 3. Constructor (The setup, runs once per object)
    def __init__(self):
        super().__init__('my_node_name')
        # ...Create publishers, subscribers, timers here...

    # 4. Callbacks / Methods (The actions)
    def my_callback(self):
        # ...The work happens here...

# 5. Main Execution (The "ON" switch for the program)
def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode() # <-- The Object is created here!
    rclpy.spin(my_node) # Keeps the node alive
    
    # Cleanup after Ctrl+C
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

📡 4. Design Patterns
Publisher Pattern (The Broadcaster Drone)
Goal: Repeatedly send messages to a topic at a fixed frequency.

Key Import:

from std_msgs.msg import String

Inside __init__(self):

# Create the publisher
# Format: self.create_publisher(MsgType, 'topic_name', QoS_queue_size)
self.publisher_ = self.create_publisher(String, 'chatter', 10)

# Create a timer to trigger the callback
timer_period = 0.5 # seconds
self.timer = self.create_timer(timer_period, self.publisher_callback)

The Callback Function:

def publisher_callback(self):
    msg = String()
    msg.data = "My message content"
    self.publisher_.publish(msg)
    self.get_logger().info(f'Publishing: "{msg.data}"')
