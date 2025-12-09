---
sidebar_position: 2
title: rclpy + Python Agents in ROS 2
description: Learn to develop ROS 2 nodes using rclpy, the Python client library, transforming Python scripts into robot agents for modular and intelligent robotic behaviors.
keywords: [rclpy, ROS 2 Python, Python Agents, Robot Programming, Publisher, Subscriber, ROS 2 Nodes]
---

# rclpy + Python Agents in ROS 2

While ROS 2 supports multiple programming languages, Python (via `rclpy`) is often a preferred choice for developing robot applications due to its readability, extensive libraries, and rapid prototyping capabilities. This lesson will introduce you to `rclpy` and how to create simple Python-based ROS 2 nodes, effectively turning your Python scripts into robot agents.

## Introduction to `rclpy`

`rclpy` is the Python client library for ROS 2. It provides a straightforward API to interact with the core ROS 2 concepts we discussed: creating nodes, publishing to topics, subscribing to topics, and implementing services.

### Key Features of `rclpy`

*   **Ease of Use**: Python's syntax makes it easy to quickly write and test ROS 2 nodes.
*   **Integration with Python Ecosystem**: Leverage powerful Python libraries for AI, data processing, computer vision, etc.
*   **Asynchronous Programming**: `rclpy` supports Python's `asyncio` for non-blocking operations, which is crucial for responsive robot behavior.

## Creating a Simple ROS 2 Python Node

Let's create a basic publisher and subscriber node in Python.

### 1. Setup Your ROS 2 Workspace

Ensure you have a ROS 2 workspace (e.g., `ros2_ws`) set up and sourced. If not, follow the official ROS 2 installation and workspace tutorial.

```bash
# Example: Create a new ROS 2 package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_python_pkg
cd my_python_pkg
```

### 2. The Publisher Node (`simple_publisher.py`)

This node will continuously publish a string message to a topic.

```python
# ~/ros2_ws/src/my_python_pkg/my_python_pkg/simple_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Standard message type for strings

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher_node') # Node name
        self.publisher_ = self.create_publisher(String, 'my_topic', 10) # Topic name, QoS profile
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"') # Log to console
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher) # Keep node alive until Ctrl+C
    simple_publisher.destroy_node()
    rclpy.shutdown() # Shutdown ROS 2

if __name__ == '__main__':
    main()
```

### 3. The Subscriber Node (`simple_subscriber.py`)

This node will listen to the topic `my_topic` and print any messages it receives.

```python
# ~/ros2_ws/src/my_python_pkg/my_python_pkg/simple_subscriber.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'my_topic', # Must match publisher's topic name
            self.listener_callback,
            10) # QoS profile
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Update `setup.py`

For ROS 2 to find your Python nodes, you need to declare them in `setup.py` within your package.

```python
# ~/ros2_ws/src/my_python_pkg/setup.py

from setuptools import find_packages, setup

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_python_pkg.simple_publisher:main',
            'simple_subscriber = my_python_pkg.simple_subscriber:main',
        ],
    },
)
```

### 5. Build and Run

```bash
# From your workspace root (~/ros2_ws)
colcon build --packages-select my_python_pkg
source install/setup.bash # Source your workspace

# Run the publisher in one terminal
ros2 run my_python_pkg simple_publisher

# Run the subscriber in another terminal
ros2 run my_python_pkg simple_subscriber
```

You should see the subscriber receiving and printing the messages published by the publisher. This demonstrates the fundamental publisher-subscriber communication using `rclpy`.

## Python Agents and Behaviors

The nodes you create in `rclpy` act as "agents" within your robot system. Each agent can embody specific behaviors or functionalities:

*   **Perception Agents**: Nodes that process sensor data (e.g., an object detection agent, a facial recognition agent).
*   **Control Agents**: Nodes that send commands to actuators (e.g., a locomotion agent, a manipulation agent).
*   **Decision-Making Agents**: Nodes that implement higher-level AI logic (e.g., a navigation agent, a task planning agent).

By combining these specialized Python agents and letting them communicate via ROS 2 topics and services, you can build sophisticated and intelligent robot behaviors.

In the next lesson, we will explore how to represent the physical structure of humanoid robots using URDF (Unified Robot Description Format) files, which are essential for simulation and control.
