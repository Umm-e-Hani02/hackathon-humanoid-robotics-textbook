---
sidebar_position: 4
title: Hands-on - Control a Simulated Joint via ROS 2 Nodes
description: A hands-on exercise to create ROS 2 Python nodes using rclpy to publish commands and control a simulated robot joint.
keywords: [ROS 2, rclpy, Python, Hands-on, Robot Control, Simulated Joint, Publisher, Subscriber]
---

# Hands-on: Control a Simulated Joint via ROS 2 Nodes

In this exercise, you will create a simple ROS 2 Python node that publishes commands to control a simulated robot joint. This builds upon your understanding of ROS 2 nodes, topics, and `rclpy`.

## Prerequisites

*   A working ROS 2 environment (e.g., Foxy, Galactic, Humble, Iron)
*   Basic familiarity with Python and the command line
*   Understanding of ROS 2 concepts (Nodes, Topics) from previous lessons

## Goal

To create a ROS 2 publisher node in Python that sends target joint positions to a simulated joint and observe the joint's movement.

## Setup

For simplicity, we will simulate a joint in this exercise. In a full simulation (e.g., Gazebo), the joint controller would subscribe to a topic and move the actual joint. Here, we'll create a basic Python script that "receives" and "displays" the joint command.

1.  **Create a ROS 2 Package (if you haven't already)**:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python my_robot_control
    cd my_robot_control
    ```

2.  **Create the Joint Controller Subscriber (`joint_subscriber.py`)**:
    This script will act as our simulated joint. It will subscribe to a topic and print the received joint command.

    ```python
    # ~/ros2_ws/src/my_robot_control/my_robot_control/joint_subscriber.py

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64 # For joint position commands

    class JointSubscriber(Node):
        def __init__(self):
            super().__init__('joint_subscriber_node')
            self.subscription = self.create_subscription(
                Float64,
                '/joint_commands', # Topic for joint commands
                self.joint_command_callback,
                10)
            self.get_logger().info('Joint Subscriber Node started. Waiting for commands...')

        def joint_command_callback(self, msg):
            self.get_logger().info(f'Received joint command: {msg.data:.2f} radians')
            # In a real robot, this value would be sent to a motor controller
            # For this exercise, we just print it to simulate the effect.
            print(f"--- SIMULATED JOINT MOVEMENT ---")
            print(f"Joint moving to position: {msg.data:.2f} radians")
            print(f"----------------------------------")

    def main(args=None):
        rclpy.init(args=args)
        joint_subscriber = JointSubscriber()
        rclpy.spin(joint_subscriber)
        joint_subscriber.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

3.  **Create the Joint Publisher (`joint_publisher.py`)**:
    This script will publish a sequence of joint positions to control our simulated joint.

    ```python
    # ~/ros2_ws/src/my_robot_control/my_robot_control/joint_publisher.py

        import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64
    import time

    class JointPublisher(Node):
        def __init__(self):
            super().__init__('joint_publisher_node')
            self.publisher_ = self.create_publisher(Float64, '/joint_commands', 10)
            self.joint_positions = [0.0, 0.5, -0.5, 0.0, 1.0, -1.0, 0.0] # Radians
            self.current_position_index = 0
            self.timer = self.create_timer(2.0, self.timer_callback) # Publish every 2 seconds
            self.get_logger().info('Joint Publisher Node started.')

        def timer_callback(self):
            if self.current_position_index < len(self.joint_positions):
                msg = Float64()
                msg.data = self.joint_positions[self.current_position_index]
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing joint command: {msg.data:.2f} radians')
                self.current_position_index += 1
            else:
                self.get_logger().info('Finished publishing all joint commands. Shutting down.')
                self.destroy_node()
                rclpy.shutdown()

    def main(args=None):
        rclpy.init(args=args)
        joint_publisher = JointPublisher()
        rclpy.spin(joint_publisher)
        # Note: rclpy.spin() will block until the node is destroyed or ROS 2 is shut down.
        # The timer_callback handles the shutdown in this example.

    if __name__ == '__main__':
        main()
    ```

4.  **Update `setup.py`**:
    Add the new Python executables to your `my_robot_control` package's `setup.py`.

    ```python
    # ~/ros2_ws/src/my_robot_control/setup.py

    from setuptools import find_packages, setup

    package_name = 'my_robot_control'

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
        description='ROS 2 package for controlling a simulated robot joint.',
        license='Apache License 2.0',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'joint_publisher = my_robot_control.joint_publisher:main',
                'joint_subscriber = my_robot_control.joint_subscriber:main',
            ],
        },
    )
    ```

5.  **Build Your Package**:
    From your ROS 2 workspace root (`~/ros2_ws`), build the package:
    ```bash
    colcon build --packages-select my_robot_control
    ```

6.  **Source Your Workspace**:
    ```bash
    source install/setup.bash
    ```

## Running the Exercise

Open two separate terminal windows, navigate to your ROS 2 workspace (`~/ros2_ws`), and source it in both.

**Terminal 1: Start the Joint Subscriber (Simulated Joint)**
```bash
ros2 run my_robot_control joint_subscriber
```
This terminal will show messages as the joint receives commands.

**Terminal 2: Start the Joint Publisher (Controller)**
```bash
ros2 run my_robot_control joint_publisher
```
This terminal will send a sequence of joint position commands. Observe how the `joint_subscriber` in Terminal 1 reacts.

## Reflection

*   How does the publisher-subscriber model facilitate modular control?
*   What would be the next steps to integrate this into a full robot simulation (e.g., Gazebo)?
*   How would you add error handling or feedback mechanisms (e.g., current joint position) to this system?

This exercise provides a foundational understanding of how ROS 2 nodes can be used to command and monitor robot joints, a critical step towards controlling complex humanoid movements.

## Debugging Tips

*   **`colcon build` Issues**: If `colcon build` fails, check the error messages carefully. Common issues include Python syntax errors, missing dependencies in `package.xml` or `setup.py`, or incorrect `entry_points` in `setup.py`.
*   **Node Not Found**: If `ros2 run` fails with "package not found" or "executable not found", ensure:
    *   You have built your workspace (`colcon build`).
    *   You have sourced your workspace (`source install/setup.bash`).
    *   The `entry_points` in `setup.py` are correct and match your Python script names.
*   **No Messages Received**: If the subscriber isn't receiving messages:
    *   **Topic Name Mismatch**: Double-check that the publisher and subscriber are using the *exact same topic name* (`/joint_commands` in this case).
    *   **QoS Profile Mismatch**: While `10` is a common default, ensure the QoS profiles (`depth` parameter in `create_publisher` and `create_subscription`) are compatible.
    *   **Publisher Not Running**: Ensure the publisher node is running in its own terminal.
    *   **ROS 2 Daemon**: Sometimes restarting the ROS 2 daemon can help if communication issues persist: `ros2 daemon stop` then `ros2 daemon start`.
*   **Python Errors in Node**: If a node crashes, the terminal running that node will usually show a Python traceback. Read it from the bottom up to pinpoint the error.
*   **"Shutting down" message from publisher**: The publisher in this example is designed to shut down after publishing all its commands. If you want it to publish indefinitely, remove the `if self.current_position_index < len(self.joint_positions):` block and the `self.destroy_node()` and `rclpy.shutdown()` calls from the `else` block.
