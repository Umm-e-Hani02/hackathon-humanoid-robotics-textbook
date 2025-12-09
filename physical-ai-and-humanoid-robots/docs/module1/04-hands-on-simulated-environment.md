---
sidebar_position: 4
title: Hands-on Simulated Robot Environment
description: A hands-on exercise to explore the basics of simulated robot environments, focusing on robot perception and action in a virtual world.
keywords: [Simulated Robot, Hands-on, Robotics Simulation, Python, Perception, Action, Embodied AI]
---
# Hands-on: Explore a Basic Simulated Robot Environment

This exercise will guide you through setting up and exploring a very basic simulated robot environment. The goal is to familiarize yourself with how a robot perceives its virtual surroundings and how its actions can influence that environment.

## Prerequisites

*   A computer with Python installed (version 3.8 or higher).
*   A basic understanding of the command line.

## Setup: Installing a Minimal Simulation

We'll use a simple Python library for this exercise to keep the focus on the concepts rather than complex installation. For more advanced simulations (Gazebo, Unity), we will cover those in later modules.

1.  **Create a Project Directory**:
    Open your terminal or command prompt and create a new folder for this exercise:
    ```bash
    mkdir robot_sim_exercise
    cd robot_sim_exercise
    ```

2.  **Create a Virtual Environment** (Recommended):
    ```bash
    python -m venv venv
    # On Windows:
    # .\venv\Scripts\activate
    # On macOS/Linux:
    # source venv/bin/activate
    ```

3.  **Install a Simple Simulation Library**:
    We'll use a hypothetical `simple_robot_sim` library for demonstration. In a real scenario, you might install `pybullet`, `gymnasium`, or similar. For this exercise, we will simulate its presence.

## Exercise: Interacting with the Simulated Environment

Let's imagine our `simple_robot_sim` library provides an API to create a robot and an environment.

1.  **Create a Python File**:
    Create a file named `explore_robot.py` in your `robot_sim_exercise` directory:
    ```python
    # explore_robot.py

    # --- Hypothetical simple_robot_sim library usage ---
    # In a real scenario, this would be imported and used.
    # For this exercise, we'll simulate the interaction.

    class SimulatedRobot:
        def __init__(self):
            self.position = [0.0, 0.0]  # x, y coordinates
            self.orientation = 0.0      # angle in radians
            self.sensors = {
                "front_distance": 100,  # distance to nearest object in front
                "left_distance": 100,
                "right_distance": 100,
                "touch_sensor": False
            }
            print("Robot initialized at (0,0), facing 0 degrees.")

        def move_forward(self, distance):
            self.position[0] += distance * math.cos(self.orientation)
            self.position[1] += distance * math.sin(self.orientation)
            print(f"Moved forward by {distance}. New position: {self.position}")
            self._update_sensors()

        def turn_left(self, angle_deg):
            self.orientation += math.radians(angle_deg)
            print(f"Turned left by {angle_deg} degrees. New orientation: {math.degrees(self.orientation):.2f} degrees.")
            self._update_sensors()

        def turn_right(self, angle_deg):
            self.orientation -= math.radians(angle_deg)
            print(f"Turned right by {angle_deg} degrees. New orientation: {math.degrees(self.orientation):.2f} degrees.")
            self._update_sensors()

        def get_sensor_readings(self):
            return self.sensors

        def _update_sensors(self):
            # In a real simulation, this would calculate distances based on environment
            # For this simplified exercise, we'll just simulate some changes
            import random
            self.sensors["front_distance"] = random.randint(10, 150)
            self.sensors["left_distance"] = random.randint(10, 150)
            self.sensors["right_distance"] = random.randint(10, 150)
            self.sensors["touch_sensor"] = random.choice([True, False, False, False]) # 25% chance of touch

    import math
    import time

    def main():
        robot = SimulatedRobot()

        print("\n--- Initial State ---")
        print(f"Robot Position: {robot.position}")
        print(f"Robot Orientation: {math.degrees(robot.orientation):.2f} degrees")
        print(f"Sensor Readings: {robot.get_sensor_readings()}")

        print("\n--- Performing Actions ---")
        robot.move_forward(5)
        time.sleep(0.5)
        print(f"Sensor Readings after move: {robot.get_sensor_readings()}")

        robot.turn_left(45)
        time.sleep(0.5)
        print(f"Sensor Readings after turn: {robot.get_sensor_readings()}")

        robot.move_forward(3)
        time.sleep(0.5)
        print(f"Sensor Readings after second move: {robot.get_sensor_readings()}")

        if robot.get_sensor_readings()["touch_sensor"]:
            print("\nRobot detected a touch! Reacting by backing up.")
            robot.move_forward(-1) # Move backward
        else:
            print("\nNo touch detected.")

        print("\n--- Final State ---")
        print(f"Robot Position: {robot.position}")
        print(f"Robot Orientation: {math.degrees(robot.orientation):.2f} degrees")
        print(f"Final Sensor Readings: {robot.get_sensor_readings()}")

    if __name__ == "__main__":
        main()
    ```

2.  **Run the Simulation**:
    Save the `explore_robot.py` file and run it from your terminal:
    ```bash
    python explore_robot.py
    ```

## Questions for Reflection

After running the simulation, consider the following:

*   How did the robot's position and orientation change after each action?
*   How did the sensor readings vary? What does this tell you about the robot's "perception" of its environment?
*   Imagine adding a visual sensor (camera). How would the robot's understanding of its environment change?
*   What are the limitations of this very simple simulation compared to a real-world robot?

This basic exercise illustrates the fundamental loop of **perception -> action -> feedback** that is central to embodied intelligence and Physical AI. In more advanced simulations, these concepts become much more sophisticated, involving complex physics engines, detailed sensor models, and realistic environments.

## Debugging Tips

*   **Syntax Errors**: If your Python script doesn't run, double-check for typos, missing colons, or incorrect indentation. Python is very sensitive to these.
*   **Module Not Found**: Ensure you have activated your virtual environment before running `python explore_robot.py`.
*   **Unexpected Output**: Review the `print` statements in the `SimulatedRobot` class and `main` function. Trace the robot's position, orientation, and sensor readings step-by-step to understand the flow.
*   **Randomness**: Since the sensor readings are randomized, you might get different outputs each time. This is normal for simulating real-world variations.
