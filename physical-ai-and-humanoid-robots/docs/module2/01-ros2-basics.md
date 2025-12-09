---
sidebar_position: 1
title: ROS 2 Basics - Nodes, Topics, and Services
description: Learn the fundamental communication concepts in ROS 2, including Nodes, Topics, and Services, essential for building modular and scalable robotic applications.
keywords: [ROS 2, Nodes, Topics, Services, Robotics Communication, Publisher, Subscriber, Client, Server, Message]
---

# ROS 2 Basics: Nodes, Topics, and Services

The Robot Operating System (ROS) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. ROS 2 is the latest iteration, designed for enhanced real-time performance, security, and multi-robot system support.

At its core, ROS 2 facilitates communication between different parts of a robot's software system. This communication is built around a few fundamental concepts: **Nodes**, **Topics**, and **Services**.

## Nodes: The Workers of ROS 2

A **Node** is an executable process in ROS 2. It's typically responsible for performing a single, modular task within the robot's ecosystem. Think of nodes as individual programs or processes that do specific jobs, like controlling a motor, reading sensor data, or performing navigation algorithms.

*   **Modularity**: Breaking down a complex robot system into many smaller, independent nodes promotes modularity, making development, debugging, and maintenance easier.
*   **Encapsulation**: Each node encapsulates its own functionality and doesn't need to know the internal workings of other nodes.
*   **Language Agnostic**: Nodes can be written in different programming languages (e.g., Python, C++) and still communicate seamlessly.

**Example**:
*   A `camera_driver` node that publishes image data.
*   A `motor_controller` node that subscribes to movement commands and publishes motor status.
*   A `path_planner` node that takes map data and a goal, and publishes a path.

## Topics: The Data Highways

**Topics** are the primary means of asynchronous, many-to-many, one-way communication in ROS 2. Nodes publish data to topics, and other nodes subscribe to those topics to receive the data. Data published to a topic is a **message**, which is a structured data type.

*   **Publisher/Subscriber Model**: A node that sends data is a **publisher**. A node that receives data is a **subscriber**.
*   **Loose Coupling**: Publishers and subscribers don't need to know about each other directly. They only need to agree on the topic name and the message type.
*   **Real-time Streams**: Topics are ideal for continuous streams of data, such as sensor readings, video feeds, or robot odometry.

**Analogy**: Think of topics like radio stations. A radio station (topic) broadcasts information. Anyone can tune into that station (subscribe) to receive the broadcast. Multiple stations (topics) can exist, and multiple listeners (subscribers) can tune into the same station, while multiple broadcasters (publishers) can contribute to a single station.

**Example**:
*   A `camera_driver` node **publishes** `sensor_msgs/Image` messages to the `/image_raw` topic.
*   An `image_processor` node **subscribes** to `/image_raw` and **publishes** processed images to `/image_processed`.
*   A `navigation` node **subscribes** to `/image_processed` and `/robot_odom` (odometry data).

## Services: Request/Response Interaction

**Services** are a type of synchronous, one-to-one communication in ROS 2, used for a request/response pattern. When a node needs to request a specific operation from another node and wait for a response, it uses a service.

*   **Client/Server Model**: A node that requests an operation is a **client**. A node that provides the operation is a **server**.
*   **Synchronous**: The client sends a request and blocks until it receives a response from the server.
*   **Atomic Operations**: Services are suitable for operations that are expected to complete within a predictable time and return a single result, like getting a configuration parameter or triggering a specific action.

**Analogy**: Services are like making a phone call to a specific person. You (client) call someone (server) to ask a question (request), and you wait for them to answer (response).

**Example**:
*   A `navigation` node might act as a **client** to request a map from a `map_server` node, which acts as the **server**.
*   A `gripper_controller` node might provide a service `/set_gripper_position` where a client can send a request with a desired position, and the server responds once the gripper has moved.

## Summary

Nodes, Topics, and Services form the backbone of communication in ROS 2. By understanding how these components work together, you can design modular, robust, and scalable robotic applications. In the next lesson, we will dive into `rclpy`, the Python client library for ROS 2, and explore how to write nodes that utilize these communication mechanisms.
