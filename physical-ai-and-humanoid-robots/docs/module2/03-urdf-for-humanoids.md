---
sidebar_position: 3
title: URDF for Humanoids - Describing Robot Anatomy
description: Learn how Unified Robot Description Format (URDF) defines the physical characteristics of humanoid robots for simulation, visualization, and control in ROS 2.
keywords: [URDF, Humanoid Robots, Robot Anatomy, Kinematics, Xacro, ROS 2, Robot Simulation]
---

# URDF for Humanoids: Describing Robot Anatomy

To effectively simulate, visualize, and control humanoid robots, we need a standardized way to describe their physical characteristics. The **Unified Robot Description Format (URDF)** serves this purpose within the ROS ecosystem. URDF is an XML-based file format used to describe all aspects of a robot, including its kinematic and dynamic properties, visual appearance, and collision models.

## What is URDF?

URDF defines a robot as a collection of **links** (rigid bodies) connected by **joints** (allowing relative motion between links). Each element within a URDF file contributes to a comprehensive model of the robot.

### Key Elements of a URDF File

1.  **`<robot>` Tag**: The root element, containing all other elements. It has a `name` attribute for the robot.

2.  **`<link>` Tag**: Describes a rigid body segment of the robot.
    *   **`<visual>`**: Defines the visual properties of the link, such as its geometry (e.g., box, cylinder, sphere, mesh) and material (color, texture). This is what you see in a simulator or visualization tool.
    *   **`<collision>`**: Defines the collision properties of the link. This is used by physics engines to detect contact. Often a simplified version of the visual geometry to save computation.
    *   **`<inertial>`**: Defines the mass, center of mass, and inertia tensor of the link. Crucial for realistic physics simulation.

3.  **`<joint>` Tag**: Describes the connection between two links.
    *   **`name`**: Unique identifier for the joint.
    *   **`type`**: Specifies the joint's movement type (e.g., `revolute` for rotation, `prismatic` for linear motion, `fixed` for no motion).
    *   **`parent` / `child`**: Specifies which link is the parent and which is the child in the kinematic chain.
    *   **`<origin>`**: Defines the joint's position and orientation relative to its parent link.
    *   **`<axis>`**: For revolute and prismatic joints, defines the axis of motion.
    *   **`<limit>`**: For revolute and prismatic joints, defines the upper and lower position limits, velocity limits, and effort limits.
    *   **`<calibration>`, `<dynamics>`, `<safety_controller>`**: Optional tags for fine-tuning joint behavior.

### A Simple Humanoid Segment Example (Conceptual)

Imagine a single leg segment of a humanoid. It would have a `thigh` link connected to a `shin` link by a `knee` joint.

```xml
<robot name="simple_humanoid_leg">

  <link name="base_link" />

  <link name="thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <link name="shin">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="thigh"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" velocity="1.0" effort="100"/>
  </joint>

  <joint name="knee_joint" type="revolute">
    <parent link="thigh"/>
    <child link="shin"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/> <!-- Position relative to thigh end -->
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0" velocity="1.0" effort="100"/>
  </joint>

</robot>
```
**Note**: This is a simplified example. Real humanoid URDFs are much larger and often include meshes (for visuals) and more complex joint setups.

## Xacro: Extending URDF

Writing large URDF files can be tedious and repetitive. **Xacro (XML Macros)** is a preprocessor for XML files (like URDF) that allows for more concise and readable descriptions. It enables:

*   **Macros**: Define reusable blocks of XML.
*   **Properties**: Define variables to avoid hardcoding values.
*   **Mathematical Expressions**: Perform calculations within the file.
*   **Conditional Inclusion**: Include or exclude parts of the description based on conditions.

Most complex humanoid robot descriptions use Xacro to generate the final URDF.

## Why URDF is Essential for Humanoids

*   **Simulation**: Physics engines (like Gazebo) use the inertial, collision, and joint properties to accurately simulate robot behavior.
*   **Visualization**: Tools like RViz use the visual properties to render a 3D model of the robot.
*   **Kinematics/Dynamics**: Software libraries use the link and joint descriptions to perform forward and inverse kinematics, and calculate dynamics.
*   **Hardware Abstraction**: Provides a common interface for control algorithms, regardless of the underlying physical robot.

Understanding URDF is a foundational skill for anyone working with humanoid robots in simulation or real-world applications. In the next hands-on exercise, you will have the opportunity to control a simulated robot joint, applying some of the ROS 2 communication concepts with a robot described by URDF.
