// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Introduction to Physical AI',
      items: [
        'introduction-to-physical-ai/introduction',
        'introduction-to-physical-ai/what-is-physical-ai',
        'introduction-to-physical-ai/embodied-intelligence',
        'introduction-to-physical-ai/historical-context-and-evolution',
        'introduction-to-physical-ai/applications-in-humanoid-robotics',
        'introduction-to-physical-ai/challenges-and-future-directions',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/sensors-and-humanoid-basics',
        'module1/hands-on-simulated-environment',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module2/ros2-basics',
        'module2/rclpy-python-agents',
        'module2/urdf-for-humanoids',
        'module2/hands-on-joint-control',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module3/gazebo-simulation',
        'module3/unity-environments',
        'module3/sensor-simulation',
        'module3/hands-on-obstacle-course',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module4/isaac-sdk-and-sim',
        'module4/isaac-ros',
        'module4/nav2-path-planning',
        'module4/hands-on-path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Hardware Setup',
      items: [
        'hardware-setup/setup-guide',
      ],
    },

  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
