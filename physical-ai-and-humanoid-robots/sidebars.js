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
        'intro-physical-ai/what-is-physical-ai',
      ],
    },
    {
      type: 'category',
      label: 'Hardware Setup',
      items: [
        'hardware-setup/setup-guide',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: Physical AI Fundamentals',
      items: [
        'module1/what-is-physical-ai',
        'module1/embodied-intelligence',
        'module1/sensors-and-humanoid-basics',
        'module1/hands-on-simulated-environment',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: ROS2 and Python Agents',
      items: [
        'module2/ros2-basics',
        'module2/rclpy-python-agents',
        'module2/urdf-for-humanoids',
        'module2/hands-on-joint-control',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Simulation Environments',
      items: [
        'module3/gazebo-simulation',
        'module3/unity-environments',
        'module3/sensor-simulation',
        'module3/hands-on-obstacle-course',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Advanced Robotics Platforms',
      items: [
        'module4/isaac-sdk-and-sim',
        'module4/isaac-ros',
        'module4/nav2-path-planning',
        'module4/hands-on-path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Tutorial Basics',
      items: [
        'tutorial-basics/create-a-document',
        'tutorial-basics/create-a-blog-post',
        'tutorial-basics/create-a-page',
        'tutorial-basics/markdown-features',
        'tutorial-basics/deploy-your-site',
        'tutorial-basics/congratulations',
      ],
    },
    {
      type: 'category',
      label: 'Tutorial Extras',
      items: [
        'tutorial-extras/manage-docs-versions',
        'tutorial-extras/translate-your-site',
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
