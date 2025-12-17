import React from 'react';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';

const ModuleList = [
  {
    title: 'Module 1: Foundations of Physical AI',
    link: '/docs/introduction-to-physical-ai/what-is-physical-ai',
    description: 'Start with the core principles of embodied intelligence, sensors, and actuators.',
  },
  {
    title: 'Module 2: ROS 2 and Python Agents',
    link: '/docs/module2/ros2-basics',
    description: 'Learn the Robot Operating System (ROS 2) to create professional-grade control systems.',
  },
  {
    title: 'Module 3: Advanced Simulation',
    link: '/docs/module3/gazebo-simulation',
    description: 'Master virtual testing in Gazebo and Unity to accelerate development and validation.',
  },
  {
    title: 'Module 4: AI for Humanoids',
    link: '/docs/module4/isaac-sdk-and-sim',
    description: 'Implement navigation and perception using NVIDIA Isaac Sim and other advanced tools.',
  },
];

function ModuleCard({ title, description, link }) {
  return (
    <div className={styles.moduleCard}>
      <div className={styles.cardContent}>
        <h3 className="item-title">{title}</h3>
        <p className={styles.cardDescription}>{description}</p>
      </div>
      <div className={styles.cardActions}>
        <Link
          className="button button--primary"
          to={link}>
          Explore
        </Link>
      </div>
    </div>
  );
}

export default function Modules() {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <h2 className="section-title">Explore the Modules</h2>
        <div className={styles.modulesGrid}>
          {ModuleList.map((props, idx) => (
            <ModuleCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}