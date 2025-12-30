import React from 'react';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';

const ModuleList = [
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    link: '/docs/module1/sensors-and-humanoid-basics',
    description: 'Master the Robot Operating System for building robust, distributed robotic applications.',
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    link: '/docs/module2/ros2-basics',
    description: 'Create accurate virtual replicas of physical robots for testing and validation.',
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
    link: '/docs/module3/gazebo-simulation',
    description: 'Implement intelligent perception, navigation, and decision-making systems.',
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    link: '/docs/module4/isaac-sdk-and-sim',
    description: 'Build multimodal AI systems that integrate vision, language, and robotic control.',
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
          Learn More
        </Link>
      </div>
    </div>
  );
}

export default function Modules() {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <h2 className="section-title">Explore The Modules</h2>
        <div className={styles.modulesGrid}>
          {ModuleList.map((props, idx) => (
            <ModuleCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}