import React from 'react';
import styles from './styles.module.css';

const learningPoints = [
  'Core principles of Physical AI and embodied intelligence',
  'ROS 2 (Robot Operating System 2) for robotic control',
  'Simulation techniques using Gazebo and Unity environments',
  'AI-driven navigation and perception using NVIDIA Isaac',
  'Hands-on exercises to solidify understanding and practical skills',
  'Integration of vision, language, and robotic control systems'
];

export default function WhatYouWillLearn() {
  return (
    <section className={styles.features}>
      <div className="container">
        <h2 className="section-title">What You'll Learn</h2>
        <div className={styles.content}>
          <p className={styles.description}>
            This book provides a comprehensive guide to building intelligent humanoid robots,
            covering everything from fundamental concepts to advanced implementations.
            You'll gain hands-on experience with industry-standard tools and frameworks.
          </p>

          <div className={styles.learningPoints}>
            <ul className={styles.learningList}>
              {learningPoints.map((point, index) => (
                <li key={index} className={styles.learningItem}>
                  <span className={styles.learningIcon}>•</span>
                  {point}
                </li>
              ))}
            </ul>
          </div>
        </div>
      </div>
    </section>
  );
}
