import React from 'react';
import styles from './styles.module.css';

const features = [
  {
    title: 'Embodied Intelligence',
    description: 'Learn how AI agents interact with the physical world through sensors and actuators.'
  },
  {
    title: 'ROS 2 Framework',
    description: 'Master the Robot Operating System for building robust, distributed robotic applications.'
  },
  {
    title: 'Simulation & Control',
    description: 'Create digital twins and implement advanced control algorithms for humanoid robots.'
  },
  {
    title: 'Vision-Language-Action',
    description: 'Build multimodal AI systems that integrate perception, reasoning, and action.'
  },
  {
    title: 'AI-Driven Navigation',
    description: 'Implement intelligent path planning and obstacle avoidance systems.'
  },
  {
    title: 'Hardware Integration',
    description: 'Connect your AI algorithms to real robotic platforms and sensors.'
  }
];

export default function KeyFeatures() {
  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <h2 className="section-title">Key Topics</h2>
        <div className={styles.featuresGrid}>
          {features.map((feature, index) => (
            <div key={index} className={styles.featureCard}>
              <h3 className={styles.featureTitle}>{feature.title}</h3>
              <p className={styles.featureDescription}>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}