import React from 'react';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';

const learningPoints = [
  'The core principles of Embodied Intelligence and Physical AI.',
  'Hands-on experience with ROS 2, Python, and URDF for robot development.',
  'How to build and test robots in advanced simulators like Gazebo and NVIDIA Isaac Sim.',
  'Techniques for AI-driven perception, navigation, and path planning.',
];

export default function HeroSection() {
  return (
    <header className={styles.hero}>
      <div className={styles.heroContainer}>
        <h1 className={styles.heroTitle}>Physical AI & Humanoid Robots</h1>
        <p className={styles.heroSubtitle}>
          Your practical guide to building, programming, and deploying intelligent humanoid robots from the ground up.
        </p>
        
        <div className={styles.learnSection}>
          <h2 className="section-title">What You'll Learn</h2>
          <ul className={styles.learnList}>
            {learningPoints.map((point, index) => (
              <li key={index} className={styles.learnListItem}>
                <svg className={styles.learnIcon} xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"></path><polyline points="22 4 12 14.01 9 11.01"></polyline></svg>
                <span>{point}</span>
              </li>
            ))}
          </ul>
        </div>

        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Get Started
          </Link>
        </div>
      </div>
    </header>
  );
}