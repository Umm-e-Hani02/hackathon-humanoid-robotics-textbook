import React from 'react';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';

export default function HeroSection() {
  return (
    <header className={styles.hero}>
      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          <div className={styles.textContent}>
            <h1 className={styles.heroTitle}>Physical AI & Humanoid Robotics</h1>
            <p className={styles.heroSubtitle}>
              Learn to build, simulate, and control intelligent humanoid robots with cutting-edge AI techniques.
            </p>

            <div className={`${styles.buttons} button-container`}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                Start Learning
              </Link>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}