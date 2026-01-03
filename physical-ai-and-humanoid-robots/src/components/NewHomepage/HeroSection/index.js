import React from 'react';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';

export default function HeroSection() {
  const roboticImage = useBaseUrl('/img/robotic.png');

  return (
    <header className={styles.hero}>
      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          <div className={styles.textContent}>

            <h1 className={styles.heroTitle}>
              Physical AI &<br />
              Humanoid Robotics
            </h1>
            <p className={styles.heroSubtitle}>
              A comprehensive technical guide to building, simulating, and controlling intelligent humanoid robots with cutting-edge AI.
            </p>

            <div className={`${styles.buttons} button-container`}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                Start Reading
              </Link>
            </div>
          </div>

          <div className={styles.visualContent}>
            <div className={styles.abstractVisual} aria-hidden="true">
              <img
                src={roboticImage}
                alt="Technical robotic illustration"
                className={styles.techVisualization}
              />
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}
