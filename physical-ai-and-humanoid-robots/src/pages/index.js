import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHero() {
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">

        <Heading as="h1" className={styles.heroTitle}>
          Pioneering Humanoid Intelligence
        </Heading>
        <p className={styles.heroSubtitle}>
          Unveiling the future of Physical AI: a comprehensive guide to building, simulating, and controlling advanced humanoid robots.
        </p>
        <div className={styles.buttons}>
          <Link
            className={clsx('button button--lg', styles.readButton)}
            to="/docs/intro">
            Start Reading
          </Link>
          <Link
            className={clsx('button button--lg', styles.exploreButton)}
            to="/docs/module1/what-is-physical-ai">
            Explore Modules
          </Link>
        </div>
      </div>
    </header>
  );
}

function WhatYoullLearn() {
  return (
    <section className={styles.section}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          What You'll Learn
        </Heading>
        <ul className={styles.bulletPoints}>
          <li>
            <svg width="24" height="24" viewBox="0 0 24 24"><path d="M9 16.17L4.83 12l-1.42 1.41L9 19 21 7l-1.41-1.41z"></path></svg>
            The fundamentals of Physical AI and Embodied Intelligence.
          </li>
          <li>
            <svg width="24" height="24" viewBox="0 0 24 24"><path d="M9 16.17L4.83 12l-1.42 1.41L9 19 21 7l-1.41-1.41z"></path></svg>
            How to use ROS2 and Python for robot control.
          </li>
          <li>
            <svg width="24" height="24" viewBox="0 0 24 24"><path d="M9 16.17L4.83 12l-1.42 1.41L9 19 21 7l-1.41-1.41z"></path></svg>
            Advanced simulation techniques in Gazebo, Unity, and NVIDIA Isaac Sim.
          </li>
          <li>
            <svg width="24" height="24" viewBox="0 0 24 24"><path d="M9 16.17L4.83 12l-1.42 1.41L9 19 21 7l-1.41-1.41z"></path></svg>
            Navigation and path planning for humanoid robots.
          </li>
        </ul>
      </div>
    </section>
  );
}

const modules = [
  'Module 1: The Robotic Nervous System (ROS 2)',
  'Module 2: The Digital Twin',
  'Module 3: The AI-Robot Brain',
  'Module 4: Vision Language Action VLA',
];

function ModulesOverview() {
  return (
    <section className={styles.section}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          Modules Overview
        </Heading>
        <div className={styles.modulesGrid}>
          {modules.map((module, idx) => (
            <div key={idx} className={styles.moduleCard}>
              <Heading as="h3">{module}</Heading>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function WhyThisBook() {
  return (
    <section className={styles.section}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          Why This Book?
        </Heading>
        <div className={styles.benefitsList}>
          <div className={styles.benefit}>
            <Heading as="h4">Hands-On Learning</Heading>
            <p>Go from theory to practice with hands-on projects in every module.</p>
          </div>
          <div className={styles.benefit}>
            <Heading as="h4">Open Source</Heading>
            <p>The entire book is open source and available for free.</p>
          </div>
          <div className={styles.benefit}>
            <Heading as="h4">Community-Driven</Heading>
            <p>Contribute to the book and learn from other community members.</p>
          </div>
        </div>
      </div>
    </section>
  );
}



export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="A comprehensive guide to building, simulating, and controlling intelligent humanoid robots.">
      <HomepageHero />
      <main>
        <WhatYoullLearn />
        <ModulesOverview />
        <WhyThisBook />

      </main>
    </Layout>
  );
}
