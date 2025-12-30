import React from 'react';
import styles from './styles.module.css';

const benefits = [
  'Comprehensive coverage from foundational concepts to advanced implementation techniques',
  'Hands-on projects and exercises to reinforce learning',
  'Industry-relevant technologies and frameworks',
  'Cutting-edge developments in embodied AI and humanoid robotics',
  'Practical guidance for building real-world robotic systems'
];

export default function WhyThisBook() {
  return (
    <section className={styles.whyThisBook}>
      <div className="container">
        <h2 className="section-title">Why This Book?</h2>
        <div className={styles.content}>
          <p className={styles.description}>
            This book bridges the gap between theoretical AI concepts and practical robotics implementation.
            With a focus on humanoid robots, you'll learn to build systems that perceive, reason, and act
            in the physical world using state-of-the-art tools and methodologies.
          </p>

          <div className={styles.benefits}>
            <h3 className={styles.benefitsTitle}>Key Points:</h3>
            <ul className={styles.benefitsList}>
              {benefits.map((benefit, index) => (
                <li key={index} className={styles.benefitItem}>
                  <span className={styles.benefitIcon}>✓</span>
                  {benefit}
                </li>
              ))}
            </ul>
          </div>
        </div>
      </div>
    </section>
  );
}
