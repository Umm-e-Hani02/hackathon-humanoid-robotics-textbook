import React from 'react';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';

export default function CtaSection() {
  return (
    <section className={styles.ctaSection}>
      <div className="container text--center">
        <h2 className="section-title">Ready to Start Your Journey?</h2>
        <p className={styles.ctaSubtitle}>
          Begin exploring the fascinating world of physical AI and humanoid robotics today.
        </p>
        <div className={styles.buttonContainer}>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Start Learning
          </Link>
        </div>
      </div>
    </section>
  );
}
