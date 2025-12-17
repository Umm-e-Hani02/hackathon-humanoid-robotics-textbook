import React from 'react';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';
import clsx from 'clsx';

export default function CtaSection() {
  return (
    <section className={styles.ctaSection}>
      <div className="container text--center">
        <h2 className="section-title">Ready to Begin?</h2>
        <p className={styles.ctaSubtitle}>
          Dive into the first chapter and start your journey into the world of
          physical AI.
        </p>
        <Link
          className="button button--primary button--lg"
          to="/docs/introduction-to-physical-ai/introduction">
          Start Building Your Robot
        </Link>
      </div>
    </section>
  );
}
