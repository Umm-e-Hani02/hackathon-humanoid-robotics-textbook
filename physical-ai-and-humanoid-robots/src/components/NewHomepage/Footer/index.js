import React from 'react';
import styles from './styles.module.css';

export default function Footer() {
  return (
    <footer className={styles.footer}>
      <div className="container text--center">
        <p className={styles.footerText}>
          Designed with 💜. Built with Docusaurus.
        </p>
        <p className={styles.footerText}>
          Copyright © {new Date().getFullYear()} Physical AI Book Project.
        </p>
      </div>
    </footer>
  );
}
