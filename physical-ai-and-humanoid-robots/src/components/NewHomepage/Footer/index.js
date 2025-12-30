import React from 'react';
import styles from './styles.module.css';
import Link from '@docusaurus/Link';

export default function Footer() {
  return (
    <footer className={styles.footer}>
      <div className="container">
        <div className={styles.footerContent}>
          <div className={styles.footerSection}>
            <h3 className={styles.footerTitle}>Physical AI & Humanoid Robotics</h3>
            <p className={styles.footerDescription}>
              Your comprehensive guide to building intelligent humanoid robots with cutting-edge AI techniques.
            </p>
          </div>

          <div className={styles.footerSection}>
            <h4 className={styles.sectionTitle}>Resources</h4>
            <ul className={styles.footerLinks}>
              <li><Link to="/docs/intro" className={styles.footerLink}>Documentation</Link></li>
              <li><Link to="/docs/module1/sensors-and-humanoid-basics" className={styles.footerLink}>Getting Started</Link></li>
              <li><Link to="https://github.com/Umm-e-Hani02/hackathon-humanoid-robotics-textbook" className={styles.footerLink}>GitHub</Link></li>
            </ul>
          </div>

          <div className={styles.footerSection}>
            <h4 className={styles.sectionTitle}>Connect</h4>
            <div className={styles.socialLinks}>
              <a href="https://github.com/Umm-e-Hani02/hackathon-humanoid-robotics-textbook" className={styles.socialLink} target="_blank" rel="noopener noreferrer">
                <svg className={styles.socialIcon} xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                  <path d="M9 19c-5 1.5-5-2.5-7-3m14 6v-3.87a3.37 3.37 0 0 0-.94-2.61c3.14-.35 6.44-1.54 6.44-7A5.44 5.44 0 0 0 20 4.77 5.07 5.07 0 0 0 19.91 1S18.73.65 16 2.48a13.38 13.38 0 0 0-7 0C6.27.65 5.09 1 5.09 1A5.07 5.07 0 0 0 5 4.77a5.44 5.44 0 0 0-1.5 3.78c0 5.42 3.3 6.61 6.44 7A3.37 3.37 0 0 0 9 18.13V22"></path>
                </svg>
              </a>
            </div>
          </div>
        </div>

        <div className={styles.footerBottom}>
          <p className={styles.copyright}>
            Copyright © {new Date().getFullYear()} Physical AI Book Project. Built with Docusaurus.
          </p>
        </div>
      </div>
    </footer>
  );
}
