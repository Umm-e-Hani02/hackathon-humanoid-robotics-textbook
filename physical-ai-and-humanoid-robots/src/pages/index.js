import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';

// Feature data for the cards section
const FeatureList = [
  {
    title: 'Introduction to Physical AI',
    subtitle: 'Foundation of embodied intelligence',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default, // Placeholder for icon
    description: (
      <>
        Dive deep into the core concepts of Physical AI, understanding how
        intelligence emerges from the interaction of body and environment.
      </>
    ),
    to: '/docs/intro-physical-ai/what-is-physical-ai', // Link to the module
  },
  {
    title: 'Humanoid Movement Systems',
    subtitle: 'Kinematics, dynamics, and control',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default, // Placeholder for icon
    description: (
      <>
        Explore the intricate mechanisms behind humanoid locomotion, balance,
        and manipulation, from theory to practical implementation.
      </>
    ),
    to: '/docs/humanoid-movement-systems/kinematics-and-dynamics', // Link to the module
  },
  {
    title: 'Control & Intelligence',
    subtitle: 'Algorithms for autonomous behavior',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default, // Placeholder for icon
    description: (
      <>
        Learn to design and implement advanced control strategies and AI algorithms
        that enable robots to perceive, decide, and act autonomously.
      </>
    ),
    to: '/docs/control-intelligence/control-algorithms', // Link to the module
  },
  {
    title: 'Development Roadmap',
    subtitle: 'Future of Physical AI',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default, // Placeholder for icon
    description: (
      <>
        Understand the current trends, challenges, and future directions in
        Physical AI and humanoid robotics research and development.
      </>
    ),
    to: '/docs/development-roadmap/future-directions', // Link to the module
  },
];

// Feature card component
function Feature({Svg, title, subtitle, description, to}) {
  return (
    <div className={clsx('col col--3')}> {/* col--3 for 4 cards in a row */}
      <Link to={to} className={styles.featureCardLink}> {/* Wrap card in Link */}
        <div className={styles.featureCard}>
          <div className="text--center">
            <Svg className={styles.featureSvg} role="img" />
          </div>
          <div className="text--center padding-horiz--md">
            <Heading as="h3">{title}</Heading>
            <p className={styles.featureSubtitle}>{subtitle}</p>
            <p>{description}</p>
          </div>
        </div>
      </Link>
    </div>
  );
}

// Homepage header (Hero section)
function HomepageHero() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="row hero__content">
          <div className="col col--6 hero__text-content">
            <Heading as="h1" className="hero__title">
              Physical AI & Humanoid Robotics
            </Heading>
            <p className="hero__subtitle">
              A comprehensive guide to building, simulating, and controlling intelligent humanoid robots.
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Start Learning
              </Link>
            </div>
          </div>
          <div className="col col--6 hero__image-content">
            <img src="/img/robot-hero.png" alt="Humanoid Robot" className={styles.heroImage} />
          </div>
        </div>
      </div>
    </header>
  );
}

// Homepage features (Card section)
function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

// Main Home component
export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="A comprehensive guide to building, simulating, and controlling intelligent humanoid robots.">
      <HomepageHero />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}