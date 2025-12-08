import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';

// Feature data for the cards section
const FeatureList = [
  {
    title: 'Simulate Complex Systems',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default, // Placeholder SVG
    description: (
      <>
        Explore advanced simulations for humanoid robots, understanding their
        interactions with diverse physical environments.
      </>
    ),
  },
  {
    title: 'Develop Intelligent Control',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default, // Placeholder SVG
    description: (
      <>
        Learn to design and implement cutting-edge AI algorithms for
        robot movement, perception, and decision-making.
      </>
    ),
  },
  {
    title: 'Build and Integrate Hardware',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default, // Placeholder SVG
    description: (
      <>
        Gain practical experience in assembling and integrating various
        hardware components into functional robotic systems.
      </>
    ),
  },
];

// Feature card component
function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

// Homepage header (Hero section)
function HomepageHero() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading Now 🚀
          </Link>
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
      title={`Hello from ${siteConfig.title}`}
      description="Learn to build, simulate, and control intelligent humanoid robots.">
      <HomepageHero />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}