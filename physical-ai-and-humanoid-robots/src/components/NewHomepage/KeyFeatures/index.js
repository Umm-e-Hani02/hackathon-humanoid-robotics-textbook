import React from 'react';
import styles from './styles.module.css';
import clsx from 'clsx';

const FeatureList = [
  {
    title: 'Embodied Intelligence',
    description: (
      <>
        Learn how to give robots a physical presence and enable them to
        interact with the world through sensors and actuators.
      </>
    ),
  },
  {
    title: 'ROS 2 & Python',
    description: (
      <>
        Master the fundamentals of the Robot Operating System (ROS 2) and
        write Python-based agents to control your humanoid.
      </>
    ),
  },
  {
    title: 'Advanced Simulation',
    description: (
      <>
        Utilize powerful tools like Gazebo, Unity, and NVIDIA Isaac Sim to
        test and train your robots in realistic virtual environments.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4', styles.feature)}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function KeyFeatures() {
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
