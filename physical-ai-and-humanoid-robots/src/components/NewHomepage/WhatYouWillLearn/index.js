import React from 'react';
import styles from './styles.module.css';
import clsx from 'clsx';

const WhatYouWillLearnList = [
  {
    title: 'Foundations of Physical AI',
    description: (
      <>
        Understand the core principles of embodied intelligence, where AI meets the physical world.
      </>
    ),
  },
  {
    title: 'Humanoid Robot Design',
    description: (
      <>
        Explore the mechanics, sensors, and actuators that make humanoid robots possible.
      </>
    ),
  },
  {
    title: 'ROS 2 and Python for Robotics',
    description: (
      <>
        Gain hands-on experience with the essential tools for developing and controlling robots.
      </>
    ),
  },
  {
    title: 'Advanced Simulation Techniques',
    description: (
      <>
        Master simulation environments like Gazebo and Unity to test and refine your creations.
      </>
    ),
  },
  {
    title: 'AI and Machine Learning for Robots',
    description: (
      <>
        Learn how to implement perception, navigation, and manipulation in your humanoid robots.
      </>
    ),
  },
  {
    title: 'NVIDIA Isaac Sim and ROS',
    description: (
      <>
        Leverage the power of NVIDIA's robotics platform for photorealistic simulation and advanced AI.
      </>
    ),
  },
];

function WhatYouWillLearnItem({title, description}) {
  return (
    <div className={clsx('col col--4', styles.feature)}>
      <div className="text--center padding-horiz--md">
        <h3 className="item-title">{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function WhatYouWillLearn() {
  return (
    <section className={styles.features}>
      <div className="container">
        <h2 className="section-title">Key Topics Covered</h2>
        <div className="row">
          {WhatYouWillLearnList.map((props, idx) => (
            <WhatYouWillLearnItem key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
