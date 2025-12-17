import React from 'react';
import styles from './styles.module.css';
import clsx from 'clsx';

const WhyThisBookList = [
  {
    title: 'Hands-On and Practical',
    description: (
      <>
        This book is focused on practical, hands-on examples that you can build and test yourself.
      </>
    ),
  },
  {
    title: 'Beginner-Friendly',
    description: (
      <>
        We start from the basics and gradually build up to more complex topics, making it accessible for everyone.
      </>
    ),
  },
  {
    title: 'Cutting-Edge Technologies',
    description: (
      <>
        Learn the most relevant and modern tools and frameworks in the robotics industry.
      </>
    ),
  },
  {
    title: 'Focus on Humanoid Robots',
    description: (
      <>
        This book is one of the few resources specifically dedicated to the exciting field of humanoid robotics.
      </>
    ),
  },
  {
    title: 'From Simulation to the Real World',
    description: (
      <>
        The skills you learn can be applied to both simulated and physical robots.
      </>
    ),
  },
  {
    title: 'Join a Growing Community',
    description: (
      <>
        Be part of the future of AI and robotics, a field with immense potential and opportunities.
      </>
    ),
  },
];

function WhyThisBookItem({title, description}) {
  return (
    <div className={clsx('col col--4', styles.feature)}>
      <div className="text--center padding-horiz--md">
        <h3 className="item-title">{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function WhyThisBook() {
  return (
    <section className={styles.features}>
      <div className="container">
        <h2 className="section-title">Why This Book</h2>
        <div className="row">
          {WhyThisBookList.map((props, idx) => (
            <WhyThisBookItem key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
