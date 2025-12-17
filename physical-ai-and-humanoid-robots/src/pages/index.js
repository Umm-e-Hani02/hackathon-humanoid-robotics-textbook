import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import HeroSection from '../components/NewHomepage/HeroSection';
import Modules from '../components/NewHomepage/Modules';
import KeyFeatures from '../components/NewHomepage/KeyFeatures';
import WhatYouWillLearn from '../components/NewHomepage/WhatYouWillLearn';
import WhyThisBook from '../components/NewHomepage/WhyThisBook';
import CtaSection from '../components/NewHomepage/CtaSection';

import '../css/new-homepage.css';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="An introductory guide to Physical AI and Humanoid Robotics."
    >
      <main>
        <HeroSection />
        {/* <KeyFeatures /> */}
        <Modules />
        <WhatYouWillLearn />
        <WhyThisBook />
        <CtaSection />
      </main>
    </Layout>
  );
}