// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';



// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)



/** @type {import('@docusaurus/types').Config} */

const config = {

  title: 'Physical AI and Humanoid Robots',

  tagline: 'Learn to build, simulate, and control intelligent humanoid robots.',

  favicon: 'img/logo.png',



  // Set the production url of your site here``

  url: 'https://Umm-e-Hani02.github.io',

  // Set the /<baseUrl>/ pathname under which your site is served

  // For GitHub pages deployment, it is often '/<projectName>/'

  baseUrl: '/hackathon-humanoid-robotics-textbook/',

  trailingSlash: false,



  // GitHub pages deployment config.

  // If you aren't using GitHub pages, you don't need these.

  organizationName: 'Umm-e-Hani02', // Usually your GitHub org/user name.

  projectName: 'hackathon-humanoid-robotics-textbook', // Usually your repo name.



  onBrokenLinks: 'throw',

  

  // Even if you don't use internationalization, you can use this field to set

    // useful metadata like html lang. For example, if your site is Chinese, you

    // may want to replace "en" with "zh-Hans".

    i18n: {

      defaultLocale: 'en',

      locales: ['en'],

    },

  

    customFields: {
          // For local development, use localhost. For production, use Railway URL
          apiUrl: process.env.API_URL || 'http://127.0.0.1:8000/agent/chat',
          // Production URL: 'https://hackathon-humanoid-robotics-textbook-production.up.railway.app/agent/chat'



        },

  

    presets: [

      [

        'classic',

        /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Umm-e-Hani02/hackathon-humanoid-robotics-textbook/tree/main/',
        },
        theme: {
          customCss: ['./src/css/custom.css', './src/css/new-homepage.css'],
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        defaultMode: 'light',
      },
      navbar: {
        title: 'Physical AI Book',
        logo: {
          alt: 'Physical AI Book Logo',
          src: 'img/logo.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          // {
          //   to: '/#',
          //   label: 'Home',
          //   position: 'left',
          // },
          {
            href: 'https://github.com/Umm-e-Hani02/hackathon-humanoid-robotics-textbook',
            label: 'Github',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'light',
        links: [
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
              {
                label: 'X',
                href: 'https://x.com/docusaurus',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/Umm-e-Hani02/hackathon-humanoid-robotics-textbook',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Book. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;