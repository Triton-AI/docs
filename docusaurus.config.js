// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const { themes } = require("prism-react-renderer");
const remarkExternalUrlRef = require("./src/plugins/remark-externalUrlRef");

const lightCodeTheme = themes.github;
const darkCodeTheme = themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "TritonAI",
  tagline: "Official Documentation for TritonAI @ UCSD",
  url: "https://triton-ai.github.io",
  baseUrl: "/docs/",
  //onBrokenLinks: "throw", // Remove or add comments if needed
  onBrokenMarkdownLinks: "warn",
  //onBrokenAnchors: "throw", // Remove or add comments if needed
  onDuplicateRoutes: "warn",
  favicon: "img/tritonai_favicon.ico",
  organizationName: "triton-ai", // Usually your GitHub org/user name.
  projectName: "docs",
  trailingSlash: false,

  presets: [
    [
      "classic",
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve("./sidebars.js"),
          remarkPlugins: [remarkExternalUrlRef],
          routeBasePath: "/",
        },
        blog: false,
        theme: {
          customCss: require.resolve("./src/css/custom.css"),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      docs: {
        sidebar: {
          autoCollapseCategories: true,
        },
      },
      navbar: {
        title: "",
        logo: undefined,
        items: [
          {
            type: "html",
            position: "left",
            className: " navbar__logo navbar__item_logo_container",
            value: `
            <a href="/docs" target="_blank">
              <img src="https://triton-ai.github.io/docs/img/tritonai_favicon.ico" alt="TritonAI" width="100%" height="100%">
            </a>
            <svg width="2" height="60%" viewBox="0 0 2 33" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path fill-rule="evenodd" clip-rule="evenodd" d="M0 33V0H2V33H0Z" fill="white"/>
            </svg>
            `,
          },
          {
            href: "/docs",
            position: "left",
            label: "Triton AI",
          },
          {
            href: "https://discord.gg/ZzfBGvG3FF ",
            label: "Discord",
            position: "right",
          },
        ],
      },
      footer: {
        style: "dark",
        logo: {
          src: "img/tritonai_favicon.ico",
          alt: "TritonAI Logo",
          href: "https://appcircle.io",

        },
        links: [
        ],
        copyright: `Copyright © ${new Date().getFullYear()}, TritonAI @ UCSD`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: [
          "ruby",
          "groovy",
          "java",
          "csharp",
          "kotlin",
          "bash",
          "diff",
          "json",
          "markdown",
          "shell-session",
          "yaml",
        ],
      },
      algolia: {
        apiKey: "b56a5dc4e52ec9e97ad93981cc668c4a",
        indexName: "appcircle",
        appId: "4U9FKQJ034",
        contextualSearch: true,
      },
      zooming: {
        selector: ".markdown img",
        delay: 500,
        background: {
          light: "rgb(255, 255, 255)",
          dark: "#1b1b1d",
        },
      },
    }),
  plugins: [
    [
      "@docusaurus/plugin-google-analytics",
      {
        trackingID: "UA-40954643-12",
        anonymizeIP: true,
      },
    ],
    "docusaurus-plugin-sass",
    "docusaurus-plugin-zooming",
  ],
};

module.exports = config;
