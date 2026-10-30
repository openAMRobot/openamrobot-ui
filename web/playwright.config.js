// Route smoke test: the production build, in the existing Demo Mode, at the
// two panel resolutions. Run with `npm run build && npm run test:e2e`.
const { defineConfig } = require("@playwright/test");

const PORT = 4173;

module.exports = defineConfig({
  testDir: "./e2e",
  forbidOnly: !!process.env.CI,
  retries: 0,
  reporter: "list",
  use: {
    baseURL: `http://127.0.0.1:${PORT}`,
    // Lets a machine with a preinstalled Chromium run the test without
    // `npx playwright install`; CI installs the browser instead.
    launchOptions: process.env.PW_CHROMIUM_PATH
      ? { executablePath: process.env.PW_CHROMIUM_PATH }
      : {},
  },
  projects: [
    { name: "1280x800", use: { viewport: { width: 1280, height: 800 } } },
    { name: "1920x1200", use: { viewport: { width: 1920, height: 1200 } } },
  ],
  webServer: {
    command: `node e2e/serve-build.js`,
    url: `http://127.0.0.1:${PORT}`,
    env: { PORT: String(PORT) },
    reuseExistingServer: !process.env.CI,
  },
});
