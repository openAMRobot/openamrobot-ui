// Headless browser test of the operator UI against a running digital twin
// (twin.launch.py). Not part of `npm run test:e2e`: it needs the twin, the
// restricted rosbridge and the Flask app on 127.0.0.1. Run by
// .github/workflows/twin-e2e.yml; see docs/digital_twin.md to run it by hand.
const { defineConfig } = require("@playwright/test");

module.exports = defineConfig({
  testDir: "./e2e-twin",
  forbidOnly: !!process.env.CI,
  retries: 0,
  workers: 1,
  timeout: 180000,
  reporter: [["list"], ["json", { outputFile: process.env.TWIN_E2E_JSON || "twin-e2e.json" }]],
  use: {
    baseURL: process.env.TWIN_UI_URL || "http://127.0.0.1:5050",
    viewport: { width: 1280, height: 800 },
    screenshot: "on",
    trace: "retain-on-failure",
    launchOptions: process.env.PW_CHROMIUM_PATH
      ? { executablePath: process.env.PW_CHROMIUM_PATH }
      : {},
  },
});
