// Every registered page renders in Demo Mode without console errors or
// uncaught exceptions. Routes are read from src/pages/registry.js and from
// the page plugins under src/plugins/, so a new page is covered without
// editing this file.
const fs = require("fs");
const path = require("path");
const { test, expect } = require("@playwright/test");

const SRC = path.join(__dirname, "..", "src");
const pluginIndexes = fs
  .readdirSync(path.join(SRC, "plugins"), { withFileTypes: true })
  .filter((entry) => entry.isDirectory())
  .map((entry) => path.join(SRC, "plugins", entry.name, "index.js"))
  .filter((file) => fs.existsSync(file));
const ROUTES = [path.join(SRC, "pages", "registry.js"), ...pluginIndexes].flatMap((file) =>
  [...fs.readFileSync(file, "utf8").matchAll(/\bpath:\s*"([^"]+)"/g)].map((m) => m[1]),
);

// The app must work on a robot without internet: every request to a host
// other than the local test server is blocked, and any attempt fails the
// test.
const LOCAL_ORIGIN = "http://127.0.0.1:4173";

test.beforeEach(async ({ context }, testInfo) => {
  await context.addInitScript(() => {
    window.localStorage.setItem(
      "openamrRuntimeConfig",
      JSON.stringify({ demoMode: true }),
    );
  });
  testInfo.externalRequests = [];
  await context.route(
    (url) => url.origin !== LOCAL_ORIGIN && url.protocol.startsWith("http"),
    (route) => {
      testInfo.externalRequests.push(route.request().url());
      return route.abort("blockedbyclient");
    },
  );
});

test("the registry lists the pages", () => {
  expect(ROUTES.length).toBeGreaterThanOrEqual(18);
  expect(ROUTES).toEqual(expect.arrayContaining(["/", "/config", "/notes"]));
});

for (const route of ROUTES) {
  test(`${route} renders in Demo Mode without console errors`, async ({ page }, testInfo) => {
    const errors = [];
    const blocklyMedia = [];
    page.on("response", (response) => {
      if (new URL(response.url()).pathname.startsWith("/blockly-media/")) {
        blocklyMedia.push(`${response.status()} ${response.headers()["content-type"]}`);
      }
    });
    page.on("console", (message) => {
      if (message.type() === "error") errors.push(`console: ${message.text()}`);
    });
    page.on("pageerror", (error) => errors.push(`pageerror: ${error.message}`));

    await page.goto(route, { waitUntil: "networkidle" });
    await expect(page.locator("#root")).not.toBeEmpty();
    // Demo Mode is on: its banner is shown.
    await expect(page.getByText(/demo mode/i).first()).toBeVisible();
    // Let demo telemetry tick a few times.
    await page.waitForTimeout(1500);

    expect(errors).toEqual([]);
    expect(testInfo.externalRequests).toEqual([]);
    if (route === "/blocks") {
      // Blockly's media is served by the app itself.
      expect(blocklyMedia.length).toBeGreaterThan(0);
      // A missing file would come back as the index.html fallback.
      blocklyMedia.forEach((entry) => {
        expect(entry).toMatch(/^200 /);
        expect(entry).not.toMatch(/text\/html/);
      });
    }
  });
}
