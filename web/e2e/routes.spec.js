// Every registered page renders in Demo Mode without console errors or
// uncaught exceptions. Routes are read from src/pages/registry.js and from
// the page plugins under src/plugins/, so a new page is covered without
// editing this file.
//
// /report is a standalone registry page: it has a sidebar link but renders
// outside AppLayout. Its layout is tested in the second block below.
const fs = require("fs");
const path = require("path");
const { test, expect } = require("@playwright/test");

const SRC = path.join(__dirname, "..", "src");
const pluginIndexes = fs
  .readdirSync(path.join(SRC, "plugins"), { withFileTypes: true })
  .filter((entry) => entry.isDirectory())
  .map((entry) => path.join(SRC, "plugins", entry.name, "index.js"))
  .filter((file) => fs.existsSync(file));
const REGISTERED_ROUTES = [path.join(SRC, "pages", "registry.js"), ...pluginIndexes].flatMap((file) =>
  [...fs.readFileSync(file, "utf8").matchAll(/\bpath:\s*"([^"]+)"/g)].map((m) => m[1]),
);
const ROUTES = REGISTERED_ROUTES.filter((route) => route !== "/report");

// Blockly loads its sounds and sprites from this host by default. Serve the
// same files from the installed blockly package so the test does not depend
// on the internet.
const BLOCKLY_MEDIA_HOST = "https://blockly-demo.appspot.com/static/media/";
const BLOCKLY_MEDIA_DIR = path.join(__dirname, "..", "node_modules", "blockly", "media");

test.beforeEach(async ({ context }) => {
  await context.addInitScript(() => {
    window.localStorage.setItem(
      "openamrRuntimeConfig",
      JSON.stringify({ demoMode: true }),
    );
  });
  await context.route(`${BLOCKLY_MEDIA_HOST}**`, (route) =>
    route.fulfill({
      path: path.join(BLOCKLY_MEDIA_DIR, path.basename(new URL(route.request().url()).pathname)),
    }),
  );
});

test("the registry lists the pages", () => {
  expect(ROUTES.length).toBeGreaterThanOrEqual(18);
  expect(ROUTES).toEqual(expect.arrayContaining(["/", "/config", "/notes"]));
  expect(REGISTERED_ROUTES).toContain("/report");
});

for (const route of ROUTES) {
  test(`${route} renders in Demo Mode without console errors`, async ({ page }) => {
    const errors = [];
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
  });
}

// /report uses a separate full-screen layout (no AppLayout chrome), so it
// does not show the Demo Mode banner. It is tested separately here so that
// the smoke test covers it without weakening the banner check above.
test("/report renders in Demo Mode without console errors", async ({ page }) => {
  const errors = [];
  page.on("console", (message) => {
    if (message.type() === "error") errors.push(`console: ${message.text()}`);
  });
  page.on("pageerror", (error) => errors.push(`pageerror: ${error.message}`));

  await page.goto("/", { waitUntil: "networkidle" });
  await page.getByRole("button", { name: "Skip" }).click();
  const reportingLink = page.getByRole("link", { name: "Reporting Mode" });
  await expect(reportingLink).toBeVisible();
  await reportingLink.click();
  await expect(page).toHaveURL(/\/report$/);
  await expect(page.locator("#root")).not.toBeEmpty();
  // The page renders its own scope header instead of AppLayout chrome.
  await expect(page.getByTestId("reporting-page")).toBeVisible();
  await expect(page.getByTestId("reporting-header")).toBeVisible();
  await expect(page.getByRole("navigation", { name: "Primary navigation" })).toHaveCount(0);
  // Let any timers tick.
  await page.waitForTimeout(1500);

  expect(errors).toEqual([]);
});
