// Operator UI against a running digital twin (twin.launch.py, stack platform,
// with openamr-platform-sw#90 on the stack). Headless Chromium, one worker,
// the steps run in order on one twin. Simulation evidence only: not hardware,
// physical or safety acceptance.
//
// Environment
//   TWIN_UI_URL      Flask app, default http://127.0.0.1:5050
//   TWIN_ROS_ENV     script that sources ROS and both workspaces (for ros2 CLI checks)
//   TWIN_EVIDENCE    directory for the evidence JSON lines
//   GITHUB_SHA       when set, /api/build-info must name this commit
const fs = require("fs");
const path = require("path");
const { execSync } = require("child_process");
const { test, expect } = require("@playwright/test");

const EVIDENCE = process.env.TWIN_EVIDENCE || path.join(__dirname, "..", "twin-evidence");
fs.mkdirSync(EVIDENCE, { recursive: true });
const record = (step, data) =>
  fs.appendFileSync(
    path.join(EVIDENCE, "steps.jsonl"),
    JSON.stringify({ step, wall: new Date().toISOString(), ...data }) + "\n",
  );

const ros = (cmd, timeoutS = 30, env = {}) => {
  const setup = process.env.TWIN_ROS_ENV;
  if (!setup) throw new Error("TWIN_ROS_ENV is not set");
  try {
    return execSync(`bash -c 'source ${setup} && timeout ${timeoutS} ${cmd.replace(/'/g, "'\\''")}'`, {
      encoding: "utf8",
      env: { ...process.env, ...env },
      stdio: ["ignore", "pipe", "pipe"],
    });
  } catch (error) {
    return `${error.stdout || ""}${error.stderr || ""}[exit ${error.status}]`;
  }
};

const GOAL = { id: "twin-goal", name: "TwinGoal", x: 1.0, y: 0.0, z: 0.0, w: 1.0 };

test.describe.configure({ mode: "serial" });

let page;
const display = () => page.getByTestId("operator-status-display");
const stopRow = () => page.getByTestId("operator-status-stop");
const taskRow = () => page.getByTestId("operator-status-task");

// Resume through the banner; a refused release (the gate names the reason in
// /safety/stop_state) is retried up to three times, each refusal recorded.
const resumeUntilRunning = async (step) => {
  for (let attempt = 1; attempt <= 3; attempt += 1) {
    const resume = page.getByRole("button", { name: "Resume", exact: true });
    if (!(await resume.isVisible())) break;
    await expect(resume).toBeEnabled({ timeout: 30000 });
    await resume.click();
    await page.getByRole("button", { name: "Confirm resume", exact: true }).click();
    try {
      await expect(stopRow()).toHaveText("RUNNING", { timeout: 10000 });
      record(step, { attempt, stop: "RUNNING" });
      return;
    } catch {
      record(step, { attempt, stop: await stopRow().textContent() });
    }
  }
  await expect(stopRow()).toHaveText("RUNNING", { timeout: 10000 });
};

test.beforeAll(async ({ browser }) => {
  const context = await browser.newContext();
  await context.addInitScript((goal) => {
    window.localStorage.setItem("openamrOnboardingSeen", "true");
    window.localStorage.setItem("openamrRuntimeConfig", JSON.stringify({ demoMode: false }));
    window.localStorage.setItem("openamrSavedWaypoints", JSON.stringify([goal]));
  }, GOAL);
  page = await context.newPage();
  page.on("console", (m) => {
    if (m.type() === "error") record("console-error", { text: m.text().slice(0, 300) });
  });
});

test("the served bundle matches build-info.json", async ({ request }) => {
  const response = await request.get("/api/build-info");
  const body = await response.json();
  record("build-info", { status: response.status(), body });
  expect(response.status()).toBe(200);
  expect(body.ok).toBe(true);
  if (process.env.GITHUB_SHA) expect(body.commit).toBe(process.env.GITHUB_SHA);
});

test("connects to the twin and shows live status, map and pose", async () => {
  await page.goto("/"); // the Map page is the app root (pages/registry.js)
  await expect(page.getByText("Robot connected").first()).toBeVisible({ timeout: 60000 });
  await expect(display()).not.toHaveText("DISCONNECTED", { timeout: 30000 });
  await expect(page.getByTestId("operator-status-pose-age")).toHaveText("fresh", { timeout: 60000 });
  await expect(page.getByTestId("operator-status-nav-age")).toHaveText("fresh", { timeout: 90000 });
  await page.waitForTimeout(3000);
  await page.screenshot({ path: path.join(EVIDENCE, "01-connected-map.png"), fullPage: true });
  record("connected", {
    display: await display().textContent(),
    detail: await page.getByTestId("operator-status-detail").textContent(),
    readiness: await page.getByTestId("operator-status-readiness").textContent(),
    stop: await stopRow().textContent(),
    task: await taskRow().textContent(),
    ui_map_info: ros("ros2 topic info /ui/map"),
  });
});

test("Resume releases the stop lock through the gate", async () => {
  // The gate starts locked (fail closed); Stop (software) pressed in an
  // earlier step would also hold here until Resume.
  await resumeUntilRunning("resume-attempt");
  await expect(stopRow()).toHaveText("RUNNING", { timeout: 30000 });
  await expect(display()).not.toHaveText(/STOPPED|STOP STATE UNKNOWN/, { timeout: 10000 });
  record("resume", { display: await display().textContent(), stop: await stopRow().textContent() });
});

test("a navigation goal from the UI runs and Cancel ends it CANCELED", async () => {
  await page.getByRole("button", { name: `▸ ${GOAL.name}` }).click();
  await expect(page.getByText("Navigating").first()).toBeVisible({ timeout: 60000 });
  await expect(taskRow()).toHaveText(/Navigation ACTIVE/, { timeout: 30000 });
  record("goal-active", { task: await taskRow().textContent() });
  await page.getByRole("button", { name: "Cancel", exact: true }).click();
  await expect(page.getByText("Canceled").first()).toBeVisible({ timeout: 30000 });
  await expect(taskRow()).toHaveText(/Navigation CANCELED/, { timeout: 30000 });
  record("goal-cancel", {
    task: await taskRow().textContent(),
    status: ros("ros2 topic echo --once --full-length /navigate_to_pose/_action/status", 20).slice(-1500),
  });
});

test("Stop (software) during a goal locks, cancels the goal, and Resume releases", async () => {
  await page.getByRole("button", { name: `▸ ${GOAL.name}` }).click();
  await expect(taskRow()).toHaveText(/Navigation ACTIVE/, { timeout: 60000 });
  const t0 = Date.now();
  await page.getByRole("button", { name: "Stop (software)" }).first().click();
  await expect(display()).toHaveText("STOPPED", { timeout: 2000 });
  await expect(stopRow()).toHaveText(/LOCKED/, { timeout: 5000 });
  const tLocked = Date.now();
  await expect(taskRow()).toHaveText(/Navigation (CANCELED|ABORTED)/, { timeout: 15000 });
  const task = await taskRow().textContent();
  await page.screenshot({ path: path.join(EVIDENCE, "02-stopped.png"), fullPage: true });
  record("stop", {
    lock_shown_after_ms: tLocked - t0,
    task_after_stop: task,
    cmd_vel_publishers: ros("ros2 topic info -v /cmd_vel"),
    cmd_vel_sample: ros("ros2 topic echo --once /cmd_vel", 10),
  });
  // The strict cancel criterion is CANCELED, not ABORTED (#90 review).
  expect(task).toMatch(/Navigation CANCELED/);

  await resumeUntilRunning("resume-after-stop-attempt");
  // A cancelled goal stays cancelled after Resume.
  await page.waitForTimeout(5000);
  await expect(taskRow()).not.toHaveText(/ACTIVE/);
  record("resume-after-stop", { stop: await stopRow().textContent(), task: await taskRow().textContent() });
});

test("Baseline 1 mission status (Proposed #82) is shown and labelled Proposed", async () => {
  // Fixture message published by this test, in the #82 payload shape; the
  // Baseline 1 mission itself does not run in this job.
  const payload = JSON.stringify({
    phase: "grasp",
    state: "running",
    sim: 1.0,
    phases: [{ phase: "navigate", verdict: "PASS" }, { phase: "grasp", verdict: "RUNNING" }],
  });
  ros(`ros2 topic pub -r 2 -t 6 /baseline1/mission_status std_msgs/msg/String "{data: '$MISSION_DATA'}"`, 20, {
    MISSION_DATA: payload,
  });
  await expect(taskRow()).toHaveText("Baseline 1: grasp (running)", { timeout: 20000 });
  await expect(page.getByTestId("operator-status-task-source")).toHaveText(/Proposed/);
  record("mission-status", { task: await taskRow().textContent() });
});

test("STALE when /navigation/status stops, then back", async () => {
  ros("pkill -STOP -f '[n]avigation_status_node'", 5);
  try {
    await expect(display()).toHaveText("STALE", { timeout: 30000 });
    await page.screenshot({ path: path.join(EVIDENCE, "03-stale.png"), fullPage: true });
    record("stale", { detail: await page.getByTestId("operator-status-detail").textContent() });
  } finally {
    ros("pkill -CONT -f '[n]avigation_status_node'", 5);
  }
  await expect(display()).not.toHaveText("STALE", { timeout: 30000 });
});

test("DISCONNECTED when the bridge goes away", async () => {
  ros("pkill -INT -f '[r]osbridge_websocket'", 5);
  await expect(display()).toHaveText("DISCONNECTED", { timeout: 15000 });
  await expect(page.getByText("Stopped (software)").first()).toBeVisible();
  await page.screenshot({ path: path.join(EVIDENCE, "04-disconnected.png"), fullPage: true });
  record("disconnected", { detail: await page.getByTestId("operator-status-detail").textContent() });
});
