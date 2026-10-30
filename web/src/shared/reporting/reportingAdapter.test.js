import {
  DISPLAY,
  READY_WITH_WARNINGS_REASON,
  RESULT_UNKNOWN_TEXT,
  STALE_AFTER_MS,
  deriveReport,
  initialReportingState,
  reduceReporting,
} from "./reportingAdapter";
import { battery, diagnostics, goalStatus, lock, navFeedback, navStatus } from "./testing/fixtures";

// Wall clock for local time 0: 30 Oct 2026, 09:00:00 panel local time, so
// local time 1000 is shown as 09:00:01.
const WALL0 = new Date(2026, 9, 30, 9, 0, 0).getTime();

// Build a state by applying events at the given local times.
const apply = (events, start = initialReportingState()) => {
  let state = start;
  const logs = [];
  events.forEach(([event, at]) => {
    const r = reduceReporting(state, event, at, WALL0 + at);
    state = r.state;
    logs.push(...r.logs);
  });
  return { state, logs };
};

// Connected, lock released, healthy navigation, fresh diagnostics and battery at t=1000.
const healthy = (nav = navStatus(), at = 1000) => [
  [{ type: "connection", status: "connected" }, 0],
  [{ type: "lock", msg: lock(false) }, at],
  [{ type: "navigationStatus", msg: nav }, at],
  [{ type: "diagnostics", msg: diagnostics(0) }, at],
  [{ type: "battery", msg: battery(80) }, at],
];

const view = (events, now = 1000) => deriveReport(apply(events).state, now);

describe("display states", () => {
  it("DISCONNECTED before any connection", () => {
    expect(deriveReport(initialReportingState(), 0).display).toBe(DISPLAY.DISCONNECTED);
  });

  it("UNKNOWN while connected but the lock has no sample", () => {
    const v = view([[{ type: "connection", status: "connected" }, 0]]);
    expect(v.display).toBe(DISPLAY.UNKNOWN);
    expect(v.reason).toMatch(/lock state is unknown/);
  });

  it("UNKNOWN when navigation status is missing", () => {
    const v = view([
      [{ type: "connection", status: "connected" }, 0],
      [{ type: "lock", msg: lock(false) }, 1000],
    ]);
    expect(v.display).toBe(DISPLAY.UNKNOWN);
    expect(v.reason).toBe("No navigation status received yet");
  });

  it("STOPPED (SOFTWARE) from the stop lock, never SAFETY_STOPPED", () => {
    const v = view([...healthy(), [{ type: "lock", msg: lock(true) }, 1000]]);
    expect(v.display).toBe(DISPLAY.STOPPED_SOFTWARE);
    expect(v.display).not.toBe(DISPLAY.SAFETY_STOPPED);
  });

  it("FAULTED on navigation health FAULT", () => {
    const v = view(healthy(navStatus({ health: 3, active_reasons: [1005] })));
    expect(v.display).toBe(DISPLAY.FAULTED);
    expect(v.reason).toBe("Navigation configuration fault");
  });

  it("FAULTED on a /diagnostics ERROR", () => {
    const v = view([...healthy(), [{ type: "diagnostics", msg: diagnostics(2, "lidar", "no data") }, 1000]]);
    expect(v.display).toBe(DISPLAY.FAULTED);
    expect(v.reason).toBe("Diagnostics error: lidar: no data (last error at 09:00:01)");
  });

  it("FAULTED when a node is inactive and the stack state is unknown", () => {
    const v = view(healthy(navStatus({ stack: { state: 0, reason: 1004, inactive_nodes: ["amcl"] } })));
    expect(v.display).toBe(DISPLAY.FAULTED);
  });

  it("NEEDS_ASSISTANCE when recovery attempts are used up", () => {
    const v = view(healthy(navStatus({ active_reasons: [5002] })));
    expect(v.display).toBe(DISPLAY.NEEDS_ASSISTANCE);
    expect(v.reason).toBe("Recovery attempts used up");
  });

  it("NEEDS_ASSISTANCE when localization is lost", () => {
    const v = view(healthy(navStatus({ localization: { state: 1, health: 3, reason: 3002 } })));
    expect(v.display).toBe(DISPLAY.NEEDS_ASSISTANCE);
  });

  it("RECOVERING while a recovery runs", () => {
    const v = view(healthy(navStatus({ task: { state: 3, reason: 5001 }, recovery: { action: 2, attempt: 2, attempt_limit: 6, reason: 5001 } })));
    expect(v.display).toBe(DISPLAY.RECOVERING);
    expect(v.reason).toBe("Recovery in progress (attempt 2 of 6)");
  });

  it("BLOCKED when the path is blocked", () => {
    const v = view(healthy(navStatus({ task: { state: 2 }, active_reasons: [4008] })));
    expect(v.display).toBe(DISPLAY.BLOCKED);
    expect(v.reason).toBe("Path is blocked near the robot");
  });

  it("EXECUTING while a goal is active, with progress from Nav2 feedback", () => {
    const v = view([
      ...healthy(),
      [{ type: "goalStatus", msg: goalStatus(1, 2) }, 1000],
      [{ type: "navFeedback", msg: navFeedback(1, 10) }, 1000],
      [{ type: "navFeedback", msg: navFeedback(1, 4, 6, 1) }, 1000],
    ]);
    expect(v.display).toBe(DISPLAY.EXECUTING);
    expect(v.progress).toEqual({ percent: 60, distanceRemaining: 4, etaSeconds: 6, recoveries: 1 });
    expect(v.task.label).toBe("Navigation goal");
  });

  it("WAITING during a mission wait step", () => {
    const v = view([
      ...healthy(),
      [{ type: "mission", name: "Patrol", run: { runId: "m1", step: 2, total: 3, status: "running", waitingOn: "Mission step 2: waiting 30 s" } }, 1000],
    ]);
    expect(v.display).toBe(DISPLAY.WAITING);
    expect(v.task).toEqual({ kind: "mission", label: "Mission: Patrol", step: "Step 2 of 3", running: true });
  });

  it("READY only when every condition holds", () => {
    expect(view(healthy()).display).toBe(DISPLAY.READY);
  });

  it.each([
    ["navigation not ready", healthy(navStatus({ navigation_readiness: 0, not_ready_reasons: [9001] })), "No link to the base controller"],
    ["navigation degraded", healthy(navStatus({ health: 2 })), "Navigation health is not OK"],
    ["battery low", [...healthy(), [{ type: "battery", msg: battery(25) }, 1000]], "Battery low (25%)"],
    ["no battery data", healthy().filter(([e]) => e.type !== "battery"), "No battery data"],
    ["no diagnostics", healthy().filter(([e]) => e.type !== "diagnostics"), "No valid diagnostics received"],
  ])("not READY when %s: WAITING with the blocking reason", (_label, events, reason) => {
    const v = view(events);
    expect(v.display).toBe(DISPLAY.WAITING);
    expect(v.reason).toBe(reason);
  });

  it("never derives SAFETY_STOPPED, even with an E-stop reason in navigation status", () => {
    const v = view(healthy(navStatus({ navigation_readiness: 0, not_ready_reasons: [9003] })));
    expect(v.display).not.toBe(DISPLAY.SAFETY_STOPPED);
    expect(v.rows.find((r) => r.label === "Physical E-stop").value).toBe("Not reported");
  });

  it("shows Arms, Lift and Physical E-stop as Not reported", () => {
    const rows = view(healthy()).rows;
    ["Arms", "Lift", "Physical E-stop"].forEach((label) =>
      expect(rows.find((r) => r.label === label).value).toBe("Not reported"),
    );
  });
});

describe("priority conflicts", () => {
  it("DISCONNECTED wins over everything", () => {
    const { state } = apply([...healthy(), [{ type: "lock", msg: lock(true) }, 1000]]);
    const v = deriveReport(reduceReporting(state, { type: "connection", status: "disconnected" }, 1000).state, 1000);
    expect(v.display).toBe(DISPLAY.DISCONNECTED);
  });

  it("a known engaged lock wins over a navigation fault", () => {
    const v = view([...healthy(navStatus({ health: 3 })), [{ type: "lock", msg: lock(true) }, 1000]]);
    expect(v.display).toBe(DISPLAY.STOPPED_SOFTWARE);
  });

  it("an engaged lock wins over stale navigation status, which shows as a warning", () => {
    const v = view([...healthy(), [{ type: "lock", msg: lock(true) }, 3000]], 3000);
    expect(v.display).toBe(DISPLAY.STOPPED_SOFTWARE);
    expect(v.warnings).toContain("Navigation data is stale");
  });

  // Owner decision: a known, engaged lock ranks above UNKNOWN navigation
  // data in every form; only DISCONNECTED ranks higher.
  it.each([
    ["missing", [
      [{ type: "connection", status: "connected" }, 0],
      [{ type: "lock", msg: lock(true) }, 1000],
    ]],
    ["invalid", [...healthy(navStatus({ contract_version: 99 })), [{ type: "lock", msg: lock(true) }, 1000]]],
  ])("an engaged lock wins over %s navigation status", (_label, events) => {
    const v = view(events);
    expect(v.display).toBe(DISPLAY.STOPPED_SOFTWARE);
    expect(v.reason).toBe("The software stop lock is engaged");
  });

  it("FAULTED wins over an active goal", () => {
    const v = view([...healthy(navStatus({ health: 3 })), [{ type: "goalStatus", msg: goalStatus(1, 2) }, 1000]]);
    expect(v.display).toBe(DISPLAY.FAULTED);
  });

  it("RECOVERING wins over BLOCKED", () => {
    const v = view(healthy(navStatus({ task: { state: 3 }, recovery: { action: 1, attempt: 1, attempt_limit: 6 }, active_reasons: [4008] })));
    expect(v.display).toBe(DISPLAY.RECOVERING);
  });
});

describe("staleness", () => {
  it("navigation status goes stale after 1.5 s: UNKNOWN", () => {
    // Keep every other source fresh at the check time.
    const others = (at) => [
      [{ type: "lock", msg: lock(false) }, at],
      [{ type: "diagnostics", msg: diagnostics(0) }, at],
      [{ type: "battery", msg: battery(80) }, at],
    ];
    const edge = 1000 + STALE_AFTER_MS.navigationStatus;
    expect(view([...healthy(), ...others(edge)], edge).display).toBe(DISPLAY.READY);
    const v = view([...healthy(), ...others(edge + 1)], edge + 1);
    expect(v.display).toBe(DISPLAY.UNKNOWN);
    expect(v.reason).toBe("Navigation status is stale");
  });

  it("the stop lock goes stale after 1 s: UNKNOWN, never released", () => {
    const v = view(healthy(), 1000 + STALE_AFTER_MS.stopLock + 1);
    expect(v.display).toBe(DISPLAY.UNKNOWN);
    expect(v.rows.find((r) => r.label === "Software stop lock").value).toBe("Unknown");
  });

  it("stale battery blocks READY and is marked stale", () => {
    const at = 1000 + STALE_AFTER_MS.battery + 1;
    const v = view(
      [
        ...healthy(),
        [{ type: "lock", msg: lock(false) }, at],
        [{ type: "navigationStatus", msg: navStatus() }, at],
        [{ type: "diagnostics", msg: diagnostics(0) }, at],
      ],
      at,
    );
    expect(v.display).toBe(DISPLAY.WAITING);
    expect(v.reason).toBe("Battery data is stale or invalid");
    expect(v.rows.find((r) => r.label === "Battery").freshness).toBe("Stale");
  });

  it("stale diagnostics block READY", () => {
    const at = 1000 + STALE_AFTER_MS.diagnostics + 1;
    const v = view(
      [
        ...healthy(),
        [{ type: "lock", msg: lock(false) }, at],
        [{ type: "navigationStatus", msg: navStatus() }, at],
        [{ type: "battery", msg: battery(80) }, at],
      ],
      at,
    );
    expect(v.display).toBe(DISPLAY.WAITING);
    expect(v.reason).toBe("Diagnostics are stale");
  });

  it("invalid navigation status is UNKNOWN, not healthy", () => {
    const v = view(healthy(navStatus({ contract_version: 2 })));
    expect(v.display).toBe(DISPLAY.UNKNOWN);
    expect(v.reason).toBe("Navigation status is invalid");
  });

  it("an out-of-range battery value is invalid, not low or healthy", () => {
    const v = view([...healthy(), [{ type: "battery", msg: battery(140) }, 1000]]);
    expect(v.rows.find((r) => r.label === "Battery").freshness).toBe("Invalid");
    expect(v.display).toBe(DISPLAY.WAITING);
  });
});

describe("disconnect", () => {
  it("a reconnect starts with no data, keeping the run and result on screen", () => {
    const { state } = apply([
      ...healthy(),
      [{ type: "goalStatus", msg: goalStatus(1, 2) }, 1000],
      [{ type: "goalStatus", msg: goalStatus(1, 4) }, 1000],
      [{ type: "connection", status: "disconnected" }, 1100],
      [{ type: "connection", status: "connected" }, 1200],
    ]);
    const v = deriveReport(state, 1200);
    expect(v.display).toBe(DISPLAY.UNKNOWN);
    expect(v.rows.find((r) => r.label === "Battery").value).toBe("Unknown");
    expect(v.result.outcome).toBe("SUCCEEDED");
  });
});

describe("runs and results", () => {
  it("a new goal id opens a new run and clears the result panel", () => {
    const v1 = view([...healthy(), [{ type: "goalStatus", msg: goalStatus(1, 4) }, 1000]]);
    expect(v1.result.outcome).toBe("SUCCEEDED");
    expect(v1.completion).toBe("Completed: Navigation goal");
    const v2 = view([
      ...healthy(),
      [{ type: "goalStatus", msg: goalStatus(1, 4) }, 1000],
      [{ type: "goalStatus", msg: goalStatus(2, 2) }, 1000],
    ]);
    expect(v2.result).toBeNull();
    expect(v2.display).toBe(DISPLAY.EXECUTING);
  });

  it("a mission start opens a new run and clears the result panel", () => {
    const v = view([
      ...healthy(),
      [{ type: "goalStatus", msg: goalStatus(1, 6) }, 1000],
      [{ type: "mission", name: "Patrol", run: { runId: "m1", step: 1, total: 2, status: "running" } }, 1000],
    ]);
    expect(v.result).toBeNull();
    expect(v.task.label).toBe("Mission: Patrol");
  });

  it("a mission waypoint goal keeps the mission as the current task", () => {
    const v = view([
      ...healthy(),
      [{ type: "mission", name: "Patrol", run: { runId: "m1", step: 1, total: 2, status: "running" } }, 1000],
      [{ type: "goalStatus", msg: goalStatus(5, 2) }, 1000],
    ]);
    expect(v.task.label).toBe("Mission: Patrol");
  });

  it("ignores duplicates and out-of-order events by goal id and stamp", () => {
    const { state } = apply([
      ...healthy(),
      [{ type: "goalStatus", msg: goalStatus(2, 2, 200) }, 1000],
      // An older goal (earlier stamp) arriving late is ignored.
      [{ type: "goalStatus", msg: goalStatus(1, 4, 150) }, 1000],
      // A duplicate status for the current goal changes nothing.
      [{ type: "goalStatus", msg: goalStatus(2, 2, 200) }, 1000],
      [{ type: "goalStatus", msg: goalStatus(2, 6, 200) }, 1000],
      // Once terminal, a late EXECUTING for the same goal is ignored.
      [{ type: "goalStatus", msg: goalStatus(2, 2, 200) }, 1000],
    ]);
    expect(state.goal.id.endsWith("02")).toBe(true);
    expect(state.result.outcome).toBe("ABORTED");
    // A goal id seen before does not reopen.
    const again = reduceReporting(state, { type: "goalStatus", msg: goalStatus(1, 2, 300) }, 1000).state;
    expect(again.goal.id.endsWith("02")).toBe(true);
  });

  it("aborted results carry the navigation reason in plain words", () => {
    const v = view([
      ...healthy(navStatus({ task: { state: 5, reason: 4004 } })),
      [{ type: "goalStatus", msg: goalStatus(1, 6) }, 1000],
    ]);
    expect(v.result).toEqual(expect.objectContaining({ outcome: "ABORTED", reason: "No valid path to the goal" }));
  });

  it("a malformed outcome keeps the run open, warns, logs once, and a later valid outcome replaces it", () => {
    const { state, logs } = apply([
      ...healthy(),
      [{ type: "goalStatus", msg: goalStatus(1, 2) }, 1000],
      [{ type: "goalStatus", msg: goalStatus(1, 42) }, 1000],
      [{ type: "goalStatus", msg: goalStatus(1, "done") }, 1000],
    ]);
    const v = deriveReport(state, 1000);
    expect(v.display).toBe(DISPLAY.EXECUTING); // the run stays open
    expect(v.result.invalid).toBe(true);
    expect(v.warnings).toContain(RESULT_UNKNOWN_TEXT);
    expect(logs).toHaveLength(1);
    expect(logs[0].message).toContain(RESULT_UNKNOWN_TEXT);

    const after = reduceReporting(state, { type: "goalStatus", msg: goalStatus(1, 4) }, 1000).state;
    const v2 = deriveReport(after, 1000);
    expect(v2.result).toEqual(expect.objectContaining({ outcome: "SUCCEEDED", invalid: false }));
    expect(v2.warnings).not.toContain(RESULT_UNKNOWN_TEXT);
  });

  it("an unknown mission or program status is a malformed outcome, not a result", () => {
    const { state, logs } = apply([
      ...healthy(),
      [{ type: "uiRun", kind: "program", run: { runId: "p1", name: "Demo", step: 1, total: 2, status: "running" } }, 1000],
      [{ type: "uiRun", kind: "program", run: { runId: "p1", name: "Demo", step: 2, total: 2, status: "exploded" } }, 1000],
    ]);
    expect(deriveReport(state, 1000).result.invalid).toBe(true);
    expect(logs).toHaveLength(1);
    const after = reduceReporting(state, { type: "uiRun", kind: "program", run: { runId: "p1", name: "Demo", step: 2, total: 2, status: "succeeded" } }, 1000).state;
    expect(deriveReport(after, 1000).result).toEqual(expect.objectContaining({ outcome: "SUCCEEDED", invalid: false }));
  });

  it("program and route runs report step n of m and their outcome", () => {
    const v = view([
      ...healthy(),
      [{ type: "uiRun", kind: "route", run: { runId: "r1", name: "Waypoint queue", step: 2, total: 4, status: "running" } }, 1000],
    ]);
    expect(v.display).toBe(DISPLAY.EXECUTING);
    expect(v.task.step).toBe("Step 2 of 4");
    const done = view([
      ...healthy(),
      [{ type: "uiRun", kind: "route", run: { runId: "r1", name: "Waypoint queue", step: 2, total: 4, status: "running" } }, 1000],
      [{ type: "uiRun", kind: "route", run: { runId: "r1", name: "Waypoint queue", step: 2, total: 4, status: "failed", reason: "Goal failed" } }, 1000],
    ]);
    expect(done.result).toEqual(expect.objectContaining({ outcome: "FAILED", reason: "Goal failed" }));
  });
});

// Owner decision (30 Oct 2026, review of #54): validate every /diagnostics
// message and judge freshness per entry name.
describe("diagnostics validation and per-entry freshness", () => {
  // healthy() with its diagnostics event replaced by `msg`.
  const healthyWithDiagnostics = (msg) =>
    healthy().map(([e, at]) => (e.type === "diagnostics" ? [{ type: "diagnostics", msg }, at] : [e, at]));
  const recovery = [[{ type: "diagnostics", msg: diagnostics(0) }, 1000]];
  const diagRow = (v) => v.rows.find((r) => r.label === "Diagnostics");
  // Fresh lock, navigation and battery at `at`, without diagnostics.
  const refreshAt = (at) => [
    [{ type: "lock", msg: lock(false) }, at],
    [{ type: "navigationStatus", msg: navStatus() }, at],
    [{ type: "battery", msg: battery(80) }, at],
  ];

  it.each([
    ["{}", {}],
    ["{ status: [] }", { status: [] }],
    ["an entry with an invalid level (7)", { status: [{ name: "lidar", level: 7, message: "" }] }],
    ["an entry with a level that is not a number", { status: [{ name: "lidar", level: "2", message: "" }] }],
    ["an entry with an empty name", { status: [{ name: "  ", level: 0, message: "" }] }],
  ])("diagnostics replaced by %s: not READY, then valid recovery data gives READY", (_label, msg) => {
    const { state, logs } = apply(healthyWithDiagnostics(msg));
    const v = deriveReport(state, 1000);
    expect(v.display).toBe(DISPLAY.WAITING);
    expect(v.reason).toBe("No valid diagnostics received");
    expect(diagRow(v).value).toBe("Unknown");
    if (Array.isArray(msg.status) && msg.status.length === 0) expect(logs).toHaveLength(0);
    else expect(logs.some((l) => /diagnostics/.test(l.message))).toBe(true);

    const after = apply(recovery, state).state;
    const v2 = deriveReport(after, 1000);
    expect(v2.display).toBe(DISPLAY.READY);
    expect(diagRow(v2).value).toBe("OK");
  });

  it("an invalid entry is ignored while a valid entry in the same message counts", () => {
    const msg = { status: [{ name: "bad", level: 9 }, { name: "lidar", level: 1, message: "rate low" }] };
    const { state, logs } = apply(healthyWithDiagnostics(msg));
    const v = deriveReport(state, 1000);
    expect(v.display).toBe(DISPLAY.READY);
    expect(diagRow(v).value).toBe("Warning");
    expect(logs).toHaveLength(1);
  });

  it("an unrelated diagnostic never refreshes a stale entry", () => {
    // imu reports WARN at t=1000 and goes stale; lidar reports OK at 2500.
    const v = view(
      [
        ...healthyWithDiagnostics(diagnostics(1, "imu", "rate low")),
        ...refreshAt(2500),
        [{ type: "diagnostics", msg: diagnostics(0, "lidar") }, 2500],
      ],
      2500,
    );
    expect(v.display).toBe(DISPLAY.READY);
    expect(v.warnings).not.toContain("Diagnostics warning: imu: rate low");
  });

  it("every entry stale: WAITING, Diagnostics row Unknown and stale", () => {
    const at = 2500;
    const v = view(
      [
        ...healthy(),
        [{ type: "lock", msg: lock(false) }, at],
        [{ type: "navigationStatus", msg: navStatus() }, at],
        [{ type: "battery", msg: battery(80) }, at],
      ],
      at,
    );
    expect(v.display).toBe(DISPLAY.WAITING);
    expect(v.reason).toBe("Diagnostics are stale");
    expect(diagRow(v)).toEqual(expect.objectContaining({ value: "Unknown", freshness: "Stale" }));
  });

  it("a fresh ERROR or STALE level shows FAULTED and the row Error", () => {
    const err = view(healthyWithDiagnostics(diagnostics(2, "lidar", "no data")));
    expect(err.display).toBe(DISPLAY.FAULTED);
    expect(diagRow(err).value).toBe("Error");
    const staleLevel = view(healthyWithDiagnostics(diagnostics(3, "camera")));
    expect(staleLevel.display).toBe(DISPLAY.FAULTED);
    expect(staleLevel.reason).toBe("Diagnostics stale: camera (last fault at 09:00:01)");
    expect(diagRow(staleLevel).value).toBe("Error");
  });
});

// Owner decision (30 Oct 2026, second P2 on #54): a diagnostics fault is
// latched per source name. Only a fresh OK or WARN from the same name, or a
// new connection, clears it; ageing out and other names never do.
describe("diagnostics fault latch", () => {
  const refreshAt = (at) => [
    [{ type: "lock", msg: lock(false) }, at],
    [{ type: "navigationStatus", msg: navStatus() }, at],
    [{ type: "battery", msg: battery(80) }, at],
  ];
  const diagRow = (v) => v.rows.find((r) => r.label === "Diagnostics");
  // Source A (lidar) reports ERROR at t=1000; source B (imu) reports OK at
  // t=1000 and again at t=2500, when everything else is also fresh.
  const aErrorThenStale = [
    ...healthy(),
    [{ type: "diagnostics", msg: diagnostics(2, "lidar", "no data") }, 1000],
    [{ type: "diagnostics", msg: diagnostics(0, "imu") }, 1000],
    ...refreshAt(2500),
    [{ type: "diagnostics", msg: diagnostics(0, "imu") }, 2500],
  ];

  it("ERROR from A, then A goes stale while B keeps sending OK: stays FAULTED", () => {
    // At t=2500 the lidar entry is 1.5 s old, past the 1 s freshness limit.
    const v = view(aErrorThenStale, 2500);
    expect(v.display).toBe(DISPLAY.FAULTED);
    expect(v.reason).toBe("Diagnostics error: lidar: no data (last error at 09:00:01)");
    expect(diagRow(v).value).toBe("Error");
    // Still latched much later, while B stays fresh.
    const later = view([...aErrorThenStale, ...refreshAt(60000), [{ type: "diagnostics", msg: diagnostics(0, "imu") }, 60000]], 60000);
    expect(later.display).toBe(DISPLAY.FAULTED);
  });

  it("A then reports OK: the latch clears and the page can become READY", () => {
    expect(view(aErrorThenStale, 2500).display).toBe(DISPLAY.FAULTED);
    const v = view([...aErrorThenStale, [{ type: "diagnostics", msg: diagnostics(0, "lidar") }, 2600]], 2600);
    expect(v.display).toBe(DISPLAY.READY);
    expect(v.reason).toBe("No blocking faults in current reported data");
    expect(diagRow(v).value).toBe("OK");
  });

  it("A reporting WARN also clears the latch; an invalid entry for A does not", () => {
    const invalid = [...aErrorThenStale, [{ type: "diagnostics", msg: { status: [{ name: "lidar", level: 7 }] } }, 2600]];
    expect(view(invalid, 2600).display).toBe(DISPLAY.FAULTED);
    const warn = view([...invalid, [{ type: "diagnostics", msg: diagnostics(1, "lidar", "rate low") }, 2700]], 2700);
    expect(warn.display).toBe(DISPLAY.READY);
    expect(warn.warnings).toContain("Diagnostics warning: lidar: rate low");
    // Review of #54: READY with a WARN must not claim that every check passes.
    expect(warn.reason).toBe(READY_WITH_WARNINGS_REASON);
    expect(warn.reason).toBe("Ready, with reported warnings shown below");
  });

  it("a STALE level latches the same way", () => {
    const v = view(
      [
        ...healthy(),
        [{ type: "diagnostics", msg: diagnostics(3, "camera") }, 1000],
        ...refreshAt(2500),
        [{ type: "diagnostics", msg: diagnostics(0) }, 2500],
      ],
      2500,
    );
    expect(v.display).toBe(DISPLAY.FAULTED);
    expect(v.reason).toBe("Diagnostics stale: camera (last fault at 09:00:01)");
  });

  it("the reason carries the time of the last error, and other faults are warnings", () => {
    const v = view(
      [
        ...healthy(),
        [{ type: "diagnostics", msg: diagnostics(2, "lidar", "no data") }, 1000],
        [{ type: "diagnostics", msg: diagnostics(2, "imu", "") }, 1500],
        [{ type: "diagnostics", msg: diagnostics(2, "lidar", "still no data") }, 2000],
      ],
      2000,
    );
    expect(v.reason).toBe("Diagnostics error: lidar: still no data (last error at 09:00:02)");
    expect(v.warnings).toContain("Also in fault: Diagnostics error: imu (last error at 09:00:01)");
  });

  it("a latched fault shows FAULTED even when navigation data is stale", () => {
    const v = view(
      [
        ...healthy(),
        [{ type: "diagnostics", msg: diagnostics(2, "lidar", "no data") }, 1000],
        [{ type: "lock", msg: lock(false) }, 2500],
      ],
      2500,
    );
    expect(v.display).toBe(DISPLAY.FAULTED);
  });

  it("a reconnect clears the latch; WAITING until valid diagnostics arrive", () => {
    expect(view(aErrorThenStale, 2500).display).toBe(DISPLAY.FAULTED);
    const reconnected = [
      ...aErrorThenStale,
      [{ type: "connection", status: "disconnected" }, 3000],
      [{ type: "connection", status: "connected" }, 3100],
      ...refreshAt(3200),
    ];
    const v = view(reconnected, 3200);
    expect(v.display).toBe(DISPLAY.WAITING);
    expect(v.reason).toBe("No valid diagnostics received");
    expect(diagRow(v).value).toBe("Unknown");
    // Invalid data does not end the wait.
    const invalid = [...reconnected, [{ type: "diagnostics", msg: { status: [{ name: "lidar", level: 9 }] } }, 3300]];
    expect(view(invalid, 3300).display).toBe(DISPLAY.WAITING);
    const ok = view([...invalid, [{ type: "diagnostics", msg: diagnostics(0, "imu") }, 3400]], 3400);
    expect(ok.display).toBe(DISPLAY.READY);
  });
});
