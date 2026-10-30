import { evaluateLock, LOCK_LOCKED, LOCK_UNKNOWN } from "../motion/stopLock";
import { STOP_LOCK_STALE_AFTER_MS } from "../motion/timing";

/**
 * Consumer-neutral reporting adapter (AGENTS.md: one adapter for operator
 * and Reporting consumers). Scope: base and navigation only.
 *
 * Two pure parts, tested on their own:
 *   reduceReporting(state, event, nowMs) -> { state, logs }
 *   deriveReport(state, nowMs)           -> the view every consumer renders
 * and one live part (createReportingSource in reportingSource.js) that only
 * subscribes. Nothing here publishes, advertises or calls a service or
 * action (a Jest test enforces it).
 *
 * NavigationStatus reports navigation health only. It is not whole-robot
 * readiness, arm or base authority, or E-stop state. SAFETY_STOPPED is
 * reserved for a future physical safety-chain status producer and is never
 * derived here; the software stop lock maps to STOPPED (SOFTWARE).
 *
 * Every threshold and mapping row is documented in docs/reporting-mode.md.
 */

export const DISPLAY = {
  DISCONNECTED: "DISCONNECTED",
  UNKNOWN: "UNKNOWN",
  STOPPED_SOFTWARE: "STOPPED (SOFTWARE)",
  SAFETY_STOPPED: "SAFETY_STOPPED", // reserved: never derived by this adapter
  FAULTED: "FAULTED",
  NEEDS_ASSISTANCE: "NEEDS_ASSISTANCE",
  RECOVERING: "RECOVERING",
  BLOCKED: "BLOCKED",
  EXECUTING: "EXECUTING",
  WAITING: "WAITING",
  READY: "READY",
};

// Staleness thresholds, each from the producer's actual publish behaviour
// (openamr-platform-sw main 131e500f, openamrobot-ui battery.py). Accepted
// defaults (owner decision, 30 Oct 2026).
export const STALE_AFTER_MS = {
  // navigation_status_node: publishes on change, at least once per 2 Hz
  // heartbeat (0.5 s, +0.05 s check tolerance). Three heartbeats.
  navigationStatus: 1500,
  // battery.py: 1 s timer on /battery_status. Three periods.
  battery: 3000,
  // navigation_status_node mirrors /diagnostics on every 10 Hz tick.
  // Ten ticks.
  diagnostics: 1000,
  // safety_stop_gate_node republishes /safety/stop_lock at 10 Hz. Shared
  // with the command path (shared/motion/timing.js).
  stopLock: STOP_LOCK_STALE_AFTER_MS,
};

// Low battery for READY. Basis: S-01 initial battery policy "return at 25%
// SOC", subject to commissioning. Accepted default (30 Oct 2026).
export const BATTERY_LOW_PERCENT = 25;

// NavigationStatus constants (openamr_nav_msgs, contract version 1).
const NAV = {
  CONTRACT_VERSION: 1,
  HEALTH: { UNKNOWN: 0, OK: 1, DEGRADED: 2, FAULT: 3 },
  READY: 1,
  STACK: { UNKNOWN: 0, NOT_STARTED: 1, STARTING: 2, ACTIVE: 3, RESETTING: 4, FAILED: 5 },
  TASK: { UNKNOWN: 0, IDLE: 1, ACTIVE: 2, RECOVERING: 3, SUCCEEDED: 4, ABORTED: 5, CANCELED: 6 },
  LOC: { UNKNOWN: 0, LOST: 1, DEGRADED: 2, OK: 3 },
  RECOVERY_NONE: 0,
};

// Plain-word text for NavigationStatus reason codes.
export const REASON_TEXT = {
  1001: "Navigation is not started",
  1002: "Navigation is starting",
  1003: "Navigation is resetting",
  1004: "A navigation component is inactive",
  1005: "Navigation configuration fault",
  2001: "Sensor data is stale",
  2002: "A sensor is not configured",
  2003: "Waiting for first sensor data",
  2004: "Sensor data is invalid",
  2005: "Sensor data rate is low",
  3001: "Position is not set",
  3002: "Position estimate is uncertain",
  3003: "Position estimate is stale while moving",
  3004: "Position correction paused for docking",
  3005: "Robot frames are unavailable",
  4001: "Goal is outside the map",
  4002: "Goal is blocked",
  4003: "Start position is blocked",
  4004: "No valid path to the goal",
  4005: "Path planning timed out",
  4006: "Path following timed out",
  4007: "Robot is stuck",
  4008: "Path is blocked near the robot",
  4009: "Unknown navigation fault",
  5001: "Recovery in progress",
  5002: "Recovery attempts used up",
  6001: "Collision layer is inactive",
  6002: "A motion source is unprotected",
  6003: "Rear coverage is unavailable",
  6004: "Protection state is unknown",
  6005: "Speed limit active",
  6006: "Reverse speed limited",
  6007: "A motion source is partly protected",
  6008: "Slowing for a nearby obstacle",
  7001: "Dock engagement unknown",
  7002: "Dock markers not visible",
  7003: "Dock alignment failed",
  7004: "Docking timed out",
  7005: "Dock approach failed",
  7006: "Undocking failed",
  7007: "Undocking before the goal",
  7008: "Docking failed",
  8001: "Parking target unreachable",
  8002: "Parking position missed",
  8003: "Parking is not available",
  9001: "No link to the base controller",
  9002: "Base is not ready",
  9003: "Base reports an E-stop (not a safety indication)",
  9004: "Battery low (navigation status)",
};

export const reasonText = (code) =>
  REASON_TEXT[code] || (code ? `Navigation reason ${code}` : "");

const BLOCKED_REASONS = new Set([4002, 4003, 4004, 4008]);
const ASSISTANCE_REASONS = new Set([5002, 4007]);
const FAULT_REASONS = new Set([1005, 4009]);

// action_msgs/GoalStatus
const GOAL = { UNKNOWN: 0, ACCEPTED: 1, EXECUTING: 2, CANCELING: 3, SUCCEEDED: 4, CANCELED: 5, ABORTED: 6 };
const GOAL_TERMINAL = { 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED" };

const OUTCOME_FROM_UI_STATUS = { succeeded: "SUCCEEDED", failed: "FAILED", stopped: "CANCELED" };

export const RESULT_UNKNOWN_TEXT = "Result unknown (invalid data)";

export const initialReportingState = () => ({
  connection: "disconnected",
  lock: { lastValue: null, lastSampleMs: null },
  nav: { msg: null, receivedAt: null, invalid: "" },
  battery: { percent: null, receivedAt: null, invalid: "" },
  // Valid /diagnostics entries by name, each with its own receive time
  // (local monotonic) and wall-clock time. An entry whose last report was
  // ERROR or STALE is a latched fault until the same name reports OK or
  // WARN, or a new connection starts (owner decision, 30 Oct 2026).
  diagnostics: { entries: {} },
  goal: null, // { id, stampS, status, distance0, distance, eta, recoveries }
  seenGoals: [],
  task: null, // { runId, kind, name, step, total, status, waitingOn }
  result: null, // { runId, kind, name, outcome, reason, invalid }
});

const isFiniteNumber = (v) => typeof v === "number" && Number.isFinite(v);

const uuidKey = (uuid) =>
  Array.isArray(uuid) && uuid.length === 16 && uuid.every((b) => Number.isInteger(b) && b >= 0 && b < 256)
    ? uuid.map((b) => b.toString(16).padStart(2, "0")).join("")
    : null;

const stampSeconds = (stamp) =>
  stamp && isFiniteNumber(Number(stamp.sec)) && isFiniteNumber(Number(stamp.nanosec))
    ? Number(stamp.sec) + Number(stamp.nanosec) * 1e-9
    : null;

const validateNav = (msg) => {
  if (!msg || typeof msg !== "object") return "not a message";
  if (Number(msg.contract_version) !== NAV.CONTRACT_VERSION) {
    return `unsupported contract_version ${JSON.stringify(msg.contract_version)}`;
  }
  if (![0, 1, 2, 3].includes(msg.health)) return "health out of range";
  if (![0, 1].includes(msg.navigation_readiness)) return "navigation_readiness out of range";
  return "";
};

const openRun = (state, run) => ({ ...state, task: run, result: null });

const setResult = (state, result, logs) => {
  if (result.invalid) {
    logs.push({
      type: "system",
      severity: "warning",
      message: `Reporting: ${RESULT_UNKNOWN_TEXT} for ${result.kind} run ${result.runId}`,
    });
  }
  return { ...state, result };
};

const reduceGoalStatus = (state, msg, logs) => {
  const list = Array.isArray(msg?.status_list) ? msg.status_list : null;
  if (!list) return state;
  let next = state;
  // Oldest first so a batch applies in stamp order.
  const entries = list
    .map((entry) => ({
      id: uuidKey(entry?.goal_info?.goal_id?.uuid),
      stampS: stampSeconds(entry?.goal_info?.stamp),
      status: entry?.status,
    }))
    .filter((e) => e.id && e.stampS !== null)
    .sort((a, b) => a.stampS - b.stampS);

  entries.forEach(({ id, stampS, status }) => {
    const current = next.goal;
    if (!current || current.id !== id) {
      // A goal id seen before, or older than the current goal: out of
      // order or a duplicate. Ignore it.
      if (next.seenGoals.includes(id)) return;
      if (current && stampS < current.stampS) {
        // Remember it, so a later event under the same id stays ignored.
        next = { ...next, seenGoals: [...next.seenGoals, id].slice(-50) };
        return;
      }
      // A new goal opens a new navigation run and clears the result panel.
      // A mission, program or route that sent it stays the current task.
      next = {
        ...next,
        result: null,
        goal: { id, stampS, status: null, distance0: null, distance: null, eta: null, recoveries: 0 },
        seenGoals: [...next.seenGoals, id].slice(-50),
      };
    }
    const goal = next.goal;
    const valid = Number.isInteger(status) && status >= 0 && status <= 6;
    if (!valid) {
      // Malformed outcome: keep the run open, warn once per run.
      if (!next.result || next.result.runId !== goal.id) {
        next = setResult(
          next,
          { runId: goal.id, kind: "navigation", name: "Navigation goal", outcome: "UNKNOWN", reason: "", invalid: true },
          logs,
        );
      }
      return;
    }
    if (goal.status !== null && GOAL_TERMINAL[goal.status]) return; // terminal is latched
    if (goal.status === status) return; // duplicate
    next = { ...next, goal: { ...goal, status } };
    if (GOAL_TERMINAL[status]) {
      const navTask = next.nav.msg?.task;
      const reason =
        status === GOAL.ABORTED ? reasonText(navTask?.reason) || "Navigation aborted" : status === GOAL.CANCELED ? "Cancelled" : "";
      // A valid outcome replaces an earlier "Result unknown" for this run.
      next = setResult(
        next,
        { runId: goal.id, kind: "navigation", name: "Navigation goal", outcome: GOAL_TERMINAL[status], reason, invalid: false },
        logs,
      );
    }
  });
  return next;
};

const reduceFeedback = (state, msg) => {
  const id = uuidKey(msg?.goal_id?.uuid);
  if (!state.goal || id !== state.goal.id) return state;
  const fb = msg.feedback || {};
  const distance = isFiniteNumber(fb.distance_remaining) ? fb.distance_remaining : null;
  const eta = stampSeconds(fb.estimated_time_remaining);
  const goal = {
    ...state.goal,
    distance,
    distance0: distance === null ? state.goal.distance0 : Math.max(state.goal.distance0 ?? 0, distance),
    eta,
    recoveries: Number.isInteger(fb.number_of_recoveries) ? fb.number_of_recoveries : state.goal.recoveries,
  };
  return { ...state, goal };
};

const reduceTaskRun = (state, kind, run, name, logs) => {
  if (!run) return state;
  const runId = run.runId;
  const current = state.task;
  let next = state;
  if (run.status === "running") {
    if (!current || current.runId !== runId) {
      next = openRun(next, { runId, kind, name, step: run.step, total: run.total, status: "running", waitingOn: run.waitingOn || "" });
    } else {
      next = { ...next, task: { ...current, step: run.step, total: run.total, waitingOn: run.waitingOn || "" } };
    }
    return next;
  }
  // Terminal or unknown status for a run we did not see start: open it so
  // its result is still reported.
  if (!current || current.runId !== runId) {
    next = openRun(next, { runId, kind, name, step: run.step, total: run.total, status: "running", waitingOn: "" });
  }
  if (next.result && next.result.runId === runId && !next.result.invalid) return next; // latched
  const outcome = OUTCOME_FROM_UI_STATUS[run.status];
  next = { ...next, task: { ...next.task, status: outcome ? "done" : "running" } };
  return setResult(
    next,
    outcome
      ? { runId, kind, name, outcome, reason: run.reason || "", invalid: false }
      : { runId, kind, name, outcome: "UNKNOWN", reason: "", invalid: true },
    logs,
  );
};

// diagnostic_msgs/DiagnosticStatus levels. rosbridge_suite 2.7.1 sends the
// byte field as an integer.
const DIAG_LEVELS = [0, 1, 2, 3]; // OK, WARN, ERROR, STALE

/**
 * Validate one /diagnostics message (owner decision, 30 Oct 2026). The
 * envelope needs a status array; each entry needs a non-empty name and a
 * level of exactly 0, 1, 2 or 3. Invalid entries are ignored and logged.
 * Each valid entry is stored with its own receive time, so a message about
 * one name never refreshes another; a message with no valid entries changes
 * nothing and does not count as received. Entries are kept until the next
 * connection, so an ERROR or STALE report stays latched after it ages out
 * and is replaced only by a later report from the same name.
 */
const reduceDiagnostics = (state, msg, nowMs, wallMs, logs) => {
  if (!msg || typeof msg !== "object" || !Array.isArray(msg.status)) {
    logs.push({
      type: "system",
      severity: "warning",
      message: "Reporting: /diagnostics message ignored (no status array)",
    });
    return state;
  }
  const entries = { ...state.diagnostics.entries };
  let invalid = 0;
  msg.status.forEach((s) => {
    const name = typeof s?.name === "string" ? s.name.trim() : "";
    const level = s?.level;
    if (!name || !Number.isInteger(level) || !DIAG_LEVELS.includes(level)) {
      invalid += 1;
      return;
    }
    entries[name] = {
      level,
      message: typeof s.message === "string" ? s.message : "",
      receivedAt: nowMs,
      receivedWallMs: wallMs,
    };
  });
  if (invalid) {
    logs.push({
      type: "system",
      severity: "warning",
      message: `Reporting: ${invalid} invalid /diagnostics ${invalid === 1 ? "entry" : "entries"} ignored (needs a name and level 0 to 3)`,
    });
  }
  return { ...state, diagnostics: { entries } };
};

/**
 * Apply one source event. Events:
 *   { type: "connection", status }
 *   { type: "lock", msg }                std_msgs/Bool
 *   { type: "navigationStatus", msg }    openamr_nav_msgs/NavigationStatus
 *   { type: "battery", msg }             std_msgs/Float32 percent
 *   { type: "diagnostics", msg }         diagnostic_msgs/DiagnosticArray
 *   { type: "goalStatus", msg }          action_msgs/GoalStatusArray
 *   { type: "navFeedback", msg }         NavigateToPose_FeedbackMessage
 *   { type: "mission", run, name }       shared/missions/missionRunner run
 *   { type: "uiRun", kind, run }         shared/runs/uiRunStore run
 */
export const reduceReporting = (state, event, nowMs, wallMs = null) => {
  const logs = [];
  let next = state;
  switch (event.type) {
    case "connection":
      if (event.status === state.connection) break;
      // A new connection starts with no data: nothing from before the
      // disconnect may look fresh or known, and latched diagnostics faults
      // are cleared (a new connection epoch). The current run and last
      // result stay on screen; stale sources are shown as such.
      next = {
        ...initialReportingState(),
        connection: event.status,
        goal: state.goal,
        seenGoals: state.seenGoals,
        task: state.task,
        result: state.result,
      };
      break;
    case "lock":
      if (typeof event.msg?.data === "boolean") {
        next = { ...state, lock: { lastValue: event.msg.data, lastSampleMs: nowMs } };
      }
      break;
    case "navigationStatus": {
      const invalid = validateNav(event.msg);
      next = { ...state, nav: { msg: invalid ? null : event.msg, receivedAt: nowMs, invalid } };
      break;
    }
    case "battery": {
      const value = event.msg?.data;
      const ok = isFiniteNumber(value) && value >= 0 && value <= 100;
      next = {
        ...state,
        battery: { percent: ok ? value : null, receivedAt: nowMs, invalid: ok ? "" : "battery value out of range" },
      };
      break;
    }
    case "diagnostics":
      next = reduceDiagnostics(state, event.msg, nowMs, wallMs, logs);
      break;
    case "goalStatus":
      next = reduceGoalStatus(state, event.msg, logs);
      break;
    case "navFeedback":
      next = reduceFeedback(state, event.msg);
      break;
    case "mission":
      next = reduceTaskRun(state, "mission", event.run, event.name || "Mission", logs);
      break;
    case "uiRun":
      next = reduceTaskRun(state, event.kind, event.run, event.run?.name || event.kind, logs);
      break;
    default:
      break;
  }
  return { state: next, logs };
};

const fresh = (receivedAt, limit, nowMs) =>
  receivedAt !== null && Number.isFinite(nowMs) && nowMs - receivedAt <= limit;

const freshness = (receivedAt, limit, nowMs, invalid) => {
  if (receivedAt === null) return "No data";
  if (invalid) return "Invalid";
  return fresh(receivedAt, limit, nowMs) ? "Fresh" : "Stale";
};

const DIAG_LEVEL = { OK: 0, WARN: 1, ERROR: 2, STALE: 3 };

const NEXT_ACTION = {
  [DISPLAY.DISCONNECTED]: "Check the panel's network connection. It reconnects on its own.",
  [DISPLAY.UNKNOWN]: "Wait for status data. Do not assume the robot is ready.",
  [DISPLAY.STOPPED_SOFTWARE]: "Check the area, then press Resume on the operator screen.",
  [DISPLAY.FAULTED]: "Call an engineer. Check Health and Events on the operator screen.",
  [DISPLAY.NEEDS_ASSISTANCE]: "An operator is needed at the robot.",
  [DISPLAY.RECOVERING]: "Wait. The robot is trying to recover.",
  [DISPLAY.BLOCKED]: "Clear the path, or cancel the task on the operator screen.",
  [DISPLAY.EXECUTING]: "Wait for completion. Use Stop (software) on the operator screen if needed.",
  [DISPLAY.WAITING]: "Resolve the reason shown before starting a task.",
  [DISPLAY.READY]: "Start a task from the operator screen.",
};

const taskView = (state) => {
  if (state.task) {
    const t = state.task;
    const kindLabel = { mission: "Mission", program: "Program", route: "Route" }[t.kind] || "Task";
    return {
      kind: t.kind,
      label: `${kindLabel}: ${t.name}`,
      step: Number.isInteger(t.step) && Number.isInteger(t.total) && t.total > 0 ? `Step ${t.step} of ${t.total}` : "",
      running: t.status === "running",
    };
  }
  if (state.goal && !GOAL_TERMINAL[state.goal.status]) {
    return { kind: "navigation", label: "Navigation goal", step: "", running: true };
  }
  return null;
};

const progressView = (state) => {
  const goal = state.goal;
  if (!goal || GOAL_TERMINAL[goal.status] || goal.distance === null) return null;
  const pct = goal.distance0 > 0 ? Math.max(0, Math.min(100, Math.round((1 - goal.distance / goal.distance0) * 100))) : 0;
  return {
    percent: pct,
    distanceRemaining: goal.distance,
    etaSeconds: goal.eta,
    recoveries: goal.recoveries,
  };
};

const pad2 = (n) => String(n).padStart(2, "0");

/** Wall-clock time of day (panel local time), for example "09:41:05". */
export const formatClockTime = (wallMs) => {
  if (!isFiniteNumber(wallMs)) return "unknown time";
  const d = new Date(wallMs);
  return `${pad2(d.getHours())}:${pad2(d.getMinutes())}:${pad2(d.getSeconds())}`;
};

const diagFaultText = (name, e) =>
  `Diagnostics ${e.level === DIAG_LEVEL.STALE ? "stale" : "error"}: ${name}${e.message ? `: ${e.message}` : ""}` +
  ` (last error at ${formatClockTime(e.receivedWallMs)})`;

/** Everything a consumer renders, for the time nowMs. */
export const deriveReport = (state, nowMs) => {
  const warnings = [];
  const lockState = state.connection === "connected" ? evaluateLock(state.lock, nowMs) : LOCK_UNKNOWN;
  const navMsg = state.nav.msg;
  const navFresh = Boolean(navMsg) && fresh(state.nav.receivedAt, STALE_AFTER_MS.navigationStatus, nowMs);
  const batteryFresh = state.battery.percent !== null && fresh(state.battery.receivedAt, STALE_AFTER_MS.battery, nowMs);
  const batteryLow = batteryFresh && state.battery.percent <= BATTERY_LOW_PERCENT;
  // Per-entry freshness: an entry older than the threshold drops out.
  const diagAll = Object.entries(state.diagnostics.entries);
  const diagEntries = diagAll.filter(([, e]) => fresh(e.receivedAt, STALE_AFTER_MS.diagnostics, nowMs));
  const diagFresh = diagEntries.length > 0;
  // Latched faults: every name whose last report was ERROR or STALE, fresh
  // or not. Ageing out and reports from other names never clear them.
  const diagFaults = diagAll
    .filter(([, e]) => e.level === DIAG_LEVEL.ERROR || e.level === DIAG_LEVEL.STALE)
    .sort(([, a], [, b]) => b.receivedAt - a.receivedAt);
  const diagWarns = diagEntries.filter(([, e]) => e.level === DIAG_LEVEL.WARN);

  const rows = [
    { label: "Connection", value: state.connection === "connected" ? "Connected" : "Disconnected", freshness: "" },
    {
      label: "Software stop lock",
      value: { locked: "Engaged", released: "Released", unknown: "Unknown" }[lockState],
      freshness: freshness(state.lock.lastSampleMs, STALE_AFTER_MS.stopLock, nowMs, ""),
    },
    {
      label: "Navigation",
      value: navMsg ? (navMsg.navigation_readiness === NAV.READY ? "Ready" : "Not ready") : "Unknown",
      freshness: freshness(state.nav.receivedAt, STALE_AFTER_MS.navigationStatus, nowMs, state.nav.invalid),
    },
    {
      label: "Localization",
      value: navMsg ? { 0: "Unknown", 1: "Lost", 2: "Degraded", 3: "OK" }[navMsg.localization?.state] || "Unknown" : "Unknown",
      freshness: freshness(state.nav.receivedAt, STALE_AFTER_MS.navigationStatus, nowMs, state.nav.invalid),
    },
    {
      label: "Battery",
      value: state.battery.percent === null ? "Unknown" : `${Math.round(state.battery.percent)}%`,
      freshness: freshness(state.battery.receivedAt, STALE_AFTER_MS.battery, nowMs, state.battery.invalid),
    },
    {
      label: "Diagnostics",
      value: diagFaults.length
        ? "Error"
        : !diagFresh
          ? "Unknown"
          : diagWarns.length
            ? "Warning"
            : "OK",
      freshness: diagAll.length === 0 ? "No data" : diagFresh ? "Fresh" : "Stale",
    },
    { label: "Arms", value: "Not reported", freshness: "" },
    { label: "Lift", value: "Not reported", freshness: "" },
    { label: "Physical E-stop", value: "Not reported", freshness: "" },
  ];

  // Warnings: everything worth knowing that is not the main reason.
  rows.forEach((r) => {
    if (r.freshness === "Stale") warnings.push(`${r.label} data is stale`);
    if (r.freshness === "Invalid") warnings.push(`${r.label} data is invalid`);
  });
  if (batteryLow) warnings.push(`Battery low (${Math.round(state.battery.percent)}%)`);
  diagWarns.forEach(([name, s]) => warnings.push(`Diagnostics warning: ${name}${s.message ? `: ${s.message}` : ""}`));
  if (navMsg && navFresh) {
    if (navMsg.health === NAV.HEALTH.DEGRADED) warnings.push("Navigation health is degraded");
    (navMsg.active_reasons || []).forEach((code) => {
      if (code >= 6000 && code < 7000) warnings.push(reasonText(code));
    });
  }
  if (state.result?.invalid) warnings.push(RESULT_UNKNOWN_TEXT);

  const make = (display, reason) => ({
    display,
    reason,
    nextAction:
      display === DISPLAY.NEEDS_ASSISTANCE || display === DISPLAY.WAITING
        ? `${NEXT_ACTION[display]}${reason ? ` (${reason})` : ""}`
        : NEXT_ACTION[display],
    task: taskView(state),
    progress: progressView(state),
    result: state.result,
    completion:
      state.result && state.result.outcome === "SUCCEEDED" && !state.result.invalid
        ? `Completed: ${state.result.name}`
        : "",
    warnings: [...new Set(warnings)],
    rows,
  });

  // Priority, highest first. See docs/reporting-mode.md, "State priority".
  // Owner decision (PR #54): a known, engaged software lock ranks above
  // UNKNOWN navigation data (missing, stale or invalid); only DISCONNECTED
  // ranks higher. An unknown lock is itself UNKNOWN.
  if (state.connection !== "connected") return make(DISPLAY.DISCONNECTED, "No connection to the robot");
  if (lockState === LOCK_LOCKED) return make(DISPLAY.STOPPED_SOFTWARE, "The software stop lock is engaged");
  if (lockState === LOCK_UNKNOWN) return make(DISPLAY.UNKNOWN, "Software stop lock state is unknown");
  // A latched diagnostics fault shows FAULTED whatever the navigation data
  // says; READY is impossible while one is latched.
  if (diagFaults.length) {
    const [name, e] = diagFaults[0];
    diagFaults.slice(1).forEach(([other, o]) => warnings.push(`Also in fault: ${diagFaultText(other, o)}`));
    return make(DISPLAY.FAULTED, diagFaultText(name, e));
  }
  if (state.nav.invalid && state.nav.receivedAt !== null && !navMsg) {
    return make(DISPLAY.UNKNOWN, "Navigation status is invalid");
  }
  if (!navMsg) return make(DISPLAY.UNKNOWN, "No navigation status received yet");
  if (!navFresh) return make(DISPLAY.UNKNOWN, "Navigation status is stale");

  const active = navMsg.active_reasons || [];
  const notReady = navMsg.not_ready_reasons || [];
  const all = [...active, ...notReady];
  const stack = navMsg.stack || {};
  const task = navMsg.task || {};
  const recovery = navMsg.recovery || {};
  const localization = navMsg.localization || {};

  // FAULTED
  if (navMsg.health === NAV.HEALTH.FAULT) {
    return make(DISPLAY.FAULTED, reasonText(all[0]) || "Navigation reports a fault");
  }
  if (stack.state === NAV.STACK.FAILED) return make(DISPLAY.FAULTED, reasonText(stack.reason) || "Navigation failed to start");
  if (stack.state === NAV.STACK.UNKNOWN && stack.reason === 1004) {
    return make(DISPLAY.FAULTED, reasonText(1004));
  }
  const faultCode = all.find((c) => FAULT_REASONS.has(c));
  if (faultCode) return make(DISPLAY.FAULTED, reasonText(faultCode));

  // NEEDS_ASSISTANCE
  const assistCode = all.find((c) => ASSISTANCE_REASONS.has(c));
  if (assistCode) return make(DISPLAY.NEEDS_ASSISTANCE, reasonText(assistCode));
  if (localization.state === NAV.LOC.LOST) {
    return make(DISPLAY.NEEDS_ASSISTANCE, reasonText(localization.reason) || "Robot position is lost");
  }

  // RECOVERING
  if (task.state === NAV.TASK.RECOVERING || (recovery.action && recovery.action !== NAV.RECOVERY_NONE)) {
    const attempt = recovery.attempt_limit ? ` (attempt ${recovery.attempt} of ${recovery.attempt_limit})` : "";
    return make(DISPLAY.RECOVERING, `${reasonText(5001)}${attempt}`);
  }

  // BLOCKED
  const blockedCode = active.find((c) => BLOCKED_REASONS.has(c));
  if (blockedCode) return make(DISPLAY.BLOCKED, reasonText(blockedCode));

  // EXECUTING / WAITING on a run
  const goalActive = state.goal && [GOAL.ACCEPTED, GOAL.EXECUTING, GOAL.CANCELING].includes(state.goal.status);
  if (state.task && state.task.status === "running" && state.task.waitingOn) {
    return make(DISPLAY.WAITING, state.task.waitingOn);
  }
  if (task.state === NAV.TASK.ACTIVE || goalActive || (state.task && state.task.status === "running")) {
    return make(DISPLAY.EXECUTING, goalActive || task.state === NAV.TASK.ACTIVE ? "Driving to the goal" : "Running a task");
  }

  // READY needs every condition; otherwise WAITING with the blocking reason.
  const blocking = [];
  if (navMsg.health !== NAV.HEALTH.OK) blocking.push("Navigation health is not OK");
  if (navMsg.navigation_readiness !== NAV.READY) {
    blocking.push(reasonText(notReady[0]) || "Navigation is not ready");
  }
  if (!diagFresh) blocking.push(diagAll.length ? "Diagnostics are stale" : "No valid diagnostics received");
  if (!batteryFresh) blocking.push(state.battery.receivedAt === null ? "No battery data" : "Battery data is stale or invalid");
  else if (batteryLow) blocking.push(`Battery low (${Math.round(state.battery.percent)}%)`);
  if (blocking.length) return make(DISPLAY.WAITING, blocking[0]);
  return make(DISPLAY.READY, "All reported checks pass");
};
