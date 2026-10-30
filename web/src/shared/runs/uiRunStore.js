/**
 * Browser-side task runs that have no robot-side producer yet: Blockly
 * programs (BlocksPage) and the map page waypoint queue (route). Missions
 * keep their own store in shared/missions/missionRunner.js.
 *
 * Writers set the run; readers (the reporting adapter) only subscribe. A
 * run here is what this browser tab is running. It does not establish what
 * the robot did: execution belongs to the robot-side executor (see
 * docs/proposals/i5-run-state.md for the proposed robot-side envelope).
 *
 * Run shape: { runId, kind: "program" | "route", name, step, total,
 *              status: "running" | "succeeded" | "failed" | "stopped",
 *              reason }
 */

const runs = { program: null, route: null };
const listeners = new Set();
let counter = 0;

export const newUiRunId = (kind) => {
  counter += 1;
  return `${kind}:${Date.now()}:${counter}`;
};

export const getUiRun = (kind) => runs[kind];

export const setUiRun = (kind, run) => {
  runs[kind] = run;
  listeners.forEach((fn) => fn(kind, run));
};

// A finished run keeps its outcome: later updates (a Stop after the queue
// already completed, a late step) do not change it.
export const updateUiRun = (kind, patch) => {
  if (!runs[kind] || runs[kind].status !== "running") return;
  setUiRun(kind, { ...runs[kind], ...patch });
};

export const subscribeUiRuns = (fn) => {
  listeners.add(fn);
  return () => listeners.delete(fn);
};
