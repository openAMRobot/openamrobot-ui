// Shared "mission run" state + a small command channel between whoever asks
// for a mission to run (MissionsPage's Run button, SchedulerRunner firing a
// mission-type schedule) and the one component that actually drives it
// (headless MissionRunner.jsx, mounted once in AppLayout — same reasoning as
// SchedulerRunner: ROS subscriptions should exist exactly once, not once per
// caller). Same module-level store + listeners pattern as schedules.js.

let run = null; // { missionId, stepIndex, status: 'running'|'succeeded'|'failed'|'stopped', log: [{label, ok, time}] } | null
const runListeners = new Set();

let pendingCommand = null;
const commandListeners = new Set();

export function getRun() {
  return run;
}

export function subscribeRun(fn) {
  runListeners.add(fn);
  return () => runListeners.delete(fn);
}

export function setRun(next) {
  run = next;
  runListeners.forEach((fn) => fn(run));
}

export function requestStart(missionId) {
  pendingCommand = { type: "start", missionId, key: Date.now() };
  commandListeners.forEach((fn) => fn(pendingCommand));
}

export function requestStop() {
  pendingCommand = { type: "stop", key: Date.now() };
  commandListeners.forEach((fn) => fn(pendingCommand));
}

export function subscribeCommands(fn) {
  commandListeners.add(fn);
  return () => commandListeners.delete(fn);
}
