// Persisted mission definitions — an ordered list of steps a mission runner
// executes in sequence. Same module-level store + listeners pattern as
// shared/schedules/schedules.js and shared/events/eventLog.js, so both the
// authoring page and the headless runner (see missionRunner.js/MissionRunner.jsx)
// read the same source of truth.
//
// Step shapes:
//   { id, type: "waypoint", waypointId }
//   { id, type: "home" }
//   { id, type: "wait", seconds }
//   { id, type: "dock" }
//   { id, type: "undock" }

const STORAGE_KEY = "openamrMissions";

let missions = load();
const listeners = new Set();

function load() {
  try {
    const stored = JSON.parse(localStorage.getItem(STORAGE_KEY) || "[]");
    return Array.isArray(stored) ? stored : [];
  } catch {
    return [];
  }
}

function persist() {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(missions));
  } catch {
    // ignore quota errors
  }
}

function emit() {
  listeners.forEach((fn) => fn(missions));
}

export function getMissions() {
  return missions;
}

export function getMission(id) {
  return missions.find((m) => m.id === id) || null;
}

export function subscribeMissions(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

export function addMission(name) {
  const mission = { id: `${Date.now()}`, name, steps: [] };
  missions = [...missions, mission];
  persist();
  emit();
  return mission;
}

export function removeMission(id) {
  missions = missions.filter((m) => m.id !== id);
  persist();
  emit();
}

export function renameMission(id, name) {
  missions = missions.map((m) => (m.id === id ? { ...m, name } : m));
  persist();
  emit();
}

export function setSteps(missionId, steps) {
  missions = missions.map((m) => (m.id === missionId ? { ...m, steps } : m));
  persist();
  emit();
}

export function addStep(missionId, step) {
  const mission = getMission(missionId);
  if (!mission) return;
  setSteps(missionId, [...mission.steps, { id: `${Date.now()}`, ...step }]);
}

export function removeStep(missionId, stepId) {
  const mission = getMission(missionId);
  if (!mission) return;
  setSteps(missionId, mission.steps.filter((s) => s.id !== stepId));
}

export function moveStep(missionId, stepId, direction) {
  const mission = getMission(missionId);
  if (!mission) return;
  const idx = mission.steps.findIndex((s) => s.id === stepId);
  const swapWith = idx + direction;
  if (idx < 0 || swapWith < 0 || swapWith >= mission.steps.length) return;
  const next = [...mission.steps];
  [next[idx], next[swapWith]] = [next[swapWith], next[idx]];
  setSteps(missionId, next);
}
