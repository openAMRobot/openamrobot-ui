// Persisted mission schedules, shared between the headless runner (which fires
// them) and the Scheduler page (which edits them) via a module-level store —
// same pattern as shared/events/eventLog.js.
//
// A schedule fires a simple, client-side navigation action at a wall-clock
// time. This runs only while a browser tab is open — it is NOT robot-side
// autonomy (that would need a backend cron + a way to run programs headless).
// The Scheduler page states this plainly.

const STORAGE_KEY = "openamrSchedules";

let schedules = load();
const listeners = new Set();

function load() {
  try {
    return JSON.parse(localStorage.getItem(STORAGE_KEY) || "[]");
  } catch {
    return [];
  }
}

function persist() {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(schedules));
  } catch {
    // ignore quota errors
  }
}

function emit() {
  listeners.forEach((fn) => fn(schedules));
}

export function getSchedules() {
  return schedules;
}

export function subscribeSchedules(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

export function addSchedule(entry) {
  schedules = [
    ...schedules,
    {
      id: `${Date.now()}`,
      enabled: true,
      repeat: "daily", // 'daily' | 'once'
      lastRunKey: null,
      ...entry,
    },
  ];
  persist();
  emit();
}

export function updateSchedule(id, fields) {
  schedules = schedules.map((s) => (s.id === id ? { ...s, ...fields } : s));
  persist();
  emit();
}

export function removeSchedule(id) {
  schedules = schedules.filter((s) => s.id !== id);
  persist();
  emit();
}
