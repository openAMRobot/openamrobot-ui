// Persisted, app-wide event log. A single module-level store (not React
// state) so a headless recorder mounted once in AppLayout can append events
// while any page subscribes to read them — the same "single source of truth
// shared across pages" idea as useSavedWaypoints, but push-based because the
// writer and readers live in different parts of the tree.

const STORAGE_KEY = "openamrEventLog";
const MAX_EVENTS = 500;

let events = load();
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
    localStorage.setItem(STORAGE_KEY, JSON.stringify(events));
  } catch {
    // Quota or private-mode failure is non-fatal — the in-memory log still works.
  }
}

function emit() {
  listeners.forEach((fn) => fn(events));
}

export const EVENT_SEVERITY = {
  info: { label: "Info", color: "text-statusBlue", dot: "bg-statusBlue" },
  success: { label: "OK", color: "text-statusGreen", dot: "bg-statusGreen" },
  warning: { label: "Warn", color: "text-statusYellow", dot: "bg-statusYellow" },
  error: { label: "Error", color: "text-statusRed", dot: "bg-statusRed" },
};

/**
 * Append an event. `dedupeKey` suppresses a repeat of the same logical event
 * (e.g. the same nav goal reaching the same terminal state) when it matches
 * the most recent entry's key — callers pass a stable key to avoid spamming
 * the log from high-rate status topics.
 */
export function addEvent({ type, severity = "info", message, dedupeKey }) {
  const last = events[0];
  if (dedupeKey && last && last.dedupeKey === dedupeKey) return;
  const entry = {
    id: `${Date.now()}-${Math.round(performance.now())}`,
    ts: new Date().toISOString(),
    type,
    severity,
    message,
    dedupeKey,
  };
  events = [entry, ...events].slice(0, MAX_EVENTS);
  persist();
  emit();
}

export function clearEvents() {
  events = [];
  persist();
  emit();
}

export function getEvents() {
  return events;
}

export function subscribeEvents(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}
