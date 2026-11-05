import { DISPLAY } from "./reportingAdapter";

/**
 * What Reporting Mode speaks, decided without the speech engine so it can
 * be tested. Each state change, result, warning and completion is spoken
 * once. A disconnect and reconnect does not repeat anything: returning to
 * the state that was spoken before the interruption is silent, and results,
 * warnings and completions are remembered by key for the page's lifetime.
 */

// States that interrupt the stable state without replacing it.
const INTERRUPTIONS = new Set([DISPLAY.DISCONNECTED, DISPLAY.UNKNOWN]);

export const createAnnouncementMemory = () => ({
  lastStable: null,
  lastSpokenState: null,
  spoken: new Set(),
});

const stateText = (view) => `${view.display.replace(/_/g, " ").toLowerCase()}. ${view.reason}`;

/** Mark everything currently shown as already spoken (used when speech is switched on). */
export const primeMemory = (memory, view) => {
  if (!view) return memory;
  if (INTERRUPTIONS.has(view.display)) memory.lastSpokenState = view.display;
  else {
    memory.lastStable = view.display;
    memory.lastSpokenState = view.display;
  }
  if (view.result) memory.spoken.add(`result:${view.result.runId}:${view.result.outcome}`);
  if (view.completion && view.result) memory.spoken.add(`completion:${view.result.runId}`);
  view.warnings.forEach((w) => memory.spoken.add(`warning:${w}`));
  return memory;
};

/** Returns the texts to speak for this view, in order, and updates memory. */
export const nextAnnouncements = (memory, view) => {
  if (!view) return [];
  const out = [];

  if (view.display !== memory.lastSpokenState) {
    const isInterruption = INTERRUPTIONS.has(view.display);
    const backToSame = !isInterruption && view.display === memory.lastStable;
    if (!backToSame) out.push(stateText(view));
    memory.lastSpokenState = view.display;
    if (!isInterruption) memory.lastStable = view.display;
  }

  if (view.result) {
    const key = `result:${view.result.runId}:${view.result.outcome}`;
    if (!memory.spoken.has(key)) {
      memory.spoken.add(key);
      const outcome = view.result.invalid ? "result unknown" : view.result.outcome.toLowerCase();
      out.push(`${view.result.name}: ${outcome}.${view.result.reason ? ` ${view.result.reason}.` : ""}`);
    }
  }

  view.warnings.forEach((w) => {
    const key = `warning:${w}`;
    if (!memory.spoken.has(key)) {
      memory.spoken.add(key);
      out.push(`Warning: ${w}.`);
    }
  });

  if (view.completion && view.result) {
    const key = `completion:${view.result.runId}`;
    if (!memory.spoken.has(key)) {
      memory.spoken.add(key);
      out.push(view.completion);
    }
  }

  return out;
};
