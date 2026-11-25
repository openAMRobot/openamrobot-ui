/**
 * Abort channel for UI-side programs and missions. Stop (software) and a
 * stop lock that is locked or unknown call abortUiMotion(); BlocksPage and
 * MissionRunner subscribe and stop their runs. Same module-level listener
 * pattern as shared/missions/missionRunner.js.
 */

const listeners = new Set();

export const subscribeAbort = (fn) => {
  listeners.add(fn);
  return () => listeners.delete(fn);
};

export const abortUiMotion = (reason) => {
  listeners.forEach((fn) => {
    try {
      fn(reason);
    } catch (err) {
      console.error("abort listener failed", err);
    }
  });
};
