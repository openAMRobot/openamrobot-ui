// Per-group freshness of /joint_states, derived from the rate the robot
// actually publishes at. No nominal rate is assumed: until enough messages
// have arrived to measure one, a group reads "measuring".
//
// Freshness is about the data only. "Updating" means positions are arriving;
// it never means a controller is healthy, the arm is ready or motion is safe.

export const RATE_WINDOW = 20; // arrivals kept per group
export const MIN_INTERVALS = 2; // intervals needed before a rate is reported
export const STALE_PERIODS = 3; // stale after this many measured periods with no update
// rosbridge and the browser add jitter of their own; a group published faster
// than this is still only called stale after this long.
export const STALE_FLOOR_MS = 250;

function median(values) {
  const sorted = [...values].sort((a, b) => a - b);
  const mid = Math.floor(sorted.length / 2);
  return sorted.length % 2 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
}

export function createFreshnessTracker(groups) {
  const arrivals = new Map(groups.map((g) => [g.id, []]));
  const groupOf = new Map();
  groups.forEach((g) => g.joints.forEach((j) => groupOf.set(j, g.id)));

  return {
    /** Record one JointState message received at nowMs. */
    record(jointNames, nowMs) {
      const touched = new Set();
      (jointNames || []).forEach((name) => {
        const id = groupOf.get(name);
        if (id) touched.add(id);
      });
      touched.forEach((id) => {
        const list = arrivals.get(id);
        list.push(nowMs);
        if (list.length > RATE_WINDOW) list.shift();
      });
    },

    /** Status of one group at nowMs. */
    status(groupId, nowMs) {
      const list = arrivals.get(groupId) || [];
      if (list.length === 0) return { state: "not_reported", periodMs: null, ageMs: null, staleAfterMs: null };
      const ageMs = nowMs - list[list.length - 1];
      const intervals = list.slice(1).map((t, i) => t - list[i]);
      if (intervals.length < MIN_INTERVALS) {
        return { state: "measuring", periodMs: null, ageMs, staleAfterMs: null };
      }
      const periodMs = median(intervals);
      const staleAfterMs = Math.max(STALE_PERIODS * periodMs, STALE_FLOOR_MS);
      return { state: ageMs > staleAfterMs ? "stale" : "updating", periodMs, ageMs, staleAfterMs };
    },
  };
}
