// Freshness of /joint_states, tracked per required joint and derived from the
// rate the robot actually publishes each joint at. No nominal rate is assumed:
// until enough valid samples have arrived to measure one, a joint reads
// "measuring".
//
// A group is "updating" only when every one of its joints has a finite,
// matching position inside that joint's own stale window. A message that
// carries some joints of a group refreshes those joints only.
//
// Freshness is about the data only. "Updating" means positions are arriving;
// it never means a controller is healthy, the arm is ready or motion is safe.

export const RATE_WINDOW = 20; // valid arrivals kept per joint
export const MIN_INTERVALS = 2; // intervals needed before a rate is reported
export const STALE_PERIODS = 3; // stale after this many measured periods with no update
// rosbridge and the browser add jitter of their own; a joint published faster
// than this is still only called stale after this long.
export const STALE_FLOOR_MS = 250;

function median(values) {
  const sorted = [...values].sort((a, b) => a - b);
  const mid = Math.floor(sorted.length / 2);
  return sorted.length % 2 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
}

/** True only for a finite number: NaN, Infinity and non-numbers are not positions. */
export const isFinitePosition = (value) => typeof value === "number" && Number.isFinite(value);

/**
 * Split one JointState into valid positions and invalid joint names. A
 * position array whose length does not match the name array cannot be
 * matched to names, so every named joint in that message is invalid.
 */
export function readJointState(msg) {
  const names = Array.isArray(msg?.name) ? msg.name : [];
  const positions = Array.isArray(msg?.position) ? msg.position : [];
  const matched = positions.length === names.length;
  const valid = {};
  const invalid = [];
  names.forEach((name, i) => {
    if (typeof name !== "string") return;
    if (matched && isFinitePosition(positions[i])) valid[name] = positions[i];
    else invalid.push(name);
  });
  return { valid, invalid };
}

function jointStatus(entry, nowMs) {
  if (!entry || (entry.arrivals.length === 0 && !entry.invalid)) {
    return { state: "not_reported", periodMs: null, ageMs: null, staleAfterMs: null };
  }
  if (entry.invalid) return { state: "invalid", periodMs: null, ageMs: null, staleAfterMs: null };
  const list = entry.arrivals;
  const ageMs = nowMs - list[list.length - 1];
  const intervals = list.slice(1).map((t, i) => t - list[i]);
  if (intervals.length < MIN_INTERVALS) return { state: "measuring", periodMs: null, ageMs, staleAfterMs: null };
  const periodMs = median(intervals);
  const staleAfterMs = Math.max(STALE_PERIODS * periodMs, STALE_FLOOR_MS);
  return { state: ageMs > staleAfterMs ? "stale" : "updating", periodMs, ageMs, staleAfterMs };
}

export function createFreshnessTracker(groups) {
  const joints = new Map();
  groups.forEach((g) => g.joints.forEach((j) => joints.set(j, { arrivals: [], invalid: false })));

  return {
    /**
     * Record one JointState at nowMs: `valid` maps joint name to a finite
     * position, `invalid` lists joints whose position was missing or not
     * finite. Only those joints change.
     */
    record({ valid = {}, invalid = [] }, nowMs) {
      Object.keys(valid).forEach((name) => {
        const entry = joints.get(name);
        if (!entry) return;
        entry.invalid = false;
        entry.arrivals.push(nowMs);
        if (entry.arrivals.length > RATE_WINDOW) entry.arrivals.shift();
      });
      invalid.forEach((name) => {
        const entry = joints.get(name);
        if (entry) entry.invalid = true;
      });
    },

    jointStatus(name, nowMs) {
      return jointStatus(joints.get(name), nowMs);
    },

    /**
     * Status of one group at nowMs, from every joint in it:
     * not_reported (no joint ever reported), updating (all joints updating),
     * measuring (all joints valid and fresh enough, some still measuring),
     * partial (some joints fresh, others missing, invalid or stale) or
     * stale (no joint fresh). missing, invalid and stale name the joints.
     */
    status(groupId, nowMs) {
      const group = groups.find((g) => g.id === groupId);
      const perJoint = (group?.joints || []).map((name) => ({ name, ...jointStatus(joints.get(name), nowMs) }));
      const named = (state) => perJoint.filter((j) => j.state === state).map((j) => j.name);
      const missing = named("not_reported");
      const invalid = named("invalid");
      const stale = named("stale");
      const fresh = perJoint.filter((j) => j.state === "updating" || j.state === "measuring");
      const periods = perJoint.filter((j) => j.periodMs !== null).map((j) => j.periodMs);
      const base = { missing, invalid, stale, periodMs: periods.length ? Math.max(...periods) : null };

      if (missing.length === perJoint.length) return { ...base, state: "not_reported" };
      if (fresh.length === perJoint.length) {
        return { ...base, state: fresh.every((j) => j.state === "updating") ? "updating" : "measuring" };
      }
      if (fresh.length > 0) return { ...base, state: "partial" };
      const ages = perJoint.filter((j) => j.ageMs !== null).map((j) => j.ageMs);
      return { ...base, state: "stale", ageMs: ages.length ? Math.min(...ages) : null };
    },
  };
}
