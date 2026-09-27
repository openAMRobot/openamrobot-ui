// Test-only helpers for the run-state adapter suites. Not imported by app code.
import { SCHEMA_VERSION, TIMEBASE } from "./runStateAdapter";

export const EPOCH_A = "fixture-epoch-7f3c";
// Sorts lexically BEFORE EPOCH_A although it is the newer epoch.
export const EPOCH_B = "fixture-epoch-1a2b";

export const producer = (epoch = EPOCH_A, hb = 1, t = 1000) => ({
  id: "openamr-ui-run-state-fixture",
  epoch,
  timebase: TIMEBASE,
  time_ms: t,
  heartbeat_seq: hb,
});

export const snapshot = ({
  epoch = EPOCH_A,
  hb = 1,
  t = 1000,
  asOf = 0,
  state = null,
  reason = null,
  allowed = null,
  run = null,
} = {}) => ({
  schema_version: SCHEMA_VERSION,
  kind: "snapshot",
  fixture: true,
  producer: producer(epoch, hb, t),
  robot: { robot_id: "fixture-robot-01", config_id: "fixture-config-a" },
  profile: "fixture-profile-default",
  as_of_seq: asOf,
  operational_state: state,
  reason,
  allowed_operations: allowed,
  run,
});

export const event = (seq, type, runId, patch, epoch = EPOCH_A) => ({
  run_id: runId,
  epoch,
  seq,
  type,
  producer_time_ms: 0,
  patch,
});

export const events = (list, { epoch = EPOCH_A, hb = 1, t = 1000 } = {}) => ({
  schema_version: SCHEMA_VERSION,
  kind: "events",
  fixture: true,
  producer: producer(epoch, hb, t),
  events: list,
});

export const created = (seq, runId, total = 3, epoch = EPOCH_A) =>
  event(
    seq,
    "RUN_CREATED",
    runId,
    {
      run: {
        run_id: runId,
        mission_id: "fixture-mission-a",
        step: null,
        progress: { steps_completed: 0, steps_total: total },
        outcome: null,
      },
      operational_state: "READY",
      reason: null,
      allowed_operations: ["FIXTURE_PROVISIONAL.START"],
    },
    epoch,
  );

export const started = (seq, runId, atMs, total = 3, epoch = EPOCH_A) =>
  event(
    seq,
    "RUN_STARTED",
    runId,
    {
      run: {
        step: { step_id: "step-1", index: 0, count: total },
        progress: { steps_completed: 0, steps_total: total },
        started_at_ms: atMs,
      },
      operational_state: "EXECUTING",
      reason: null,
      allowed_operations: ["FIXTURE_PROVISIONAL.CANCEL"],
    },
    epoch,
  );

export const ended = (seq, runId, atMs, result, state, reason = null, epoch = EPOCH_A) =>
  event(
    seq,
    "RUN_ENDED",
    runId,
    {
      run: { outcome: { result, reason }, ended_at_ms: atMs },
      operational_state: state,
      reason: null,
      allowed_operations: [],
    },
    epoch,
  );

export const STEP_FAILED = {
  registry: "FIXTURE_PROVISIONAL",
  code: "FIXTURE_PROVISIONAL.STEP_FAILED",
};

/** Controllable timers so tests can count pending polls (leak checks). */
export function createFakeTimers() {
  let nextId = 1;
  const pending = new Map();
  return {
    setTimeout(fn, ms) {
      const id = nextId++;
      pending.set(id, { fn, ms });
      return id;
    },
    clearTimeout(id) {
      pending.delete(id);
    },
    pendingCount: () => pending.size,
    runAll() {
      const due = [...pending.entries()];
      pending.clear();
      due.forEach(([, t]) => t.fn());
    },
  };
}

export const flush = async () => {
  for (let i = 0; i < 20; i += 1) {
    // eslint-disable-next-line no-await-in-loop
    await Promise.resolve();
  }
};

/**
 * In-memory producer for controller tests: mirrors the Flask fixture's
 * snapshot/events semantics (epoch-scoped seq, 409 on obsolete epoch).
 */
export function createFakeProducer() {
  const p = {
    epoch: EPOCH_A,
    hb: 1,
    t: 1000,
    log: [],
    snap: { state: null, reason: null, allowed: null, run: null },
    mode: "ok", // ok | down | disabled
    calls: { snapshot: 0, events: 0 },
    push(e) {
      const run = e.type === "RUN_CREATED" ? {} : { ...p.snap.run };
      const rp = e.patch.run || {};
      Object.assign(run, rp);
      run.run_id = e.run_id;
      p.snap = {
        state: e.patch.operational_state,
        reason: e.patch.reason,
        allowed: e.patch.allowed_operations,
        run,
      };
      p.log.push(e);
    },
    restart(epoch) {
      p.epoch = epoch;
      p.log = [];
      p.snap = { state: null, reason: null, allowed: null, run: p.snap.run && { run_id: p.snap.run.run_id } };
    },
    seq: () => p.log.length,
    transport: {
      fetchSnapshot: async () => {
        p.calls.snapshot += 1;
        if (p.mode === "down") throw new Error("Failed to fetch");
        if (p.mode === "disabled") return { status: 404, body: { code: 404 } };
        return {
          status: 200,
          body: snapshot({ epoch: p.epoch, hb: p.hb, t: p.t, asOf: p.log.length, ...p.snap }),
        };
      },
      fetchEvents: async ({ epoch, afterSeq }) => {
        p.calls.events += 1;
        if (p.mode === "down") throw new Error("Failed to fetch");
        if (p.mode === "disabled") return { status: 404, body: { code: 404 } };
        if (epoch !== p.epoch) return { status: 409, body: { code: 409 } };
        return {
          status: 200,
          body: events(p.log.filter((e) => e.seq > afterSeq), { epoch: p.epoch, hb: p.hb, t: p.t }),
        };
      },
    },
  };
  return p;
}
