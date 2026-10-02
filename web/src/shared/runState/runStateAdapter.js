// Consumer-neutral run-state adapter — PROPOSED, fixture-only, pending owner
// review (docs/proposals/i5-run-state-fixture.md).
//
// One adapter instance is shared by every consumer (the current-run view
// now, a full-screen Reporting route later). It owns the only polling loop,
// so N subscribers cause exactly one set of transport calls; the first
// subscribe starts polling and the last unsubscribe stops it. The adapter
// only reads — it has no command path of any kind.
//
// Rules implemented here (all proposal, none an accepted contract):
//   - Dedup identity is (run_id, producer epoch, seq); order is seq within
//     one epoch. The stable event id `${epoch}:${seq}` is derived, not sent.
//   - Epochs are compared for equality only. A different epoch means the
//     producer restarted: drop the event cursor, re-snapshot, reject every
//     event still carrying the old epoch.
//   - A snapshot older than what events already applied (same epoch,
//     as_of_seq < appliedSeq) is ignored — the snapshot/event race.
//   - A terminal outcome is latched per run_id. Nothing later (event,
//     snapshot, restart) can clear or change it; contradictions are counted.
//   - Freshness is derived at read time from a local monotonic receipt time
//     of the last heartbeat change. It is never a stored boolean, so a
//     stopped or hung connection cannot keep looking fresh. Producer and
//     browser clocks are never compared.
//   - Anything missing, malformed or of an unknown schema_version is shown
//     as unknown/invalid, never as healthy. Readiness and allowed
//     operations are never inferred; they are only passed through, and only
//     while the data is fresh.

export const SCHEMA_VERSION = "openamr.run_state.fixture/0.1-proposed";
export const TIMEBASE = "producer_monotonic_ms";

export const OPERATIONAL_STATES = [
  "READY",
  "EXECUTING",
  "WAITING",
  "BLOCKED",
  "RECOVERING",
  "NEEDS_ASSISTANCE",
  "SAFETY_STOPPED",
  "FAULTED",
];
export const TERMINAL_RESULTS = ["SUCCEEDED", "FAILED", "CANCELED"];

export const CONNECTION = {
  IDLE: "IDLE", // no subscribers, not polling
  CONNECTING: "CONNECTING",
  CONNECTED: "CONNECTED",
  DISCONNECTED: "DISCONNECTED",
  DISABLED: "DISABLED", // fixture backend answered 404 (flag off)
  INVALID: "INVALID", // answered, but the payload failed validation
};

export const FRESHNESS = { FRESH: "FRESH", STALE: "STALE", UNKNOWN: "UNKNOWN" };

export const DEFAULT_OPTIONS = { pollMs: 1000, staleAfterMs: 3000 };

export function initialRunState() {
  return {
    connection: CONNECTION.IDLE,
    lastError: null,
    validity: "UNKNOWN",
    invalidReason: null,
    schemaVersion: null,
    producerId: null,
    epoch: null,
    appliedSeq: null,
    needsSnapshot: true,
    heartbeatSeq: null,
    producerTimeMs: null,
    lastHeartbeatChangeAt: null, // local monotonic ms
    lastGoodAt: null, // local monotonic ms
    robot: null,
    profile: null,
    operationalState: null,
    reason: null,
    allowedOperations: null,
    run: null,
    terminalOutcomes: {}, // run_id -> latched outcome
    counters: {
      duplicatesDropped: 0,
      eventsRejected: 0,
      staleSnapshotsIgnored: 0,
      terminalConflicts: 0,
      producerRestarts: 0,
      runChanges: 0,
      obsoleteResponsesDropped: 0,
    },
  };
}

const isObj = (v) => v !== null && typeof v === "object" && !Array.isArray(v);
const isStr = (v) => typeof v === "string" && v.length > 0;

export function validateEnvelope(body, kind) {
  if (!isObj(body)) return "payload is not an object";
  if (body.schema_version !== SCHEMA_VERSION) {
    return `unsupported schema_version ${JSON.stringify(body.schema_version)}`;
  }
  if (body.kind !== kind) return `expected kind ${kind}`;
  const p = body.producer;
  if (!isObj(p) || !isStr(p.epoch)) return "missing producer.epoch";
  if (!isStr(p.id)) return "missing producer.id";
  if (p.timebase !== TIMEBASE) return "unknown producer.timebase";
  if (!Number.isFinite(p.time_ms)) return "missing producer.time_ms";
  if (!Number.isInteger(p.heartbeat_seq)) return "missing producer.heartbeat_seq";
  if (kind === "snapshot") {
    if (!Number.isInteger(body.as_of_seq) || body.as_of_seq < 0) return "missing as_of_seq";
    if (!("run" in body)) return "missing run";
    if (body.run !== null && (!isObj(body.run) || !isStr(body.run.run_id))) {
      return "run without run_id";
    }
  } else {
    if (!Array.isArray(body.events)) return "missing events";
    for (const e of body.events) {
      if (!isObj(e) || !Number.isInteger(e.seq) || !isStr(e.epoch) || !isStr(e.run_id)
        || !isStr(e.type) || !isObj(e.patch)) {
        return "malformed event";
      }
    }
  }
  return null;
}

const normalizeOutcome = (outcome) => {
  if (!isObj(outcome)) return null;
  const result = TERMINAL_RESULTS.includes(outcome.result) ? outcome.result : "UNKNOWN";
  return { result, reason: isObj(outcome.reason) ? outcome.reason : null };
};

const sameOutcome = (a, b) => JSON.stringify(a) === JSON.stringify(b);

const bump = (state, key) => ({
  ...state,
  counters: { ...state.counters, [key]: state.counters[key] + 1 },
});

function noteProducer(state, producer, receivedAt) {
  const changed =
    producer.epoch !== state.epoch ||
    producer.heartbeat_seq !== state.heartbeatSeq ||
    state.lastHeartbeatChangeAt === null;
  return {
    ...state,
    producerId: producer.id,
    heartbeatSeq: producer.heartbeat_seq,
    producerTimeMs: producer.time_ms,
    lastHeartbeatChangeAt: changed ? receivedAt : state.lastHeartbeatChangeAt,
    lastGoodAt: receivedAt,
    connection: CONNECTION.CONNECTED,
    lastError: null,
    validity: "VALID",
    invalidReason: null,
  };
}

// Applies an outcome for runId while enforcing terminal non-regression.
function withOutcome(state, runId, incoming) {
  const latched = state.terminalOutcomes[runId];
  if (latched) {
    // Latched outcome wins. A missing incoming outcome (e.g. after a producer
    // restart) keeps it silently; a different one is counted as a conflict.
    const conflict = incoming && !sameOutcome(incoming, latched);
    return { state: conflict ? bump(state, "terminalConflicts") : state, outcome: latched };
  }
  if (incoming) {
    return {
      state: { ...state, terminalOutcomes: { ...state.terminalOutcomes, [runId]: incoming } },
      outcome: incoming,
    };
  }
  return { state, outcome: null };
}

export function markInvalid(state, reason) {
  return { ...state, validity: "INVALID", invalidReason: reason, connection: CONNECTION.INVALID };
}

export function applySnapshot(state, body, receivedAt) {
  const invalid = validateEnvelope(body, "snapshot");
  if (invalid) return markInvalid(state, invalid);

  const epoch = body.producer.epoch;
  if (
    epoch === state.epoch &&
    state.appliedSeq !== null &&
    body.as_of_seq < state.appliedSeq
  ) {
    return { ...bump(state, "staleSnapshotsIgnored"), needsSnapshot: false };
  }

  let next = state;
  if (state.epoch !== null && epoch !== state.epoch) next = bump(next, "producerRestarts");
  const incomingRunId = body.run ? body.run.run_id : null;
  if (state.run && incomingRunId !== state.run.runId) next = bump(next, "runChanges");

  let run = null;
  if (body.run) {
    const r = body.run;
    const { state: s2, outcome } = withOutcome(next, r.run_id, normalizeOutcome(r.outcome));
    next = s2;
    const keepPrior = state.run && state.run.runId === r.run_id;
    const hasStart = Number.isFinite(r.started_at_ms);
    run = {
      runId: r.run_id,
      missionId: isStr(r.mission_id) ? r.mission_id : null,
      step: isObj(r.step) ? r.step : null,
      progress: isObj(r.progress) ? r.progress : null,
      outcome,
      startedAtMs: hasStart ? r.started_at_ms : null,
      endedAtMs: Number.isFinite(r.ended_at_ms) ? r.ended_at_ms : null,
      timeEpoch: hasStart ? epoch : null,
    };
    if (keepPrior && !hasStart && state.run.timeEpoch === epoch) {
      run.startedAtMs = state.run.startedAtMs;
      run.timeEpoch = state.run.timeEpoch;
    }
  }

  next = noteProducer(next, body.producer, receivedAt);
  return {
    ...next,
    schemaVersion: body.schema_version,
    epoch,
    appliedSeq: body.as_of_seq,
    needsSnapshot: false,
    robot: isObj(body.robot) ? body.robot : null,
    profile: isStr(body.profile) ? body.profile : null,
    operationalState: body.operational_state ?? null,
    reason: isObj(body.reason) ? body.reason : null,
    allowedOperations: Array.isArray(body.allowed_operations) ? body.allowed_operations : null,
    run,
  };
}

function applyOneEvent(state, e) {
  const patch = e.patch;
  const latched = state.terminalOutcomes[e.run_id];
  if (latched) {
    // A terminal run is closed: nothing may resurrect or rewrite it.
    return bump(bump(state, "eventsRejected"), "terminalConflicts");
  }
  let next = state;
  let run = state.run;
  if (e.type === "RUN_CREATED") {
    if (run && run.runId !== e.run_id) next = bump(next, "runChanges");
    run = {
      runId: e.run_id,
      missionId: null,
      step: null,
      progress: null,
      outcome: null,
      startedAtMs: null,
      endedAtMs: null,
      timeEpoch: null,
    };
  } else if (!run || run.runId !== e.run_id) {
    return bump(state, "eventsRejected");
  }
  const rp = isObj(patch.run) ? patch.run : {};
  run = { ...run };
  if ("mission_id" in rp) run.missionId = isStr(rp.mission_id) ? rp.mission_id : null;
  if ("step" in rp) run.step = isObj(rp.step) ? rp.step : null;
  if ("progress" in rp) run.progress = isObj(rp.progress) ? rp.progress : null;
  if (Number.isFinite(rp.started_at_ms)) {
    run.startedAtMs = rp.started_at_ms;
    run.timeEpoch = e.epoch;
  }
  if (Number.isFinite(rp.ended_at_ms)) run.endedAtMs = rp.ended_at_ms;
  if ("outcome" in rp) {
    const { state: s2, outcome } = withOutcome(next, e.run_id, normalizeOutcome(rp.outcome));
    next = s2;
    run.outcome = outcome;
  }
  return {
    ...next,
    run,
    operationalState: "operational_state" in patch ? patch.operational_state ?? null : next.operationalState,
    reason: "reason" in patch ? (isObj(patch.reason) ? patch.reason : null) : next.reason,
    allowedOperations:
      "allowed_operations" in patch
        ? Array.isArray(patch.allowed_operations)
          ? patch.allowed_operations
          : null
        : next.allowedOperations,
  };
}

export function applyEvents(state, body, receivedAt) {
  const invalid = validateEnvelope(body, "events");
  if (invalid) return markInvalid(state, invalid);
  if (state.epoch === null || body.producer.epoch !== state.epoch) {
    // Obsolete (or foreign) epoch: reject the batch and re-snapshot.
    return { ...bump(state, "eventsRejected"), needsSnapshot: true };
  }
  let next = state;
  const events = [...body.events].sort((a, b) => a.seq - b.seq);
  for (const e of events) {
    if (e.epoch !== next.epoch) {
      next = bump(next, "eventsRejected");
      continue;
    }
    if (e.seq <= next.appliedSeq) {
      next = bump(next, "duplicatesDropped");
      continue;
    }
    if (e.seq !== next.appliedSeq + 1) {
      next = { ...next, needsSnapshot: true }; // gap: resync from a snapshot
      break;
    }
    next = { ...applyOneEvent(next, e), appliedSeq: e.seq };
  }
  return noteProducer(next, body.producer, receivedAt);
}

// Read-time derivation. `now` is the same local monotonic clock used for
// receivedAt; the producer clock is only ever subtracted from itself.
export function deriveView(state, now, options = DEFAULT_OPTIONS) {
  const { staleAfterMs } = { ...DEFAULT_OPTIONS, ...options };
  const hadData = state.lastHeartbeatChangeAt !== null;
  let freshness = FRESHNESS.UNKNOWN;
  if (state.validity === "VALID" && hadData) {
    freshness =
      state.connection === CONNECTION.CONNECTED && now - state.lastHeartbeatChangeAt <= staleAfterMs
        ? FRESHNESS.FRESH
        : FRESHNESS.STALE;
  } else if (hadData) {
    freshness = FRESHNESS.STALE;
  }
  const healthy =
    state.connection === CONNECTION.CONNECTED && state.validity === "VALID" && freshness === FRESHNESS.FRESH;

  const run = state.run;
  let elapsedMs = null;
  if (run && run.startedAtMs !== null && run.timeEpoch === state.epoch) {
    const end = run.endedAtMs ?? state.producerTimeMs;
    if (Number.isFinite(end) && end >= run.startedAtMs) elapsedMs = end - run.startedAtMs;
  }

  return {
    connection: state.connection,
    lastError: state.lastError,
    validity: state.validity,
    invalidReason: state.invalidReason,
    freshness,
    healthy,
    ageMs: state.lastGoodAt === null ? null : Math.max(0, now - state.lastGoodAt),
    heartbeatAgeMs: hadData ? Math.max(0, now - state.lastHeartbeatChangeAt) : null,
    producer: { id: state.producerId, epoch: state.epoch, timeMs: state.producerTimeMs },
    appliedSeq: state.appliedSeq,
    robot: state.robot,
    profile: state.profile,
    operationalState: OPERATIONAL_STATES.includes(state.operationalState)
      ? state.operationalState
      : "UNKNOWN",
    rawOperationalState: state.operationalState,
    reason: state.reason,
    // Never fabricated; withheld unless the data is fresh and valid.
    allowedOperations: healthy ? state.allowedOperations : null,
    run: run
      ? {
          runId: run.runId,
          missionId: run.missionId,
          step: run.step,
          progress: run.progress,
          outcome: run.outcome,
          elapsedMs,
          elapsedFinal: elapsedMs !== null && run.endedAtMs !== null,
        }
      : null,
    currentEventId:
      state.epoch !== null && state.appliedSeq !== null ? `${state.epoch}:${state.appliedSeq}` : null,
    counters: state.counters,
  };
}

const monotonicNow = () =>
  typeof performance !== "undefined" && performance.now ? performance.now() : Date.now();

/**
 * transport: { fetchSnapshot(): Promise<{status, body}>,
 *              fetchEvents({epoch, afterSeq}): Promise<{status, body}> }
 * A thrown/rejected call means the transport is unreachable.
 */
export function createRunStateAdapter({
  transport,
  now = monotonicNow,
  timers = { setTimeout: (fn, ms) => setTimeout(fn, ms), clearTimeout: (id) => clearTimeout(id) },
  ...options
}) {
  const opts = { ...DEFAULT_OPTIONS, ...options };
  let state = initialRunState();
  const listeners = new Set();
  let generation = 0;
  let timerId = null;
  let inFlightGen = null;

  const getState = () => deriveView(state, now(), opts);
  const emit = () => {
    const view = getState();
    listeners.forEach((fn) => fn(view));
  };

  const schedule = (gen) => {
    if (gen !== generation || listeners.size === 0) return;
    timerId = timers.setTimeout(() => {
      timerId = null;
      tick(gen);
    }, opts.pollMs);
  };

  async function tick(gen) {
    if (gen !== generation || inFlightGen === gen) return;
    inFlightGen = gen;
    const wantSnapshot = state.needsSnapshot || state.epoch === null;
    let response;
    let failure = null;
    try {
      response = wantSnapshot
        ? await transport.fetchSnapshot()
        : await transport.fetchEvents({ epoch: state.epoch, afterSeq: state.appliedSeq });
    } catch (err) {
      failure = err;
    }
    if (inFlightGen === gen) inFlightGen = null;
    if (gen !== generation) {
      // Response belongs to an obsolete connection: never applied.
      state = bump(state, "obsoleteResponsesDropped");
      return;
    }
    const receivedAt = now();
    if (failure) {
      state = {
        ...state,
        connection: CONNECTION.DISCONNECTED,
        lastError: String(failure?.message || failure),
      };
    } else if (response.status === 404) {
      state = { ...state, connection: CONNECTION.DISABLED, lastError: "fixture backend disabled" };
    } else if (response.status === 409) {
      state = { ...state, needsSnapshot: true };
    } else if (response.status < 200 || response.status >= 300) {
      state = {
        ...state,
        connection: CONNECTION.DISCONNECTED,
        lastError: `HTTP ${response.status}`,
      };
    } else if (wantSnapshot) {
      state = applySnapshot(state, response.body, receivedAt);
    } else {
      state = applyEvents(state, response.body, receivedAt);
    }
    emit();
    // After a snapshot, fetch newer events straight away.
    if (wantSnapshot && state.connection === CONNECTION.CONNECTED && !state.needsSnapshot) {
      tick(gen);
    } else {
      schedule(gen);
    }
  }

  const start = () => {
    generation += 1;
    state = { ...state, connection: CONNECTION.CONNECTING, needsSnapshot: true };
    tick(generation);
  };

  const stop = () => {
    generation += 1;
    if (timerId !== null) timers.clearTimeout(timerId);
    timerId = null;
    state = { ...state, connection: CONNECTION.IDLE };
  };

  return {
    getState,
    subscribe(fn) {
      listeners.add(fn);
      if (listeners.size === 1) start();
      fn(getState());
      let active = true;
      return () => {
        if (!active) return;
        active = false;
        listeners.delete(fn);
        if (listeners.size === 0) stop();
      };
    },
    /** Drop the current connection and resync from a fresh snapshot. */
    reconnect() {
      if (listeners.size === 0) return;
      stop();
      start();
    },
    listenerCount: () => listeners.size,
  };
}
