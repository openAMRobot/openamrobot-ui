import {
  CONNECTION,
  FRESHNESS,
  applyEvents,
  applySnapshot,
  createRunStateAdapter,
  deriveView,
  initialRunState,
} from "./runStateAdapter";
import {
  EPOCH_A,
  EPOCH_B,
  STEP_FAILED,
  createFakeProducer,
  createFakeTimers,
  created,
  ended,
  event,
  events,
  flush,
  snapshot,
  started,
} from "./testFixtures";

const OPTS = { pollMs: 1000, staleAfterMs: 3000 };
const view = (state, now = 0) => deriveView(state, now, OPTS);

// Build a reducer state that has applied a snapshot at seq 0 and then events.
const withEvents = (list, snapOpts = {}) => {
  let s = applySnapshot(initialRunState(), snapshot(snapOpts), 0);
  s = applyEvents(s, events(list), 0);
  return s;
};

function makeAdapter(producer, extra = {}) {
  let now = 0;
  const timers = createFakeTimers();
  const adapter = createRunStateAdapter({
    transport: producer.transport,
    now: () => now,
    timers,
    ...OPTS,
    ...extra,
  });
  return {
    adapter,
    timers,
    advance: (ms) => {
      now += ms;
    },
    poll: async () => {
      timers.runAll();
      await flush();
    },
  };
}

describe("state vocabulary and separate outcome", () => {
  test("ready -> executing -> separate SUCCEEDED outcome", () => {
    let s = withEvents([created(1, "run-1")]);
    expect(view(s).operationalState).toBe("READY");
    expect(view(s).run.outcome).toBeNull();

    s = applyEvents(s, events([started(2, "run-1", 1500)]), 0);
    expect(view(s).operationalState).toBe("EXECUTING");
    expect(view(s).run.step.step_id).toBe("step-1");

    s = applyEvents(s, events([ended(3, "run-1", 4500, "SUCCEEDED", "READY")], { t: 4500 }), 0);
    const v = view(s);
    expect(v.operationalState).toBe("READY");
    expect(v.run.outcome).toEqual({ result: "SUCCEEDED", reason: null });
    expect(v.run.elapsedMs).toBe(3000);
    expect(v.run.elapsedFinal).toBe(true);
  });

  test("executing -> separate FAILED outcome with typed reason", () => {
    let s = withEvents([created(1, "run-2"), started(2, "run-2", 100)]);
    s = applyEvents(
      s,
      events([ended(3, "run-2", 900, "FAILED", "NEEDS_ASSISTANCE", STEP_FAILED)]),
      0,
    );
    const v = view(s);
    expect(v.run.outcome).toEqual({ result: "FAILED", reason: STEP_FAILED });
    expect(v.operationalState).toBe("NEEDS_ASSISTANCE");
    expect(["SUCCEEDED", "FAILED"]).not.toContain(v.operationalState);
  });

  test("an operational state outside the vocabulary shows UNKNOWN, not a guess", () => {
    const s = applySnapshot(initialRunState(), snapshot({ state: "FAILED" }), 0);
    expect(view(s).operationalState).toBe("UNKNOWN");
    expect(view(s).rawOperationalState).toBe("FAILED");
  });
});

describe("freshness and connection", () => {
  test("fresh -> stale -> disconnected; allowed operations never fabricated", async () => {
    const producer = createFakeProducer();
    producer.push(created(1, "run-1"));
    const { adapter, advance, poll } = makeAdapter(producer);
    const seen = [];
    const unsubscribe = adapter.subscribe((v) => seen.push(v));
    await flush();

    let v = adapter.getState();
    expect(v.connection).toBe(CONNECTION.CONNECTED);
    expect(v.freshness).toBe(FRESHNESS.FRESH);
    expect(v.allowedOperations).toEqual(["FIXTURE_PROVISIONAL.START"]);

    // Producer keeps answering but its heartbeat stops advancing.
    for (let i = 0; i < 4; i += 1) {
      advance(1000);
      await poll(); // eslint-disable-line no-await-in-loop
    }
    v = adapter.getState();
    expect(v.connection).toBe(CONNECTION.CONNECTED);
    expect(v.freshness).toBe(FRESHNESS.STALE);
    expect(v.healthy).toBe(false);
    expect(v.allowedOperations).toBeNull();
    expect(v.operationalState).toBe("READY"); // last reported, flagged stale by the view

    producer.mode = "down";
    advance(1000);
    await poll();
    v = adapter.getState();
    expect(v.connection).toBe(CONNECTION.DISCONNECTED);
    expect(v.freshness).toBe(FRESHNESS.STALE);
    expect(v.allowedOperations).toBeNull();
    unsubscribe();
  });

  test("advancing heartbeat keeps data fresh", async () => {
    const producer = createFakeProducer();
    const { adapter, advance, poll } = makeAdapter(producer);
    const unsubscribe = adapter.subscribe(() => {});
    await flush();
    for (let i = 0; i < 5; i += 1) {
      producer.hb += 1;
      advance(1000);
      await poll(); // eslint-disable-line no-await-in-loop
    }
    expect(adapter.getState().freshness).toBe(FRESHNESS.FRESH);
    unsubscribe();
  });

  test("a stopped connection cannot keep looking fresh", async () => {
    const producer = createFakeProducer();
    const { adapter, advance, timers } = makeAdapter(producer);
    const unsubscribe = adapter.subscribe(() => {});
    await flush();
    expect(adapter.getState().freshness).toBe(FRESHNESS.FRESH);
    unsubscribe();
    expect(timers.pendingCount()).toBe(0);
    advance(10);
    expect(adapter.getState().connection).toBe(CONNECTION.IDLE);
    expect(adapter.getState().freshness).toBe(FRESHNESS.STALE);
    expect(adapter.getState().healthy).toBe(false);
  });

  test("a hung request is not fresh either (read-time derivation)", async () => {
    const producer = createFakeProducer();
    const { adapter, advance, timers } = makeAdapter(producer);
    const unsubscribe = adapter.subscribe(() => {});
    await flush();
    producer.transport.fetchEvents = () => new Promise(() => {});
    timers.runAll();
    await flush();
    advance(OPTS.staleAfterMs + 1);
    expect(adapter.getState().freshness).toBe(FRESHNESS.STALE);
    unsubscribe();
  });

  test("404 from the fixture backend reports DISABLED, not disconnected", async () => {
    const producer = createFakeProducer();
    producer.mode = "disabled";
    const { adapter } = makeAdapter(producer);
    const unsubscribe = adapter.subscribe(() => {});
    await flush();
    const v = adapter.getState();
    expect(v.connection).toBe(CONNECTION.DISABLED);
    expect(v.healthy).toBe(false);
    expect(v.freshness).toBe(FRESHNESS.UNKNOWN);
    unsubscribe();
  });
});

describe("refresh / reconnect", () => {
  test("resubscribe gets snapshot plus newer events for the same run, only reads", async () => {
    const producer = createFakeProducer();
    producer.push(created(1, "run-1"));
    producer.push(started(2, "run-1", 100));
    const { adapter, poll } = makeAdapter(producer);
    let unsubscribe = adapter.subscribe(() => {});
    await flush();
    expect(adapter.getState().currentEventId).toBe(`${EPOCH_A}:2`);
    unsubscribe();

    producer.push(event(3, "STEP_CHANGED", "run-1", {
      run: { step: { step_id: "step-2", index: 1, count: 3 } },
      operational_state: "EXECUTING",
      reason: null,
      allowed_operations: ["FIXTURE_PROVISIONAL.CANCEL"],
    }));
    const before = { ...producer.calls };
    unsubscribe = adapter.subscribe(() => {});
    await flush();
    await poll();
    const v = adapter.getState();
    expect(producer.calls.snapshot).toBe(before.snapshot + 1);
    expect(producer.calls.events).toBeGreaterThan(before.events);
    expect(v.run.runId).toBe("run-1");
    expect(v.run.step.step_id).toBe("step-2");
    expect(v.currentEventId).toBe(`${EPOCH_A}:3`);
    expect(v.counters.runChanges).toBe(0);
    // The transport surface is read-only: no start/stop/command method exists.
    expect(Object.keys(producer.transport).sort()).toEqual(["fetchEvents", "fetchSnapshot"]);
    unsubscribe();
  });

  test("responses from an obsolete connection are dropped after reconnect", async () => {
    const producer = createFakeProducer();
    const resolvers = [];
    producer.transport.fetchSnapshot = () =>
      new Promise((resolve) => resolvers.push(resolve));
    const { adapter } = makeAdapter(producer);
    const unsubscribe = adapter.subscribe(() => {});
    adapter.reconnect();
    expect(resolvers).toHaveLength(2);
    // New connection sees epoch B; the old in-flight request answers late with epoch A.
    resolvers[1]({ status: 200, body: snapshot({ epoch: EPOCH_B, asOf: 0 }) });
    await flush();
    resolvers[0]({ status: 200, body: snapshot({ epoch: EPOCH_A, asOf: 9, state: "EXECUTING" }) });
    await flush();
    const v = adapter.getState();
    expect(v.producer.epoch).toBe(EPOCH_B);
    expect(v.operationalState).toBe("UNKNOWN");
    expect(v.counters.obsoleteResponsesDropped).toBe(1);
    unsubscribe();
  });
});

describe("ordering, dedup, races and terminal non-regression", () => {
  test("duplicate and out-of-order events are applied once, in seq order", () => {
    const s = withEvents([
      started(2, "run-1", 100),
      created(1, "run-1"),
      created(1, "run-1"),
      started(2, "run-1", 100),
    ]);
    expect(s.appliedSeq).toBe(2);
    expect(view(s).operationalState).toBe("EXECUTING");
    expect(s.counters.duplicatesDropped).toBe(2);
    const again = applyEvents(s, events([created(1, "run-1"), started(2, "run-1", 100)]), 0);
    expect(again.appliedSeq).toBe(2);
    expect(again.counters.duplicatesDropped).toBe(4);
  });

  test("a gap stops application and requests a snapshot", () => {
    const s = withEvents([created(1, "run-1"), started(3, "run-1", 100)]);
    expect(s.appliedSeq).toBe(1);
    expect(s.needsSnapshot).toBe(true);
    expect(view(s).operationalState).toBe("READY");
  });

  test("snapshot/event race: an older snapshot never rolls state back", () => {
    let s = withEvents([created(1, "run-1"), started(2, "run-1", 100)]);
    s = applyEvents(s, events([ended(3, "run-1", 900, "SUCCEEDED", "READY")]), 0);
    const late = snapshot({
      asOf: 2,
      state: "EXECUTING",
      run: { run_id: "run-1", mission_id: "m", outcome: null, started_at_ms: 100 },
    });
    s = applySnapshot(s, late, 5);
    expect(s.counters.staleSnapshotsIgnored).toBe(1);
    expect(view(s).operationalState).toBe("READY");
    expect(view(s).run.outcome.result).toBe("SUCCEEDED");
  });

  test("late events cannot resurrect a terminal run", () => {
    let s = withEvents([created(1, "run-1"), started(2, "run-1", 100)]);
    s = applyEvents(s, events([ended(3, "run-1", 900, "FAILED", "NEEDS_ASSISTANCE", STEP_FAILED)]), 0);
    s = applyEvents(s, events([
      event(4, "STEP_CHANGED", "run-1", {
        run: { outcome: null, step: { step_id: "step-3", index: 2, count: 3 } },
        operational_state: "EXECUTING",
        reason: null,
        allowed_operations: [],
      }),
    ]), 0);
    const v = view(s);
    expect(v.run.outcome).toEqual({ result: "FAILED", reason: STEP_FAILED });
    expect(v.operationalState).toBe("NEEDS_ASSISTANCE");
    expect(s.counters.eventsRejected).toBe(1);
    expect(s.counters.terminalConflicts).toBe(1);
  });

  test("a newer snapshot contradicting a terminal outcome keeps the latched outcome", () => {
    let s = withEvents([created(1, "run-1"), ended(2, "run-1", 900, "SUCCEEDED", "READY")]);
    s = applySnapshot(
      s,
      snapshot({ asOf: 5, state: "EXECUTING", run: { run_id: "run-1", outcome: { result: "FAILED" } } }),
      1,
    );
    expect(view(s).run.outcome.result).toBe("SUCCEEDED");
    expect(s.counters.terminalConflicts).toBe(1);
  });
});

describe("run and producer identity", () => {
  test("new run is distinguished from the finished one", () => {
    let s = withEvents([
      created(1, "run-4", 1),
      started(2, "run-4", 100, 1),
      ended(3, "run-4", 400, "SUCCEEDED", "READY"),
      created(4, "run-5", 2),
    ]);
    const v = view(s);
    expect(v.run.runId).toBe("run-5");
    expect(v.run.outcome).toBeNull();
    expect(s.counters.runChanges).toBe(1);
    // Events for the previous (terminal) run are rejected.
    s = applyEvents(s, events([event(5, "STEP_CHANGED", "run-4", { operational_state: "EXECUTING" })]), 0);
    expect(view(s).run.runId).toBe("run-5");
    expect(view(s).operationalState).toBe("READY");
  });

  test("producer restart: new epoch (not lexically ordered), cursor reset, old events rejected", () => {
    let s = withEvents([created(1, "run-3"), started(2, "run-3", 100)], { t: 100 });
    expect(EPOCH_B < EPOCH_A).toBe(true);
    s = applySnapshot(
      s,
      snapshot({ epoch: EPOCH_B, asOf: 0, t: 50, run: { run_id: "run-3", outcome: null } }),
      10,
    );
    let v = view(s, 10);
    expect(v.producer.epoch).toBe(EPOCH_B);
    expect(s.counters.producerRestarts).toBe(1);
    expect(s.appliedSeq).toBe(0);
    expect(v.run.runId).toBe("run-3");
    expect(v.operationalState).toBe("UNKNOWN");
    expect(v.run.elapsedMs).toBeNull(); // old-epoch start time is not comparable
    expect(v.allowedOperations).toBeNull();

    // Batch from the obsolete epoch: rejected, resync requested.
    s = applyEvents(s, events([started(3, "run-3", 100)], { epoch: EPOCH_A }), 11);
    expect(s.counters.eventsRejected).toBe(1);
    expect(s.needsSnapshot).toBe(true);
    // Stray old-epoch event inside a current-epoch batch: rejected individually.
    s = applyEvents(s, events([started(1, "run-3", 100, 3, EPOCH_A)], { epoch: EPOCH_B }), 12);
    expect(s.counters.eventsRejected).toBe(2);
    v = view(s, 12);
    expect(v.operationalState).toBe("UNKNOWN");
  });

  test("controller resyncs after restart via 409 and follows the new epoch", async () => {
    const producer = createFakeProducer();
    producer.push(created(1, "run-3"));
    const { adapter, poll } = makeAdapter(producer);
    const unsubscribe = adapter.subscribe(() => {});
    await flush();
    producer.restart(EPOCH_B);
    await poll(); // events with old epoch -> 409 -> needsSnapshot
    await poll(); // snapshot of the new epoch
    const v = adapter.getState();
    expect(v.producer.epoch).toBe(EPOCH_B);
    expect(v.counters.producerRestarts).toBe(1);
    expect(v.run.runId).toBe("run-3");
    unsubscribe();
  });
});

describe("malformed or unknown data never looks healthy", () => {
  const cases = [
    ["unknown schema version", { ...snapshot(), schema_version: "openamr.run_state/9.0" }],
    ["missing producer", { ...snapshot(), producer: undefined }],
    ["missing epoch", { ...snapshot(), producer: { ...snapshot().producer, epoch: "" } }],
    ["unknown timebase", { ...snapshot(), producer: { ...snapshot().producer, timebase: "wall" } }],
    ["missing run key", (() => { const b = snapshot(); delete b.run; return b; })()],
    ["run without id", snapshot({ run: { mission_id: "m" } })],
    ["not an object", "<html>index</html>"],
    ["null body", null],
  ];
  test.each(cases)("%s", (_, body) => {
    const s = applySnapshot(initialRunState(), body, 0);
    const v = view(s);
    expect(v.validity).toBe("INVALID");
    expect(v.connection).toBe(CONNECTION.INVALID);
    expect(v.healthy).toBe(false);
    expect(v.freshness).not.toBe(FRESHNESS.FRESH);
    expect(v.allowedOperations).toBeNull();
  });

  test("malformed event batch marks invalid and applies nothing", () => {
    const s0 = withEvents([created(1, "run-1")]);
    const s = applyEvents(s0, events([{ seq: "2", epoch: EPOCH_A }]), 0);
    expect(view(s).healthy).toBe(false);
    expect(s.appliedSeq).toBe(1);
  });

  test("valid envelope with missing optional data stays unknown", () => {
    const s = applySnapshot(
      initialRunState(),
      { ...snapshot(), robot: undefined, profile: undefined, operational_state: undefined, allowed_operations: undefined },
      0,
    );
    const v = view(s);
    expect(v.robot).toBeNull();
    expect(v.profile).toBeNull();
    expect(v.operationalState).toBe("UNKNOWN");
    expect(v.allowedOperations).toBeNull();
    expect(v.run).toBeNull();
  });

  test("controller with invalid payload reports INVALID, and 500 reports DISCONNECTED", async () => {
    const producer = createFakeProducer();
    producer.transport.fetchSnapshot = async () => ({ status: 200, body: { hello: 1 } });
    const { adapter } = makeAdapter(producer);
    const unsubscribe = adapter.subscribe(() => {});
    await flush();
    expect(adapter.getState().connection).toBe(CONNECTION.INVALID);
    expect(adapter.getState().healthy).toBe(false);
    unsubscribe();

    const p2 = createFakeProducer();
    p2.transport.fetchSnapshot = async () => ({ status: 500, body: null });
    const a2 = makeAdapter(p2).adapter;
    const u2 = a2.subscribe(() => {});
    await flush();
    expect(a2.getState().connection).toBe(CONNECTION.DISCONNECTED);
    u2();
  });
});

describe("shared adapter: multiple subscribers", () => {
  test("two subscribers see identical state with one transport loop; unsubscribe cleans up", async () => {
    const producer = createFakeProducer();
    producer.push(created(1, "run-1"));
    const { adapter, timers, poll } = makeAdapter(producer);
    const a = [];
    const b = [];
    const unsubA = adapter.subscribe((v) => a.push(v));
    const unsubB = adapter.subscribe((v) => b.push(v));
    expect(adapter.listenerCount()).toBe(2);
    await flush();
    const callsAfterStart = { ...producer.calls };
    expect(callsAfterStart.snapshot).toBe(1); // not one per subscriber

    producer.push(started(2, "run-1", 100));
    await poll();
    expect(producer.calls.events).toBe(callsAfterStart.events + 1);
    expect(timers.pendingCount()).toBe(1);
    expect(a[a.length - 1]).toEqual(b[b.length - 1]);
    expect(a[a.length - 1].operationalState).toBe("EXECUTING");

    unsubA();
    unsubA(); // idempotent
    expect(adapter.listenerCount()).toBe(1);
    expect(timers.pendingCount()).toBe(1);
    unsubB();
    expect(adapter.listenerCount()).toBe(0);
    expect(timers.pendingCount()).toBe(0);

    const calls = { ...producer.calls };
    timers.runAll();
    await flush();
    expect(producer.calls).toEqual(calls);
    const lenA = a.length;
    producer.push(ended(3, "run-1", 400, "SUCCEEDED", "READY"));
    await poll();
    expect(a).toHaveLength(lenA);
  });
});
