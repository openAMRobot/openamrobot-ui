import { AppConfig } from "../constants";
import { installMockRoslib, uninstallMockRoslib } from "../motion/testing/mockRoslib";
import {
  createOperatorStatusFeed,
  deriveOperatorStatus,
  DISPLAY_DISCONNECTED,
  DISPLAY_FAULTED,
  DISPLAY_NOT_READY,
  DISPLAY_READY,
  DISPLAY_STALE,
  DISPLAY_STOP_UNKNOWN,
  DISPLAY_STOPPED,
  DISPLAY_WAITING,
  NAVIGATION_STATUS_STALE_AFTER_MS,
  parseMissionStatus,
} from "./operatorStatus";

const NOW = 100000;
const released = { connection: "connected", lock: "released", localStop: false };
const navMsg = (over = {}) => ({
  health: 1,
  navigation_readiness: 1,
  not_ready_reasons: [],
  active_reasons: [],
  stack: { state: 3, inactive_nodes: [] },
  task: { state: 1, reason: 0, distance_remaining: 0 },
  ...over,
});
const fresh = (msg) => ({ msg, atMs: NOW - 100 });
const odom = fresh({});

describe("deriveOperatorStatus display states", () => {
  it("is DISCONNECTED without rosbridge, whatever the samples say", () => {
    const s = deriveOperatorStatus(
      { gate: { ...released, connection: "disconnected" }, navigation: fresh(navMsg()), odom },
      NOW,
    );
    expect(s.display).toBe(DISPLAY_DISCONNECTED);
  });

  it("is STOPPED when the lock is engaged or Stop was pressed here", () => {
    expect(
      deriveOperatorStatus({ gate: { ...released, lock: "locked" }, navigation: fresh(navMsg()), odom }, NOW)
        .display,
    ).toBe(DISPLAY_STOPPED);
    expect(
      deriveOperatorStatus({ gate: { ...released, localStop: true }, navigation: fresh(navMsg()), odom }, NOW)
        .display,
    ).toBe(DISPLAY_STOPPED);
  });

  it("shows the gate's refusal reason from /safety/stop_state", () => {
    const s = deriveOperatorStatus(
      {
        gate: { ...released, lock: "locked" },
        stopState: fresh({ state: 1, reason: "navigate_to_pose goal still executing" }),
        navigation: fresh(navMsg()),
        odom,
      },
      NOW,
    );
    expect(s.display).toBe(DISPLAY_STOPPED);
    expect(s.stop.text).toBe("RELEASE_REFUSED");
    expect(s.stop.reason).toBe("navigate_to_pose goal still executing");
    expect(s.detail).toMatch(/navigate_to_pose goal still executing/);
  });

  it("says Stop and Resume need #90 when no stop state ever arrived", () => {
    const s = deriveOperatorStatus({ gate: { ...released, lock: "unknown" }, odom }, NOW);
    expect(s.display).toBe(DISPLAY_STOP_UNKNOWN);
    expect(s.stop.text).toBe("no sample");
    expect(s.stop.reason).toMatch(/#90/);
  });

  it("is WAITING before the first navigation status, STALE after it ages out", () => {
    expect(deriveOperatorStatus({ gate: released, odom }, NOW).display).toBe(DISPLAY_WAITING);
    const old = { msg: navMsg(), atMs: NOW - NAVIGATION_STATUS_STALE_AFTER_MS - 1 };
    const s = deriveOperatorStatus({ gate: released, navigation: old, odom }, NOW);
    expect(s.display).toBe(DISPLAY_STALE);
    // A stale sample never reports its old readiness as current.
    expect(s.readiness.text).toBe("unknown");
  });

  it("is STALE when the robot pose stops arriving", () => {
    const s = deriveOperatorStatus(
      { gate: released, navigation: fresh(navMsg()), odom: { msg: {}, atMs: NOW - 10000 } },
      NOW,
    );
    expect(s.display).toBe(DISPLAY_STALE);
    expect(s.sources.pose).toMatch(/stale/);
  });

  it("is FAULTED on health FAULT and names the reasons", () => {
    const s = deriveOperatorStatus(
      { gate: released, navigation: fresh(navMsg({ health: 3, active_reasons: [4007] })), odom },
      NOW,
    );
    expect(s.display).toBe(DISPLAY_FAULTED);
    expect(s.detail).toMatch(/STUCK/);
  });

  it("is NOT READY with named reasons, READY otherwise", () => {
    const s = deriveOperatorStatus(
      {
        gate: released,
        navigation: fresh(navMsg({ navigation_readiness: 0, not_ready_reasons: [9001, 3001] })),
        odom,
      },
      NOW,
    );
    expect(s.display).toBe(DISPLAY_NOT_READY);
    expect(s.readiness.reasons).toEqual(["BASE_LINK_LOST", "LOCALIZATION_NOT_INITIALIZED"]);
    expect(deriveOperatorStatus({ gate: released, navigation: fresh(navMsg()), odom }, NOW).display).toBe(
      DISPLAY_READY,
    );
  });
});

describe("current task", () => {
  it("prefers the Proposed #82 mission status and labels it Proposed", () => {
    const data = JSON.stringify({
      phase: "navigate",
      state: "running",
      sim: 12.5,
      phases: [
        { phase: "navigate", verdict: "RUNNING" },
        { phase: "lift", verdict: "NOT RUN" },
      ],
    });
    const s = deriveOperatorStatus(
      { gate: released, navigation: fresh(navMsg()), mission: fresh({ data }), odom },
      NOW,
    );
    expect(s.task.text).toBe("Baseline 1: navigate (running)");
    expect(s.task.source).toMatch(/Proposed/);
    expect(s.task.phases).toHaveLength(2);
  });

  it("falls back to the navigation task with distance remaining", () => {
    const s = deriveOperatorStatus(
      { gate: released, navigation: fresh(navMsg({ task: { state: 2, reason: 0, distance_remaining: 1.234 } })), odom },
      NOW,
    );
    expect(s.task.text).toBe("Navigation ACTIVE, 1.23 m remaining");
  });

  it("ignores a mission payload that is not the expected JSON", () => {
    expect(parseMissionStatus("not json")).toBeNull();
    expect(parseMissionStatus(undefined)).toBeNull();
  });
});

describe("createOperatorStatusFeed", () => {
  let mock;
  beforeEach(() => {
    mock = installMockRoslib();
  });
  afterEach(() => uninstallMockRoslib());

  it("only subscribes: no publish, no advertise, no service", () => {
    const samples = [];
    const feed = createOperatorStatusFeed({
      ros: {},
      now: () => 5,
      onSample: (key, sample) => samples.push([key, sample]),
    });
    const names = mock.registry.topics.map((t) => t.name).sort();
    expect(names).toEqual(
      [
        AppConfig.NAVIGATION_STATUS_TOPIC,
        AppConfig.STOP_STATE_TOPIC,
        AppConfig.MISSION_STATUS_TOPIC,
        AppConfig.ROBOT_POSE_TOPIC,
      ].sort(),
    );
    mock.topicsNamed(AppConfig.STOP_STATE_TOPIC)[0].emit({ state: 2, reason: "" });
    expect(samples).toEqual([["stopState", { msg: { state: 2, reason: "" }, atMs: 5 }]]);
    mock.registry.topics.forEach((t) => {
      expect(t.published).toEqual([]);
      expect(t.advertised).toBe(false);
    });
    expect(mock.registry.services).toEqual([]);
    feed.stop();
    mock.registry.topics.forEach((t) => expect(t.callbacks).toEqual([]));
  });
});
