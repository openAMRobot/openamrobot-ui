import { AppConfig } from "../constants";
import {
  NOT_SYNCED_LABEL,
  bestSample,
  createRobotClock,
  evaluateClock,
  makeSample,
  monotonicNowMs,
  pushSample,
  realStampSeconds,
  stampFor,
  stampMarginS,
  toSeconds,
} from "./robotClock";
import { installMockRoslib, uninstallMockRoslib } from "./testing/mockRoslib";

// Robot clock = local clock (ms / 1000) + TRUE_OFFSET_S in these tests.
const TRUE_OFFSET_S = 1000.0;
const robotAt = (localMs) => {
  const seconds = localMs / 1000 + TRUE_OFFSET_S;
  const sec = Math.floor(seconds);
  return { sec, nanosec: Math.round((seconds - sec) * 1e9) };
};

const realState = (samples, lastGoodSyncMs) => ({
  mode: "real",
  samples,
  lastGoodSyncMs,
  lastClock: null,
  lastClockReceivedMs: null,
});

const simState = (lastClock, lastClockReceivedMs) => ({
  mode: "sim",
  samples: [],
  lastGoodSyncMs: null,
  lastClock,
  lastClockReceivedMs,
});

describe("real robot estimator (/rosapi/get_time)", () => {
  it("estimates the offset from the round-trip midpoint", () => {
    const sample = makeSample({ sentAtMs: 1000, receivedAtMs: 1100, robotTime: robotAt(1050) });
    expect(sample.rttS).toBeCloseTo(0.1, 9);
    expect(sample.offsetS).toBeCloseTo(TRUE_OFFSET_S, 6);
  });

  it("margin is RTT/2 plus 5 ms", () => {
    expect(stampMarginS({ rttS: 0.04 })).toBeCloseTo(0.025, 9);
  });

  it.each([
    ["all delay on the request leg", 1100],
    ["all delay on the response leg", 1000],
    ["symmetric delay", 1050],
  ])("never stamps ahead of robot time with %s", (_label, robotReadAtMs) => {
    const sample = makeSample({
      sentAtMs: 1000,
      receivedAtMs: 1100,
      robotTime: robotAt(robotReadAtMs),
    });
    const nowMs = 1500;
    const trueRobotNowS = toSeconds(robotAt(nowMs));
    const stampS = realStampSeconds(sample, nowMs);
    // At least 5 ms behind robot time, at most RTT + 5 ms behind.
    expect(stampS).toBeLessThanOrEqual(trueRobotNowS - 0.005 + 1e-9);
    expect(trueRobotNowS - stampS).toBeLessThanOrEqual(0.1 + 0.005 + 1e-9);
  });

  it("keeps the lowest-RTT sample of the last 5", () => {
    let samples = [];
    const rtts = [10, 80, 60, 70, 90, 50];
    rtts.forEach((rtt, i) => {
      samples = pushSample(
        samples,
        makeSample({ sentAtMs: i * 2000, receivedAtMs: i * 2000 + rtt, robotTime: robotAt(i * 2000) }),
      );
    });
    expect(samples).toHaveLength(5);
    // The 10 ms sample was the oldest of six and has left the window.
    expect(bestSample(samples).rttS).toBeCloseTo(0.05, 9);
  });

  it("is synced while RTT/2 plus the margin stays within 0.15 s", () => {
    const sample = makeSample({ sentAtMs: 0, receivedAtMs: 144, robotTime: robotAt(72) });
    const status = evaluateClock(realState([sample], 144), 200);
    expect(status.synced).toBe(true);
  });

  it("disables when RTT/2 plus the margin exceeds 0.15 s", () => {
    const sample = makeSample({ sentAtMs: 0, receivedAtMs: 146, robotTime: robotAt(73) });
    const status = evaluateClock(realState([sample], 146), 200);
    expect(status.synced).toBe(false);
    expect(status.label).toBe(NOT_SYNCED_LABEL);
    expect(stampFor(realState([sample], 146), 200)).toBeNull();
  });

  it("disables when the last good sync is older than 6 s", () => {
    const sample = makeSample({ sentAtMs: 0, receivedAtMs: 20, robotTime: robotAt(10) });
    expect(evaluateClock(realState([sample], 20), 6020).synced).toBe(true);
    expect(evaluateClock(realState([sample], 20), 6021).synced).toBe(false);
    expect(stampFor(realState([sample], 20), 6021)).toBeNull();
  });

  it("disables before the first sample", () => {
    expect(evaluateClock(realState([], null), 0).synced).toBe(false);
  });
});

describe("simulation stamping (/clock)", () => {
  it("stamps with the latest /clock value, never extrapolated", () => {
    const state = simState({ sec: 42, nanosec: 500000000 }, 1000);
    expect(stampFor(state, 1000)).toEqual({ sec: 42, nanosec: 500000000 });
    // 140 ms later with no new /clock: still exactly the last value.
    expect(stampFor(state, 1140)).toEqual({ sec: 42, nanosec: 500000000 });
  });

  it("never stamps ahead of a sim clock running slower than real time", () => {
    // Sim at half real-time speed: /clock advances 10 ms per 20 ms of wall time.
    let state = simState(null, null);
    for (let localMs = 0; localMs <= 200; localMs += 20) {
      const simS = 10 + (localMs / 1000) * 0.5;
      const sec = Math.floor(simS);
      state = simState({ sec, nanosec: Math.round((simS - sec) * 1e9) }, localMs);
      const queryMs = localMs + 15;
      const trueSimNowS = 10 + (queryMs / 1000) * 0.5;
      expect(toSeconds(stampFor(state, queryMs))).toBeLessThanOrEqual(trueSimNowS + 1e-9);
    }
  });

  it("stamps the paused value while a paused sim keeps publishing /clock", () => {
    const paused = { sec: 7, nanosec: 250000000 };
    let state = simState(paused, 0);
    for (let localMs = 0; localMs <= 1000; localMs += 20) state = simState(paused, localMs);
    expect(stampFor(state, 1010)).toEqual(paused);
  });

  it("disables when /clock goes silent for more than 0.15 s", () => {
    const state = simState({ sec: 7, nanosec: 0 }, 1000);
    expect(evaluateClock(state, 1150).synced).toBe(true);
    expect(evaluateClock(state, 1151).synced).toBe(false);
    expect(stampFor(state, 1151)).toBeNull();
  });

  it("disables while the sim clock is at zero", () => {
    const status = evaluateClock(simState({ sec: 0, nanosec: 0 }, 1000), 1000);
    expect(status.synced).toBe(false);
    expect(status.detail).toMatch(/zero/);
  });

  it("disables before the first /clock message", () => {
    expect(evaluateClock(simState(null, null), 0).synced).toBe(false);
  });
});

describe("local time source", () => {
  it("falls back to Date.now() when performance.timeOrigin is not a number", () => {
    jest.useFakeTimers();
    try {
      expect(Number.isFinite(monotonicNowMs())).toBe(true);
    } finally {
      jest.useRealTimers();
    }
  });

  it("is not synced when the local time is not usable", () => {
    expect(evaluateClock(simState({ sec: 7, nanosec: 0 }, 0), NaN).synced).toBe(false);
  });
});

describe("unknown mode", () => {
  it("is not synced and has no stamp", () => {
    const state = { mode: "unknown", samples: [], lastGoodSyncMs: null, lastClock: null, lastClockReceivedMs: null };
    const status = evaluateClock(state, 0);
    expect(status.synced).toBe(false);
    expect(status.label).toBe("Robot clock not synced");
    expect(stampFor(state, 0)).toBeNull();
  });
});

describe("createRobotClock controller", () => {
  let mock;
  let nowMs;
  const now = () => nowMs;

  beforeEach(() => {
    jest.useFakeTimers();
    mock = installMockRoslib();
    nowMs = 10000;
  });

  afterEach(() => {
    jest.useRealTimers();
    uninstallMockRoslib();
  });

  const answerMode = (value) =>
    mock.onService(AppConfig.UI_CMD_VEL_RELAY_GET_PARAMETERS_SERVICE, (req, ok) =>
      ok({ values: [value] }),
    );

  it("stays unknown, and uses neither clock source, until the relay parameter is read", () => {
    const clock = createRobotClock({ ros: {}, now });
    expect(clock.getStatus().mode).toBe("unknown");
    expect(clock.getStamp()).toBeNull();
    expect(mock.callsTo(AppConfig.UI_CMD_VEL_RELAY_GET_PARAMETERS_SERVICE)[0].names).toEqual([
      "use_sim_time",
    ]);
    expect(mock.topicsNamed(AppConfig.CLOCK_TOPIC)).toHaveLength(0);
    expect(mock.servicesNamed(AppConfig.ROSAPI_GET_TIME_SERVICE)).toHaveLength(0);
    clock.stop();
  });

  it("retries the parameter read every 2 s after a failure", () => {
    mock.onService(AppConfig.UI_CMD_VEL_RELAY_GET_PARAMETERS_SERVICE, (req, ok, fail) =>
      fail("service not available"),
    );
    const clock = createRobotClock({ ros: {}, now });
    expect(mock.callsTo(AppConfig.UI_CMD_VEL_RELAY_GET_PARAMETERS_SERVICE)).toHaveLength(1);
    jest.advanceTimersByTime(2000);
    expect(mock.callsTo(AppConfig.UI_CMD_VEL_RELAY_GET_PARAMETERS_SERVICE)).toHaveLength(2);
    expect(clock.getStatus().mode).toBe("unknown");
    clock.stop();
  });

  it("stays unknown when the parameter is not a bool", () => {
    answerMode({ type: 0 });
    const clock = createRobotClock({ ros: {}, now });
    expect(clock.getStatus().mode).toBe("unknown");
    clock.stop();
  });

  it("use_sim_time true: subscribes to /clock and stamps with its value", () => {
    answerMode({ type: 1, bool_value: true });
    const onChange = jest.fn();
    const clock = createRobotClock({ ros: {}, now, onChange });
    const [clockTopic] = mock.topicsNamed(AppConfig.CLOCK_TOPIC);
    expect(clockTopic.opts.throttle_rate).toBe(10);
    expect(clock.getStamp()).toBeNull();

    clockTopic.emit({ clock: { sec: 55, nanosec: 1000 } });
    expect(clock.getStamp()).toEqual({ sec: 55, nanosec: 1000 });
    expect(onChange).toHaveBeenLastCalledWith(expect.objectContaining({ mode: "sim", synced: true }));

    nowMs += 151;
    expect(clock.getStamp()).toBeNull();
    // Never asks /rosapi/get_time in simulation.
    expect(mock.servicesNamed(AppConfig.ROSAPI_GET_TIME_SERVICE)).toHaveLength(0);
    clock.stop();
  });

  it("use_sim_time false: resyncs from /rosapi/get_time every 2 s", () => {
    answerMode({ type: 1, bool_value: false });
    let replyDelayMs = 20;
    mock.onService(AppConfig.ROSAPI_GET_TIME_SERVICE, (req, ok) => {
      const robotTime = robotAt(nowMs + replyDelayMs / 2);
      nowMs += replyDelayMs;
      ok({ time: robotTime });
    });
    const clock = createRobotClock({ ros: {}, now });
    expect(mock.callsTo(AppConfig.ROSAPI_GET_TIME_SERVICE)).toHaveLength(1);
    expect(clock.getStatus().synced).toBe(true);

    const stamp = clock.getStamp();
    const trueRobotNowS = toSeconds(robotAt(nowMs));
    expect(toSeconds(stamp)).toBeLessThanOrEqual(trueRobotNowS - 0.005 + 1e-6);

    jest.advanceTimersByTime(2000);
    expect(mock.callsTo(AppConfig.ROSAPI_GET_TIME_SERVICE)).toHaveLength(2);

    // Robot stops answering: after 6 s without a good sync, not synced.
    mock.onService(AppConfig.ROSAPI_GET_TIME_SERVICE, () => {});
    nowMs += 6001;
    expect(clock.getStatus().synced).toBe(false);
    expect(clock.getStamp()).toBeNull();
    replyDelayMs = 0;
    clock.stop();
  });

  it("stop() ends every timer and subscription", () => {
    answerMode({ type: 1, bool_value: false });
    mock.onService(AppConfig.ROSAPI_GET_TIME_SERVICE, (req, ok) => ok({ time: robotAt(nowMs) }));
    const clock = createRobotClock({ ros: {}, now });
    clock.stop();
    jest.advanceTimersByTime(10000);
    expect(mock.callsTo(AppConfig.ROSAPI_GET_TIME_SERVICE)).toHaveLength(1);
  });
});
