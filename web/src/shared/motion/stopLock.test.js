import { AppConfig } from "../constants";
import {
  LOCK_LOCKED,
  LOCK_RELEASED,
  LOCK_UNKNOWN,
  createStopLock,
  engageStopLock,
  evaluateLock,
  releaseStopLock,
} from "./stopLock";
import { installMockRoslib, uninstallMockRoslib } from "./testing/mockRoslib";

describe("evaluateLock", () => {
  it("is unknown before any sample", () => {
    expect(evaluateLock({ lastValue: null, lastSampleMs: null }, 0)).toBe(LOCK_UNKNOWN);
  });

  it("maps true to locked and false to released", () => {
    expect(evaluateLock({ lastValue: true, lastSampleMs: 0 }, 10)).toBe(LOCK_LOCKED);
    expect(evaluateLock({ lastValue: false, lastSampleMs: 0 }, 10)).toBe(LOCK_RELEASED);
  });

  it("is unknown when the local time is not usable", () => {
    expect(evaluateLock({ lastValue: false, lastSampleMs: 0 }, NaN)).toBe(LOCK_UNKNOWN);
    expect(evaluateLock({ lastValue: false, lastSampleMs: NaN }, 10)).toBe(LOCK_UNKNOWN);
  });

  it("turns a stale released sample into unknown, never released", () => {
    expect(evaluateLock({ lastValue: false, lastSampleMs: 0 }, 1000)).toBe(LOCK_RELEASED);
    expect(evaluateLock({ lastValue: false, lastSampleMs: 0 }, 1001)).toBe(LOCK_UNKNOWN);
  });
});

describe("createStopLock", () => {
  let mock;
  let nowMs;

  beforeEach(() => {
    jest.useFakeTimers();
    mock = installMockRoslib();
    nowMs = 0;
  });

  afterEach(() => {
    jest.useRealTimers();
    uninstallMockRoslib();
  });

  it("subscribes to /safety/stop_lock as std_msgs/Bool, depth 1", () => {
    const lock = createStopLock({ ros: {}, now: () => nowMs });
    const [topic] = mock.topicsNamed("/safety/stop_lock");
    expect(topic.messageType).toBe("std_msgs/Bool");
    expect(topic.opts.queue_length).toBe(1);
    lock.stop();
  });

  it("starts unknown and follows the samples", () => {
    const onChange = jest.fn();
    const lock = createStopLock({ ros: {}, now: () => nowMs, onChange });
    expect(lock.getState()).toBe(LOCK_UNKNOWN);
    const [topic] = mock.topicsNamed(AppConfig.STOP_LOCK_TOPIC);
    topic.emit({ data: true });
    expect(lock.getState()).toBe(LOCK_LOCKED);
    topic.emit({ data: false });
    expect(lock.getState()).toBe(LOCK_RELEASED);
    expect(onChange.mock.calls.map((c) => c[0])).toEqual([LOCK_LOCKED, LOCK_RELEASED]);
    lock.stop();
  });

  it("goes unknown when the 10 Hz republish stops", () => {
    const onChange = jest.fn();
    const lock = createStopLock({ ros: {}, now: () => nowMs, onChange });
    mock.topicsNamed(AppConfig.STOP_LOCK_TOPIC)[0].emit({ data: false });
    nowMs = 1050;
    jest.advanceTimersByTime(50);
    expect(onChange).toHaveBeenLastCalledWith(LOCK_UNKNOWN);
    lock.stop();
  });

  it("ignores malformed samples", () => {
    const lock = createStopLock({ ros: {}, now: () => nowMs });
    mock.topicsNamed(AppConfig.STOP_LOCK_TOPIC)[0].emit({ data: "false" });
    expect(lock.getState()).toBe(LOCK_UNKNOWN);
    lock.stop();
  });

  it("a new connection starts unknown again", () => {
    const first = createStopLock({ ros: {}, now: () => nowMs });
    mock.topicsNamed(AppConfig.STOP_LOCK_TOPIC)[0].emit({ data: false });
    first.stop();
    const second = createStopLock({ ros: {}, now: () => nowMs });
    expect(second.getState()).toBe(LOCK_UNKNOWN);
    second.stop();
  });
});

describe("engage and release services", () => {
  let mock;
  beforeEach(() => {
    mock = installMockRoslib();
  });
  afterEach(uninstallMockRoslib);

  it("call std_srvs/Trigger on the two platform services", async () => {
    mock.onService(AppConfig.STOP_LOCK_ENGAGE_SERVICE, (req, ok) => ok({ success: true, message: "engaged" }));
    mock.onService(AppConfig.STOP_LOCK_RELEASE_SERVICE, (req, ok) => ok({ success: true, message: "released" }));
    await expect(engageStopLock({})).resolves.toEqual({ success: true, message: "engaged" });
    await expect(releaseStopLock({})).resolves.toEqual({ success: true, message: "released" });
    expect(mock.servicesNamed(AppConfig.STOP_LOCK_ENGAGE_SERVICE)[0].serviceType).toBe("std_srvs/Trigger");
    expect(mock.servicesNamed(AppConfig.STOP_LOCK_RELEASE_SERVICE)[0].serviceType).toBe("std_srvs/Trigger");
  });

  it("report failure when the service errors", async () => {
    mock.onService(AppConfig.STOP_LOCK_ENGAGE_SERVICE, (req, ok, fail) => fail("no service"));
    await expect(engageStopLock({})).resolves.toEqual({ success: false, message: "no service" });
  });
});
