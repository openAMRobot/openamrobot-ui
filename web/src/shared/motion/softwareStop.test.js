import { ACTION_CANCEL_SERVICES, AppConfig } from "../constants";
import { addEvent } from "../events/eventLog";
import { subscribeAbort } from "./abortBus";
import { resetMotionGate, setClockSource, setClockStatus, setConnection, setLockState } from "./motionGate";
import { resumeFromSoftwareStop, softwareStop } from "./softwareStop";
import { attachRos, holdVelocity, releaseAll } from "./uiCommand";
import { installMockRoslib, uninstallMockRoslib } from "./testing/mockRoslib";

jest.mock("../events/eventLog", () => ({ addEvent: jest.fn() }));

let mock;
beforeEach(() => {
  mock = installMockRoslib();
  mock.onService(AppConfig.STOP_LOCK_ENGAGE_SERVICE, (req, ok) => ok({ success: true, message: "" }));
  mock.onService(AppConfig.STOP_LOCK_RELEASE_SERVICE, (req, ok) => ok({ success: true, message: "" }));
});
afterEach(() => {
  releaseAll();
  attachRos(null);
  resetMotionGate();
  uninstallMockRoslib();
  jest.clearAllMocks();
});

describe("Stop (software)", () => {
  it("cancels every Nav2 and docking action the UI starts", () => {
    expect(ACTION_CANCEL_SERVICES).toEqual([
      "/navigate_to_pose/_action/cancel_goal",
      "/compute_path_to_pose/_action/cancel_goal",
      "/dock_robot/_action/cancel_goal",
      "/undock_robot/_action/cancel_goal",
    ]);
  });

  it("calls engage once and each cancel once, aborts runs and logs", async () => {
    const aborted = jest.fn();
    const unsubscribe = subscribeAbort(aborted);
    await softwareStop({}, "test");
    unsubscribe();

    expect(mock.callsTo(AppConfig.STOP_LOCK_ENGAGE_SERVICE)).toHaveLength(1);
    ACTION_CANCEL_SERVICES.forEach((name) => {
      const calls = mock.callsTo(name);
      expect(calls).toHaveLength(1);
      expect(calls[0].goal_info.goal_id.uuid).toEqual(new Array(16).fill(0));
      expect(calls[0].goal_info.stamp).toEqual({ sec: 0, nanosec: 0 });
    });
    expect(mock.callsTo(AppConfig.STOP_LOCK_RELEASE_SERVICE)).toHaveLength(0);
    expect(aborted).toHaveBeenCalledTimes(1);
    expect(addEvent).toHaveBeenCalledWith(
      expect.objectContaining({ type: "safety", message: expect.stringContaining("Stop (software)") }),
    );
  });

  it("is idempotent on repeated presses and works while already locked", async () => {
    setConnection("connected");
    setLockState("locked");
    await softwareStop({}, "test");
    await softwareStop({}, "test");
    await softwareStop({}, "test");
    expect(mock.callsTo(AppConfig.STOP_LOCK_ENGAGE_SERVICE)).toHaveLength(3);
    ACTION_CANCEL_SERVICES.forEach((name) => expect(mock.callsTo(name)).toHaveLength(3));
    expect(mock.callsTo(AppConfig.STOP_LOCK_RELEASE_SERVICE)).toHaveLength(0);
  });

  it("drops a held manual command", async () => {
    jest.useFakeTimers();
    attachRos({});
    setConnection("connected");
    setLockState("released");
    setClockSource({ getStamp: () => ({ sec: 1, nanosec: 0 }) });
    setClockStatus({ mode: "sim", synced: true, label: "", detail: "" });
    holdVelocity("joystick", { linear: 0.1 });
    const [cmdVel] = mock.topicsNamed(AppConfig.UI_CMD_VEL_TOPIC);
    const before = cmdVel.published.length;
    softwareStop({}, "test");
    // One zero (the gate was still open when the hold was dropped), then silence.
    expect(cmdVel.published).toHaveLength(before + 1);
    expect(cmdVel.published[before].twist.linear.x).toBe(0);
    jest.advanceTimersByTime(500);
    expect(cmdVel.published).toHaveLength(before + 1);
    jest.useRealTimers();
  });

  it("logs when the engage call fails", async () => {
    mock.onService(AppConfig.STOP_LOCK_ENGAGE_SERVICE, (req, ok, fail) => fail("unavailable"));
    const result = await softwareStop({}, "test");
    expect(result.success).toBe(false);
    expect(addEvent).toHaveBeenCalledWith(
      expect.objectContaining({ message: expect.stringContaining("did not confirm") }),
    );
  });
});

describe("Resume", () => {
  it("calls only the release service", async () => {
    await resumeFromSoftwareStop({});
    expect(mock.callsTo(AppConfig.STOP_LOCK_RELEASE_SERVICE)).toHaveLength(1);
    expect(mock.callsTo(AppConfig.STOP_LOCK_ENGAGE_SERVICE)).toHaveLength(0);
  });
});
