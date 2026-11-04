import React from "react";
import { act, render } from "@testing-library/react";

import MotionBridge from "./MotionBridge";
import { AppConfig } from "../constants";
import { subscribeAbort } from "./abortBus";
import { getMotionGate, resetMotionGate } from "./motionGate";
import { attachRos } from "./uiCommand";
import { installMockRoslib, uninstallMockRoslib } from "./testing/mockRoslib";

let mockRosStatus = "connected";
const mockRos = { id: "ros" };
jest.mock("../../app/App", () => ({
  useRos: () => mockRos,
  useRosStatus: () => mockRosStatus,
}));
jest.mock("../events/eventLog", () => ({ addEvent: jest.fn() }));

let mock;
beforeEach(() => {
  jest.useFakeTimers();
  mock = installMockRoslib();
  mockRosStatus = "connected";
  mock.onService(AppConfig.UI_CMD_VEL_RELAY_GET_PARAMETERS_SERVICE, (req, ok) =>
    ok({ values: [{ type: 1, bool_value: true }] }),
  );
});
afterEach(() => {
  attachRos(null);
  resetMotionGate();
  jest.useRealTimers();
  uninstallMockRoslib();
});

const latest = (name) => {
  const topics = mock.topicsNamed(name);
  return topics[topics.length - 1];
};

describe("MotionBridge", () => {
  it("opens the gate only with a released lock and a ticking /clock", () => {
    render(<MotionBridge />);
    expect(getMotionGate().lock).toBe("unknown");
    expect(getMotionGate().canDrive).toBe(false);
    act(() => latest(AppConfig.STOP_LOCK_TOPIC).emit({ data: false }));
    expect(getMotionGate().canDrive).toBe(false);
    act(() => latest(AppConfig.CLOCK_TOPIC).emit({ clock: { sec: 12, nanosec: 0 } }));
    expect(getMotionGate().canDrive).toBe(true);
  });

  it("reconnect starts as unknown, even if the lock was released before", () => {
    const { rerender } = render(<MotionBridge />);
    act(() => latest(AppConfig.STOP_LOCK_TOPIC).emit({ data: false }));
    expect(getMotionGate().lock).toBe("released");

    mockRosStatus = "disconnected";
    rerender(<MotionBridge />);
    expect(getMotionGate().lock).toBe("unknown");
    expect(getMotionGate().canDrive).toBe(false);

    mockRosStatus = "connected";
    rerender(<MotionBridge />);
    expect(getMotionGate().lock).toBe("unknown");
    expect(getMotionGate().clock.mode).toBe("sim");
    expect(getMotionGate().clock.synced).toBe(false);
  });

  it("a locked or unknown lock aborts programs and missions", () => {
    const aborted = jest.fn();
    const unsubscribe = subscribeAbort(aborted);
    render(<MotionBridge />);
    aborted.mockClear();
    act(() => latest(AppConfig.STOP_LOCK_TOPIC).emit({ data: false }));
    expect(aborted).not.toHaveBeenCalled();
    act(() => latest(AppConfig.STOP_LOCK_TOPIC).emit({ data: true }));
    expect(aborted).toHaveBeenCalledTimes(1);
    act(() => latest(AppConfig.STOP_LOCK_TOPIC).emit({ data: false }));
    aborted.mockClear();
    // No republish for more than 1 s: unknown, which aborts too.
    act(() => jest.advanceTimersByTime(1100));
    expect(getMotionGate().lock).toBe("unknown");
    expect(aborted).toHaveBeenCalledTimes(1);
    unsubscribe();
  });
});
