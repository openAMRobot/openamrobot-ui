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
    // Mounting while connected aborts nothing (review of #50).
    expect(aborted).not.toHaveBeenCalled();
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

  // Review of #50 (Kunal, 21 Nov): the release and abort ran on every run of
  // the effect, so a remount while connected dropped held input and aborted
  // running programs and missions with no change in connection or lock.
  it("a remount while connected does not abort programs or missions", () => {
    const aborted = jest.fn();
    const unsubscribe = subscribeAbort(aborted);
    const first = render(<MotionBridge />);
    act(() => latest(AppConfig.STOP_LOCK_TOPIC).emit({ data: false }));
    first.unmount();
    render(<MotionBridge />);
    expect(aborted).not.toHaveBeenCalled();
    unsubscribe();
  });

  it("a disconnect still aborts programs and missions", () => {
    const aborted = jest.fn();
    const unsubscribe = subscribeAbort(aborted);
    const { rerender } = render(<MotionBridge />);
    act(() => latest(AppConfig.STOP_LOCK_TOPIC).emit({ data: false }));
    mockRosStatus = "disconnected";
    rerender(<MotionBridge />);
    expect(aborted).toHaveBeenCalledTimes(1);
    expect(aborted.mock.calls[0][0]).toBe("rosbridge disconnected or reconnecting");
    unsubscribe();
  });
});
