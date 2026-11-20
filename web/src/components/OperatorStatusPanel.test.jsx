import React from "react";
import { act, render, screen } from "@testing-library/react";

import OperatorStatusPanel from "./OperatorStatusPanel";
import { AppConfig } from "../shared/constants";
import { resetMotionGate, setConnection, setLockState } from "../shared/motion/motionGate";
import { installMockRoslib, uninstallMockRoslib } from "../shared/motion/testing/mockRoslib";

let mockRosStatus = "connected";
// One stable connection object, as App.jsx provides.
const mockRos = {};
jest.mock("../app/App", () => ({ useRos: () => mockRos, useRosStatus: () => mockRosStatus }));

let mock;
let clock;
const now = () => clock;

beforeEach(() => {
  jest.useFakeTimers();
  clock = 1000;
  mockRosStatus = "connected";
  mock = installMockRoslib();
  resetMotionGate();
});
afterEach(() => {
  resetMotionGate();
  uninstallMockRoslib();
  jest.useRealTimers();
});

const emit = (topic, msg) => act(() => mock.topicsNamed(topic).at(-1).emit(msg));
const display = () => screen.getByTestId("operator-status-display").textContent;

describe("OperatorStatusPanel", () => {
  it("shows a refused Resume reason within 1 s of the gate publishing it", () => {
    setConnection("connected");
    setLockState("locked");
    render(<OperatorStatusPanel now={now} />);
    emit(AppConfig.STOP_STATE_TOPIC, { state: 1, reason: "cmd_vel_nav not zero" });
    act(() => {
      clock += 500;
      jest.advanceTimersByTime(500);
    });
    expect(display()).toBe("STOPPED");
    expect(screen.getByTestId("operator-status-stop").textContent).toBe(
      "RELEASE_REFUSED: cmd_vel_nav not zero",
    );
  });

  it("goes from READY to STALE when /navigation/status stops", () => {
    setConnection("connected");
    setLockState("released");
    render(<OperatorStatusPanel now={now} />);
    emit(AppConfig.ROBOT_POSE_TOPIC, {});
    emit(AppConfig.NAVIGATION_STATUS_TOPIC, {
      health: 1,
      navigation_readiness: 1,
      not_ready_reasons: [],
      active_reasons: [],
      stack: { state: 3 },
      task: { state: 1 },
    });
    expect(display()).toBe("READY");
    act(() => {
      clock += 7000;
      jest.advanceTimersByTime(7000);
    });
    expect(display()).toBe("STALE");
  });

  it("shows DISCONNECTED and subscribes to nothing without rosbridge", () => {
    mockRosStatus = "disconnected";
    render(<OperatorStatusPanel now={now} />);
    expect(display()).toBe("DISCONNECTED");
    expect(mock.registry.topics).toEqual([]);
  });

  it("never publishes or calls a service", () => {
    setConnection("connected");
    setLockState("released");
    render(<OperatorStatusPanel now={now} />);
    act(() => {
      clock += 3000;
      jest.advanceTimersByTime(3000);
    });
    mock.registry.topics.forEach((t) => expect(t.published).toEqual([]));
    expect(mock.registry.services).toEqual([]);
  });
});
