import React from "react";
import { act, render } from "@testing-library/react";

import MissionRunner from "./MissionRunner";
import { abortUiMotion } from "../shared/motion/abortBus";
import { resetMotionGate, setConnection, setLockState } from "../shared/motion/motionGate";
import { getRun, requestStart, setRun } from "../shared/missions/missionRunner";
import { getMission } from "../shared/missions/missions";
import { installMockRoslib, uninstallMockRoslib } from "../shared/motion/testing/mockRoslib";

jest.mock("../app/App", () => ({ useRos: () => ({}) }));
jest.mock("../shared/events/eventLog", () => ({ addEvent: jest.fn() }));
jest.mock("react-toastify", () => ({ toast: { warn: jest.fn(), error: jest.fn(), success: jest.fn() } }));
jest.mock("../shared/missions/missions", () => ({ getMission: jest.fn() }));

beforeEach(() => {
  installMockRoslib();
  resetMotionGate();
  setRun(null);
  getMission.mockReturnValue({ name: "Patrol", steps: [{ type: "wait", seconds: 60 }] });
});
afterEach(() => {
  resetMotionGate();
  uninstallMockRoslib();
});

describe("MissionRunner and the stop lock", () => {
  it("does not start a mission while the lock is locked or unknown", () => {
    render(<MissionRunner />);
    setConnection("connected");
    setLockState("locked");
    act(() => requestStart("m1"));
    expect(getRun()).toBeNull();
    setLockState("unknown");
    act(() => requestStart("m1"));
    expect(getRun()).toBeNull();
  });

  it("an abort (Stop or a locked lock) stops a running mission", async () => {
    render(<MissionRunner />);
    setConnection("connected");
    setLockState("released");
    act(() => requestStart("m1"));
    expect(getRun().status).toBe("running");
    await act(async () => {
      abortUiMotion("Stopped (software)");
    });
    expect(getRun().status).toBe("stopped");
  });
});
