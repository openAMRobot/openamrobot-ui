import React from "react";
import { act, fireEvent, render, screen } from "@testing-library/react";

import MapPage from "./MapPage";
import { RosContext, RosStatusContext, RuntimeConfigContext } from "../app/App";
import { AppConfig } from "../shared/constants";
import { loadRuntimeConfig } from "../shared/constants/runtimeConfig";
import { clearLocalStop, resetMotionGate } from "../shared/motion/motionGate";
import { navGoalBlockedReason } from "../shared/motion/navGoalGate";
import { attachRos, releaseAll } from "../shared/motion/uiCommand";
import { installMockRoslib, uninstallMockRoslib } from "../shared/motion/testing/mockRoslib";
import { openMotionGate } from "../shared/motion/testing/gate";
import { getUiRun, setUiRun } from "../shared/runs/uiRunStore";

jest.mock("react-joystick-component", () => ({ Joystick: () => <div /> }));
jest.mock("three/examples/jsm/controls/OrbitControls.js", () => ({ OrbitControls: function OrbitControls() {} }));
jest.mock("urdf-loader", () => function URDFLoader() {});

// The page calls navGoalBlockedReason(gate) while rendering and
// navGoalBlockedReason() when it dispatches a goal. Only the dispatch-time
// calls are scripted, to force the stop lock to engage between the queue's
// precheck and its first goal (review of #54).
// CRA resets mock implementations before each test, so the implementation
// is set in beforeEach.
let mockDispatchReasons = [];
jest.mock("../shared/motion/navGoalGate", () => ({
  ...jest.requireActual("../shared/motion/navGoalGate"),
  navGoalBlockedReason: jest.fn(),
}));
const { navGoalBlockedReason: realNavGoalBlockedReason } = jest.requireActual("../shared/motion/navGoalGate");

const LOCKED = "The software stop lock is engaged";

let mock;
let ros;
let nav2d;
beforeEach(() => {
  jest.useFakeTimers();
  mock = installMockRoslib();
  ros = new window.ROSLIB.Ros();
  resetMotionGate();
  attachRos(ros);
  setUiRun("route", null);
  mockDispatchReasons = [];
  navGoalBlockedReason.mockImplementation((...args) =>
    args.length || !mockDispatchReasons.length ? realNavGoalBlockedReason(...args) : mockDispatchReasons.shift(),
  );
  // NAV2D stand-in that keeps what the page stores on it (the pose
  // callback) and answers every other call with a no-op.
  nav2d = {};
  window.NAV2D = new Proxy(nav2d, { get: (t, k) => (k in t ? t[k] : () => {}) });
  window.HTMLCanvasElement.prototype.getContext = () => null;
  global.fetch = jest.fn(() => Promise.resolve({ ok: true, json: () => Promise.resolve({}) }));
});
afterEach(() => {
  clearLocalStop();
  releaseAll();
  attachRos(null);
  resetMotionGate();
  jest.useRealTimers();
  uninstallMockRoslib();
  delete window.NAV2D;
});

const renderWithOneWaypoint = () => {
  render(
    <RuntimeConfigContext.Provider value={{ config: loadRuntimeConfig(), updateConfig: () => {} }}>
      <RosContext.Provider value={ros}>
        <RosStatusContext.Provider value="connected">
          <MapPage />
        </RosStatusContext.Provider>
      </RosContext.Provider>
    </RuntimeConfigContext.Provider>,
  );
  fireEvent.click(screen.getByRole("button", { name: /Add Waypoint/ }));
  act(() => nav2d._poseCallback({ position: { x: 1, y: 2, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }));
};

const goalsSent = () => mock.topicsNamed(AppConfig.GOAL_POSE_TOPIC).flatMap((t) => t.published);

describe("Map page waypoint queue run", () => {
  it("a first goal that is sent reports the route run as running", () => {
    openMotionGate();
    renderWithOneWaypoint();
    fireEvent.click(screen.getByRole("button", { name: "Execute Queue" }));
    expect(goalsSent()).toHaveLength(1);
    expect(getUiRun("route")).toEqual(expect.objectContaining({ status: "running", step: 1, total: 1 }));
    expect(screen.getByRole("button", { name: "Stop Queue" })).toBeTruthy();
  });

  it("a first goal that is not sent ends the run as stopped, with the reason, and the queue is not executing", () => {
    openMotionGate();
    renderWithOneWaypoint();
    // Precheck passes; the lock engages before the first goal is dispatched.
    mockDispatchReasons = ["", LOCKED, LOCKED];
    fireEvent.click(screen.getByRole("button", { name: "Execute Queue" }));
    expect(getUiRun("route")).toEqual(expect.objectContaining({ status: "stopped", reason: LOCKED }));
    expect(goalsSent()).toHaveLength(0);
    expect(mockDispatchReasons).toHaveLength(0);
    expect(screen.queryByRole("button", { name: "Stop Queue" })).toBeNull();
    expect(screen.getByRole("button", { name: "Execute Queue" })).toBeTruthy();
  });
});
