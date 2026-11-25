/**
 * Owner decision (PR #50, item 1): while the stop lock is engaged or
 * unknown, no Nav2 goal is sent from the map page, missions, routes,
 * programs or docking, and the reason is shown. A goal accepted while
 * locked would otherwise wait and run after Resume.
 */
import React from "react";
import { act, fireEvent, render, screen } from "@testing-library/react";

import MapPage from "../../pages/MapPage";
import DockingControl from "../../components/DockingControl";
import MissionRunner from "../../components/MissionRunner";
import { RosContext, RosStatusContext, RuntimeConfigContext } from "../../app/App";
import { AppConfig } from "../constants";
import { loadRuntimeConfig } from "../constants/runtimeConfig";
import { createRobotActionClient } from "../../features/blocks/robotActions";
import { getRun, requestStart, setRun } from "../missions/missionRunner";
import { getMission } from "../missions/missions";
import { resetMotionGate, setConnection, setLockState } from "./motionGate";
import { attachRos, releaseAll } from "./uiCommand";
import { installMockRoslib, uninstallMockRoslib } from "./testing/mockRoslib";

jest.mock("three/examples/jsm/controls/OrbitControls.js", () => ({ OrbitControls: function OrbitControls() {} }));
jest.mock("urdf-loader", () => function URDFLoader() {});
jest.mock("../missions/missions", () => ({ ...jest.requireActual("../missions/missions"), getMission: jest.fn() }));

let mock;
let ros;

beforeEach(() => {
  jest.useFakeTimers();
  mock = installMockRoslib();
  ros = new window.ROSLIB.Ros();
  resetMotionGate();
  attachRos(ros);
  setRun(null);
  const nav2d = {};
  window.NAV2D = new Proxy(nav2d, {
    get: (target, key) => (key in target ? target[key] : () => {}),
    set: (target, key, value) => {
      target[key] = value;
      return true;
    },
  });
  window.HTMLCanvasElement.prototype.getContext = () => null;
  global.fetch = jest.fn(() => Promise.resolve({ ok: true, json: () => Promise.resolve({}) }));
});

afterEach(() => {
  releaseAll();
  attachRos(null);
  resetMotionGate();
  jest.useRealTimers();
  uninstallMockRoslib();
  delete window.NAV2D;
});

const goalsSent = () => mock.topicsNamed(AppConfig.GOAL_POSE_TOPIC).flatMap((t) => t.published);
const pose = { position: { x: 1, y: 2, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };

const withRos = (children) => (
  <RuntimeConfigContext.Provider value={{ config: loadRuntimeConfig(), updateConfig: () => {} }}>
    <RosContext.Provider value={ros}>
      <RosStatusContext.Provider value="connected">{children}</RosStatusContext.Provider>
    </RosContext.Provider>
  </RuntimeConfigContext.Provider>
);

describe("map page goals and the stop lock", () => {
  it("sends goals while the lock is released (positive control)", () => {
    setConnection("connected");
    setLockState("released");
    render(withRos(<MapPage />));
    fireEvent.click(screen.getByRole("button", { name: /go home/i }));
    expect(goalsSent()).toHaveLength(1);
    expect(screen.queryByTestId("nav-goals-disabled")).toBeNull();
  });

  it.each([
    ["engaged", "locked", /Stopped \(software\)/],
    ["unknown", "unknown", /Stop lock state unknown/],
  ])("while the lock is %s: goal controls are disabled and show the reason", (_label, lock, reason) => {
    setConnection("connected");
    setLockState(lock);
    render(withRos(<MapPage />));
    expect(screen.getByTestId("nav-goals-disabled").textContent).toMatch(reason);
    const home = screen.getByRole("button", { name: /go home/i });
    expect(home.disabled).toBe(true);
    expect(screen.getByRole("button", { name: /send goal/i }).disabled).toBe(true);
    fireEvent.click(home);
    expect(goalsSent()).toHaveLength(0);
  });

  it("a goal click after the lock engages is not sent, even in goal mode", () => {
    setConnection("connected");
    setLockState("released");
    render(withRos(<MapPage />));
    fireEvent.click(screen.getByRole("button", { name: /send goal/i }));
    act(() => setLockState("locked"));
    act(() => window.NAV2D._poseCallback(pose));
    expect(goalsSent()).toHaveLength(0);
  });

  it("docking is disabled while the lock is engaged", () => {
    setConnection("connected");
    setLockState("locked");
    render(withRos(<DockingControl compact />));
    expect(screen.getByRole("button", { name: "Dock" }).disabled).toBe(true);
    expect(screen.getByRole("button", { name: "Undock" }).disabled).toBe(true);
    expect(screen.getByText(/Docking disabled: Stopped \(software\)/)).toBeTruthy();
    const triggers = mock.topicsNamed(AppConfig.DOCK_TRIGGER_TOPIC).flatMap((t) => t.published);
    expect(triggers).toHaveLength(0);
  });
});

describe("missions, programs and the stop lock", () => {
  it("a mission step does not send its goal once the lock engages mid-run", async () => {
    getMission.mockReturnValue({ name: "Patrol", steps: [{ type: "wait", seconds: 1 }, { type: "home" }] });
    setConnection("connected");
    setLockState("released");
    render(withRos(<MissionRunner />));
    act(() => requestStart("m1"));
    expect(getRun().status).toBe("running");
    act(() => setLockState("locked"));
    await act(async () => {
      jest.advanceTimersByTime(1000);
    });
    await act(async () => {});
    expect(goalsSent()).toHaveLength(0);
    expect(getRun().status).toBe("failed");
  });

  it.each(["locked", "unknown"])("a program navigate step fails without a goal while the lock is %s", async (lock) => {
    setConnection("connected");
    setLockState(lock);
    const client = createRobotActionClient(ros);
    await expect(client.runAction({ type: "navigate", x: 1, y: 2, yaw: 0 })).rejects.toThrow(/Navigation goal not sent/);
    await expect(client.runAction({ type: "dock" })).rejects.toThrow(/Dock not started/);
    expect(goalsSent()).toHaveLength(0);
  });
});
