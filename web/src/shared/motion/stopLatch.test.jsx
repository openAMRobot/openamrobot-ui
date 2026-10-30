/**
 * Review of #50 (P1): Stop (software) must latch locally at once. The
 * engage request is asynchronous; until a lock sample arrives, the gate
 * would otherwise still read connected + released + synced, and a held
 * gamepad or a goal click would get through. These tests leave the engage
 * request pending, reject it, or answer it without any lock-topic update,
 * and check that held input and goal submissions stay blocked until a
 * confirmed Resume.
 */
import React from "react";
import { act, fireEvent, render, screen } from "@testing-library/react";

import JoystickComponent from "../../components/Joystick";
import MotionStopBanner from "../../components/MotionStopBanner";
import MapPage from "../../pages/MapPage";
import { RosContext, RosStatusContext, RuntimeConfigContext } from "../../app/App";
import { AppConfig } from "../constants";
import { loadRuntimeConfig } from "../constants/runtimeConfig";
import { createRobotActionClient } from "../../features/blocks/robotActions";
import { clearLocalStop, getMotionGate, resetMotionGate, setLockState } from "./motionGate";
import { navGoalBlockedReason } from "./navGoalGate";
import { resumeFromSoftwareStop, softwareStop } from "./softwareStop";
import { attachRos, holdVelocity, releaseAll } from "./uiCommand";
import { installMockRoslib, uninstallMockRoslib } from "./testing/mockRoslib";
import { openMotionGate } from "./testing/gate";

jest.mock("three/examples/jsm/controls/OrbitControls.js", () => ({ OrbitControls: function OrbitControls() {} }));
jest.mock("urdf-loader", () => function URDFLoader() {});
jest.mock("../events/eventLog", () => ({ addEvent: jest.fn() }));

let mock;
let ros;
let cmdVel;
let pads;

const ENGAGE_REPLIES = {
  pending: () => {},
  rejected: (req, ok, fail) => fail("service not available"),
  "answered, no lock update": (req, ok) => ok({ success: true, message: "engaged" }),
};

beforeEach(() => {
  jest.useFakeTimers();
  mock = installMockRoslib();
  ros = new window.ROSLIB.Ros();
  resetMotionGate();
  attachRos(ros);
  [cmdVel] = mock.topicsNamed(AppConfig.UI_CMD_VEL_TOPIC);
  pads = [null];
  navigator.getGamepads = () => pads;
  const nav2d = {};
  window.NAV2D = new Proxy(nav2d, {
    get: (t, k) => (k in t ? t[k] : () => {}),
    set: (t, k, v) => {
      t[k] = v;
      return true;
    },
  });
  window.HTMLCanvasElement.prototype.getContext = () => null;
  global.fetch = jest.fn(() => Promise.resolve({ ok: true, json: () => Promise.resolve({}) }));
});

afterEach(() => {
  releaseAll();
  clearLocalStop();
  attachRos(null);
  resetMotionGate();
  jest.useRealTimers();
  uninstallMockRoslib();
  delete navigator.getGamepads;
  delete window.NAV2D;
});

const linearX = () => cmdVel.published.map((m) => m.twist.linear.x);
const goals = () => mock.topicsNamed(AppConfig.GOAL_POSE_TOPIC).flatMap((t) => t.published);

describe.each(Object.keys(ENGAGE_REPLIES))("Stop with the engage request %s", (reply) => {
  beforeEach(() => mock.onService(AppConfig.STOP_LOCK_ENGAGE_SERVICE, ENGAGE_REPLIES[reply]));

  it("held input stays blocked after Stop, with no lock-topic update", () => {
    openMotionGate();
    expect(holdVelocity("gamepad", { linear: 0.1 }).ok).toBe(true);
    softwareStop(ros, "test");
    // The lock topic still says released: the local latch must hold.
    expect(getMotionGate().lock).toBe("released");
    for (let i = 0; i < 20; i += 1) {
      expect(holdVelocity("gamepad", { linear: 0.1 }).ok).toBe(false);
      jest.advanceTimersByTime(17);
    }
    jest.advanceTimersByTime(1000);
    expect(linearX()).toEqual([0.1, 0]);
  });

  it("goal submissions stay blocked after Stop", async () => {
    openMotionGate();
    softwareStop(ros, "test");
    expect(navGoalBlockedReason()).toMatch(/Stopped \(software\)/);
    const client = createRobotActionClient(ros);
    await expect(client.runAction({ type: "navigate", x: 1, y: 1, yaw: 0 })).rejects.toThrow(/Navigation goal not sent/);
    expect(goals()).toHaveLength(0);
  });
});

describe("the local latch", () => {
  it("is not cleared by a stale or fresh released sample, nor by a reconnect", () => {
    mock.onService(AppConfig.STOP_LOCK_ENGAGE_SERVICE, () => {});
    openMotionGate();
    softwareStop(ros, "test");
    setLockState("unknown");
    setLockState("released");
    expect(getMotionGate().canDrive).toBe(false);
    resetMotionGate("connected");
    openMotionGate();
    expect(getMotionGate().canDrive).toBe(false);
    expect(navGoalBlockedReason()).not.toBe("");
  });

  it("is cleared only by a Resume whose release succeeds", async () => {
    mock.onService(AppConfig.STOP_LOCK_ENGAGE_SERVICE, () => {});
    mock.onService(AppConfig.STOP_LOCK_RELEASE_SERVICE, (req, ok, fail) => fail("no service"));
    openMotionGate();
    softwareStop(ros, "test");
    await resumeFromSoftwareStop(ros);
    expect(getMotionGate().canDrive).toBe(false);

    mock.onService(AppConfig.STOP_LOCK_RELEASE_SERVICE, (req, ok) => ok({ success: true, message: "released" }));
    await resumeFromSoftwareStop(ros);
    expect(getMotionGate().canDrive).toBe(true);
    expect(holdVelocity("joystick", { linear: 0.05 }).ok).toBe(true);
  });
});

describe("components while the engage request is pending", () => {
  const withRos = (children) => (
    <RuntimeConfigContext.Provider value={{ config: loadRuntimeConfig(), updateConfig: () => {} }}>
      <RosContext.Provider value={ros}>
        <RosStatusContext.Provider value="connected">{children}</RosStatusContext.Provider>
      </RosContext.Provider>
    </RuntimeConfigContext.Provider>
  );

  beforeEach(() => mock.onService(AppConfig.STOP_LOCK_ENGAGE_SERVICE, () => {}));

  it("a deflected gamepad publishes no motion after Stop, and needs neutral after Resume", async () => {
    mock.onService(AppConfig.STOP_LOCK_RELEASE_SERVICE, (req, ok) => ok({ success: true, message: "" }));
    openMotionGate();
    render(withRos(<JoystickComponent />));
    pads = [{ axes: [0, -1] }];
    act(() => jest.advanceTimersByTime(100));
    expect(linearX().length).toBeGreaterThan(0);
    act(() => {
      softwareStop(ros, "test");
    });
    const afterStop = cmdVel.published.length;
    act(() => jest.advanceTimersByTime(1000));
    expect(cmdVel.published.length).toBe(afterStop);
    expect(linearX()[afterStop - 1]).toBe(0);

    await act(async () => {
      await resumeFromSoftwareStop(ros);
    });
    act(() => jest.advanceTimersByTime(500));
    expect(cmdVel.published.length).toBe(afterStop); // still deflected: needs neutral
    pads = [{ axes: [0, 0] }];
    act(() => jest.advanceTimersByTime(50));
    pads = [{ axes: [0, -1] }];
    act(() => jest.advanceTimersByTime(50));
    expect(cmdVel.published.length).toBeGreaterThan(afterStop);
  });

  it("the banner shows Stopped (software) with Resume, though the lock topic says released", async () => {
    mock.onService(AppConfig.STOP_LOCK_RELEASE_SERVICE, (req, ok) => ok({ success: true, message: "" }));
    openMotionGate();
    render(withRos(<MotionStopBanner />));
    expect(screen.queryByText("Stopped (software)")).toBeNull();
    act(() => {
      softwareStop(ros, "test");
    });
    expect(screen.getByText("Stopped (software)")).toBeTruthy();
    const resume = screen.getByRole("button", { name: "Resume" });
    expect(resume.disabled).toBe(false);
    fireEvent.click(resume);
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "Confirm resume" }));
    });
    expect(mock.callsTo(AppConfig.STOP_LOCK_RELEASE_SERVICE)).toHaveLength(1);
    expect(screen.queryByText("Stopped (software)")).toBeNull();
  });

  it("map page goal controls are disabled right after Stop", () => {
    openMotionGate();
    render(withRos(<MapPage />));
    expect(screen.getByRole("button", { name: /go home/i }).disabled).toBe(false);
    fireEvent.click(screen.getByRole("button", { name: "Stop (software)" }));
    expect(screen.getByRole("button", { name: /go home/i }).disabled).toBe(true);
    expect(screen.getByTestId("nav-goals-disabled").textContent).toMatch(/Stopped \(software\)/);
    fireEvent.click(screen.getByRole("button", { name: /go home/i }));
    expect(goals()).toHaveLength(0);
  });
});
