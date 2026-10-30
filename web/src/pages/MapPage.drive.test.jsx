import React from "react";
import { act, fireEvent, render, screen } from "@testing-library/react";

import MapPage from "./MapPage";
import { RosContext, RosStatusContext, RuntimeConfigContext } from "../app/App";
import { ACTION_CANCEL_SERVICES, AppConfig } from "../shared/constants";
import { loadRuntimeConfig } from "../shared/constants/runtimeConfig";
import { resetMotionGate } from "../shared/motion/motionGate";
import { attachRos, releaseAll } from "../shared/motion/uiCommand";
import { installMockRoslib, uninstallMockRoslib } from "../shared/motion/testing/mockRoslib";
import { openMotionGate } from "../shared/motion/testing/gate";

let mockJoystickProps;
jest.mock("react-joystick-component", () => ({
  Joystick: (props) => {
    mockJoystickProps = props;
    return <div data-testid="stick" />;
  },
}));
jest.mock("three/examples/jsm/controls/OrbitControls.js", () => ({ OrbitControls: function OrbitControls() {} }));
jest.mock("urdf-loader", () => function URDFLoader() {});

let mock;
let ros;
beforeEach(() => {
  jest.useFakeTimers();
  mock = installMockRoslib();
  ros = new window.ROSLIB.Ros();
  resetMotionGate();
  attachRos(ros);
  window.NAV2D = new Proxy({}, { get: () => () => {} });
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

const renderMapPage = () =>
  render(
    <RuntimeConfigContext.Provider value={{ config: loadRuntimeConfig(), updateConfig: () => {} }}>
      <RosContext.Provider value={ros}>
        <RosStatusContext.Provider value="connected">
          <MapPage />
        </RosStatusContext.Provider>
      </RosContext.Provider>
    </RuntimeConfigContext.Provider>,
  );

describe("Map page drive controls", () => {
  it("the map page joystick publishes only on /ui/cmd_vel", () => {
    openMotionGate();
    renderMapPage();
    act(() => mockJoystickProps.move({ x: 0, y: 1 }));
    act(() => jest.advanceTimersByTime(200));
    act(() => mockJoystickProps.stop());
    const velocity = mock.registry.topics.filter((t) => /Twist/.test(t.messageType));
    expect(velocity.map((t) => t.name)).toEqual(["/ui/cmd_vel"]);
    expect(velocity[0].published.length).toBeGreaterThanOrEqual(5);
  });

  it("the map page Stop (software) engages the lock and cancels each action once", () => {
    renderMapPage();
    fireEvent.click(screen.getByRole("button", { name: "Stop (software)" }));
    expect(mock.callsTo(AppConfig.STOP_LOCK_ENGAGE_SERVICE)).toHaveLength(1);
    ACTION_CANCEL_SERVICES.forEach((name) => expect(mock.callsTo(name)).toHaveLength(1));
    expect(mock.callsTo(AppConfig.STOP_LOCK_RELEASE_SERVICE)).toHaveLength(0);
  });
});
