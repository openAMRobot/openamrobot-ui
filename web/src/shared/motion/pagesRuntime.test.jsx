import React from "react";
import { act, render } from "@testing-library/react";
import { MemoryRouter } from "react-router-dom";
import { Provider } from "react-redux";

import { RosContext, RosStatusContext, RuntimeConfigContext } from "../../app/App";
import { store } from "../../app/store";
import Routes from "../../pages";
import { loadRuntimeConfig } from "../constants/runtimeConfig";
import { installMockRoslib, uninstallMockRoslib } from "./testing/mockRoslib";
import { attachRos, releaseAll } from "./uiCommand";
import { resetMotionGate } from "./motionGate";

// three.js example modules and urdf-loader ship as ES modules that CRA's Jest
// does not transform. They only serve the Robot page's 3D view, which is not
// rendered here, so stub them.
jest.mock("three/examples/jsm/controls/OrbitControls.js", () => ({ OrbitControls: function OrbitControls() {} }));
jest.mock("urdf-loader", () => function URDFLoader() {});

// Pages that render in jsdom. The Robot page needs WebGL (three.js), which
// jsdom does not provide; its velocity-free hooks are covered by the static
// guard test instead.
const PAGES = [
  "/",
  "/route",
  "/maps",
  "/blocks",
  "/scheduler",
  "/missions",
  "/info",
  "/devices",
  "/health",
  "/metrics",
  "/recordings",
  "/events",
  "/console",
  "/params",
  "/fleet",
  "/config",
];

const VELOCITY_TYPE = /(^|\/)Twist(Stamped)?$/;

let mock;
beforeEach(() => {
  jest.useFakeTimers();
  mock = installMockRoslib();
  global.fetch = jest.fn(() =>
    Promise.resolve({
      ok: true,
      json: () => Promise.resolve({}),
      text: () => Promise.resolve(""),
      arrayBuffer: () => Promise.resolve(new ArrayBuffer(0)),
    }),
  );
  window.NAV2D = new Proxy({}, { get: () => () => {} });
  window.ROS2D = new Proxy({}, { get: () => function Stub() {} });
  window.HTMLCanvasElement.prototype.getContext = () => null;
  window.matchMedia =
    window.matchMedia ||
    (() => ({ matches: false, addListener() {}, removeListener() {}, addEventListener() {}, removeEventListener() {} }));
  window.ResizeObserver =
    window.ResizeObserver ||
    class {
      observe() {}
      unobserve() {}
      disconnect() {}
    };
});

afterEach(() => {
  releaseAll();
  attachRos(null);
  resetMotionGate();
  jest.useRealTimers();
  uninstallMockRoslib();
  delete window.NAV2D;
  delete window.ROS2D;
});

const renderAt = (path, ros) =>
  render(
    <Provider store={store}>
      <RuntimeConfigContext.Provider value={{ config: loadRuntimeConfig(), updateConfig: () => {} }}>
        <RosContext.Provider value={ros}>
          <RosStatusContext.Provider value="connected">
            <MemoryRouter initialEntries={[path]}>
              <Routes />
            </MemoryRouter>
          </RosStatusContext.Provider>
        </RosContext.Provider>
      </RuntimeConfigContext.Provider>
    </Provider>,
  );

describe("velocity publishers with the real pages mounted", () => {
  it.each(PAGES)("page %s advertises no velocity topic other than /ui/cmd_vel", (path) => {
    const ros = new window.ROSLIB.Ros();
    let view;
    act(() => {
      view = renderAt(path, ros);
    });
    act(() => {
      jest.advanceTimersByTime(3000);
    });

    const velocityTopics = mock.registry.topics.filter((t) => VELOCITY_TYPE.test(t.messageType || ""));
    expect(velocityTopics.map((t) => [t.name, t.messageType])).toEqual([
      ["/ui/cmd_vel", "geometry_msgs/TwistStamped"],
    ]);
    const cmdVelNames = mock.registry.topics
      .map((t) => t.name)
      .filter((name) => /cmd_vel/.test(name || ""));
    expect(cmdVelNames).toEqual(["/ui/cmd_vel"]);
    // Mounted and idle: nothing is published on it.
    expect(velocityTopics[0].published).toHaveLength(0);
    view.unmount();
  });
});
