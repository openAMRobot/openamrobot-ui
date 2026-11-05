/**
 * Read-only guarantee for Reporting Mode: nothing under shared/reporting or
 * the Reporting page advertises, publishes, or calls a service or action.
 * Checked twice: at runtime with a mocked ROSLIB while the page renders and
 * receives data, and statically over the source files.
 */
import fs from "fs";
import path from "path";
import React from "react";
import { act, render } from "@testing-library/react";
import { MemoryRouter } from "react-router-dom";

import ReportingPage from "../../pages/ReportingPage";
import { RosContext, RosStatusContext } from "../../app/App";
import { AppConfig } from "../constants";
import { installMockRoslib, uninstallMockRoslib } from "../motion/testing/mockRoslib";
import { battery, diagnostics, goalStatus, lock, navFeedback, navStatus } from "./testing/fixtures";

// The App module pulls in every page; three.js examples and urdf-loader ship
// as ES modules that CRA's Jest does not transform. Not used here.
jest.mock("three/examples/jsm/controls/OrbitControls.js", () => ({ OrbitControls: function OrbitControls() {} }));
jest.mock("urdf-loader", () => function URDFLoader() {});
jest.mock("../events/eventLog", () => ({ addEvent: jest.fn() }));

describe("Reporting Mode is read-only", () => {
  it("at runtime: no publish, no advertise, no service or action client", () => {
    jest.useFakeTimers();
    const mock = installMockRoslib();
    let actionClients = 0;
    window.ROSLIB.ActionClient = function ActionClient() {
      actionClients += 1;
    };
    const ros = new window.ROSLIB.Ros();
    const view = render(
      <RosContext.Provider value={ros}>
        <RosStatusContext.Provider value="connected">
          <MemoryRouter initialEntries={["/report"]}>
            <ReportingPage />
          </MemoryRouter>
        </RosStatusContext.Provider>
      </RosContext.Provider>,
    );
    const emit = (name, msg) => act(() => mock.topicsNamed(name).forEach((t) => t.emit(msg)));
    emit(AppConfig.STOP_LOCK_TOPIC, lock(true));
    emit(AppConfig.STOP_LOCK_TOPIC, lock(false));
    emit(AppConfig.NAVIGATION_STATUS_TOPIC, navStatus());
    emit(AppConfig.DIAGNOSTICS_TOPIC, diagnostics(0));
    emit(AppConfig.BATTERY_TOPIC, battery(10));
    emit(AppConfig.NAV_STATUS_TOPIC, goalStatus(1, 2));
    emit(AppConfig.NAV_FEEDBACK_TOPIC, navFeedback(1, 3));
    emit(AppConfig.NAV_STATUS_TOPIC, goalStatus(1, 99));
    act(() => jest.advanceTimersByTime(5000));

    expect(mock.registry.topics.length).toBeGreaterThan(0);
    mock.registry.topics.forEach((t) => {
      expect(t.published).toHaveLength(0);
      expect(t.advertised).toBe(false);
    });
    expect(mock.registry.services).toHaveLength(0);
    expect(actionClients).toBe(0);
    view.unmount();
    uninstallMockRoslib();
    jest.useRealTimers();
  });

  it("statically: no command calls or command modules in shared/reporting or the page", () => {
    const root = path.resolve(__dirname, "../..");
    const files = [
      ...fs
        .readdirSync(__dirname, { withFileTypes: true })
        .filter((e) => e.isFile() && /\.(js|jsx)$/.test(e.name) && !/\.test\./.test(e.name))
        .map((e) => path.join(__dirname, e.name)),
      path.join(root, "pages", "ReportingPage.jsx"),
    ];
    expect(files.length).toBeGreaterThanOrEqual(4);
    const forbidden = [
      /\.publish\(/,
      /\.advertise\(/,
      /\.callService\(/,
      /ROSLIB\.Service\b/,
      /ROSLIB\.ActionClient\b/,
      /ROSLIB\.Action\b/,
      /\bengageStopLock\b/,
      /\breleaseStopLock\b/,
      /\bsoftwareStop\b/,
      /\bresumeFromSoftwareStop\b/,
      /\bholdVelocity\b/,
      /motion\/uiCommand/,
      /motion\/softwareStop/,
      /motion\/MotionBridge/,
      /requestStart|requestStop/,
    ];
    const offenders = [];
    files.forEach((file) => {
      const text = fs.readFileSync(file, "utf8");
      forbidden.forEach((re) => {
        if (re.test(text)) offenders.push(`${path.relative(root, file)} ${re}`);
      });
    });
    expect(offenders).toEqual([]);
  });
});
