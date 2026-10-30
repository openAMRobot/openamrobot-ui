import React from "react";
import { act, fireEvent, render, screen } from "@testing-library/react";
import { MemoryRouter, Route, Routes } from "react-router-dom";

import ReportingPage, { EXIT_HOLD_MS } from "./ReportingPage";
import { RosContext, RosStatusContext } from "../app/App";
import { AppConfig } from "../shared/constants";
import { installMockRoslib, uninstallMockRoslib } from "../shared/motion/testing/mockRoslib";
import { battery, diagnostics, goalStatus, lock, navFeedback, navStatus } from "../shared/reporting/testing/fixtures";

// The App module pulls in every page; three.js examples and urdf-loader ship
// as ES modules that CRA's Jest does not transform. Not used here.
jest.mock("three/examples/jsm/controls/OrbitControls.js", () => ({ OrbitControls: function OrbitControls() {} }));
jest.mock("urdf-loader", () => function URDFLoader() {});
jest.mock("../shared/events/eventLog", () => ({ addEvent: jest.fn() }));

let mock;
let ros;
let spoken;

beforeEach(() => {
  jest.useFakeTimers();
  mock = installMockRoslib();
  ros = new window.ROSLIB.Ros();
  spoken = [];
  window.SpeechSynthesisUtterance = function Utterance(text) {
    this.text = text;
  };
  window.speechSynthesis = { speak: (u) => spoken.push(u.text), cancel: jest.fn() };
});
afterEach(() => {
  jest.useRealTimers();
  uninstallMockRoslib();
  delete window.speechSynthesis;
  delete window.SpeechSynthesisUtterance;
});

const renderPage = (status = "connected") =>
  render(
    <RosContext.Provider value={ros}>
      <RosStatusContext.Provider value={status}>
        <MemoryRouter initialEntries={["/report"]}>
          <Routes>
            <Route path="/report" element={<ReportingPage />} />
            <Route path="/" element={<div>Operator home</div>} />
          </Routes>
        </MemoryRouter>
      </RosStatusContext.Provider>
    </RosContext.Provider>,
  );

const emit = (name, msg) =>
  act(() => {
    mock.topicsNamed(name).forEach((t) => t.emit(msg));
  });

const feedHealthy = () => {
  emit(AppConfig.STOP_LOCK_TOPIC, lock(false));
  emit(AppConfig.NAVIGATION_STATUS_TOPIC, navStatus());
  emit(AppConfig.DIAGNOSTICS_TOPIC, diagnostics(0));
  emit(AppConfig.BATTERY_TOPIC, battery(80));
};

const state = () => screen.getByTestId("reporting-page").dataset.state;

describe("Reporting Mode page", () => {
  it("says its scope and lists Arms, Lift and Physical E-stop as Not reported", () => {
    renderPage();
    expect(screen.getByText("Base and navigation")).toBeTruthy();
    ["Arms", "Lift", "Physical E-stop"].forEach((label) =>
      expect(screen.getByTestId(`row-${label}`).textContent).toContain("Not reported"),
    );
  });

  it("renders READY, EXECUTING with progress, STOPPED (SOFTWARE), FAULTED, stale and DISCONNECTED", () => {
    const { unmount } = renderPage();
    expect(state()).toBe("UNKNOWN");
    feedHealthy();
    expect(state()).toBe("READY");
    expect(screen.getByTestId("reporting-page")).toMatchSnapshot("READY");

    emit(AppConfig.NAV_STATUS_TOPIC, goalStatus(1, 2));
    emit(AppConfig.NAV_FEEDBACK_TOPIC, navFeedback(1, 8));
    emit(AppConfig.NAV_FEEDBACK_TOPIC, navFeedback(1, 2, 4));
    expect(state()).toBe("EXECUTING");
    expect(screen.getByTestId("reporting-progress").textContent).toContain("2.0 m remaining");
    expect(screen.getByTestId("reporting-page")).toMatchSnapshot("EXECUTING");

    emit(AppConfig.STOP_LOCK_TOPIC, lock(true));
    expect(state()).toBe("STOPPED (SOFTWARE)");
    expect(screen.getByTestId("reporting-page")).toMatchSnapshot("STOPPED (SOFTWARE)");

    emit(AppConfig.STOP_LOCK_TOPIC, lock(false));
    emit(AppConfig.NAVIGATION_STATUS_TOPIC, navStatus({ health: 3, active_reasons: [4009] }));
    expect(state()).toBe("FAULTED");
    expect(screen.getByTestId("reporting-page")).toMatchSnapshot("FAULTED");
    unmount();
  });

  it("shows a stale source as stale, never as healthy", () => {
    let nowMs = 1000;
    const spy = jest.spyOn(Date, "now").mockImplementation(() => nowMs);
    renderPage();
    feedHealthy();
    nowMs += 3500; // battery (3 s) and everything else stale
    emit(AppConfig.STOP_LOCK_TOPIC, lock(false));
    emit(AppConfig.NAVIGATION_STATUS_TOPIC, navStatus());
    emit(AppConfig.DIAGNOSTICS_TOPIC, diagnostics(0));
    expect(state()).toBe("WAITING");
    expect(screen.getByTestId("row-Battery").textContent).toContain("(stale)");
    expect(screen.getByTestId("reporting-page")).toMatchSnapshot("stale battery");
    spy.mockRestore();
  });

  it("DISCONNECTED when rosbridge is down", () => {
    renderPage("disconnected");
    expect(state()).toBe("DISCONNECTED");
    expect(screen.getByTestId("reporting-page")).toMatchSnapshot("DISCONNECTED");
  });

  it("leaves only after a long press on the corner control", () => {
    renderPage();
    const exit = screen.getByRole("button", { name: /hold to exit/i });
    fireEvent.pointerDown(exit);
    act(() => jest.advanceTimersByTime(EXIT_HOLD_MS - 100));
    fireEvent.pointerUp(exit);
    act(() => jest.advanceTimersByTime(1000));
    expect(screen.queryByText("Operator home")).toBeNull();
    fireEvent.pointerDown(exit);
    act(() => jest.advanceTimersByTime(EXIT_HOLD_MS));
    expect(screen.getByText("Operator home")).toBeTruthy();
  });

  it("speech is off by default and, when on, speaks each change once", () => {
    renderPage();
    feedHealthy();
    expect(spoken).toEqual([]);
    fireEvent.click(screen.getByRole("button", { name: "Speech off" }));
    expect(spoken).toEqual([]); // nothing already on screen is read out
    emit(AppConfig.NAV_STATUS_TOPIC, goalStatus(1, 2));
    emit(AppConfig.NAV_STATUS_TOPIC, goalStatus(1, 4));
    emit(AppConfig.NAV_STATUS_TOPIC, goalStatus(1, 4));
    expect(spoken).toEqual([
      "executing. Driving to the goal",
      "ready. No blocking faults in current reported data",
      "Navigation goal: succeeded.",
      "Completed: Navigation goal",
    ]);
  });
});
