import React from "react";
import { act, fireEvent, render, screen } from "@testing-library/react";
import { MemoryRouter, Route, Routes } from "react-router-dom";

import ReportingPage, { EXIT_HOLD_MS, MAX_WARNINGS_SHOWN, STATE_FONT_SIZE } from "./ReportingPage";
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

// TABLET-MOUNT (decision of 2 Nov 2026, review of #54): the ZX80 is portrait
// only on a fixed mount, so the page is 1200 wide and 1920 tall. JSDOM has
// no layout, so these check the structure that keeps the page inside a
// portrait screen; the overflow itself was checked in Chromium at 1200 x 1920
// (see the PR description).
describe("Reporting Mode portrait layout", () => {
  it("uses one column, a two-column source footer and a width-bounded state word", () => {
    renderPage();
    feedHealthy();
    const main = screen.getByTestId("reporting-main");
    expect(main.className).toMatch(/\bflex-col\b/);
    expect(main.className).not.toMatch(/grid-cols-/);
    const footer = screen.getByTestId("reporting-footer");
    expect(footer.className).toMatch(/\bgrid-cols-2\b/);
    expect(footer.className).not.toMatch(/grid-cols-3/);
    expect(STATE_FONT_SIZE).toBe("min(120px, calc((100vw - 80px) / 10))");
  });

  it("keeps the exit control in the header row, not over the header text", () => {
    renderPage();
    feedHealthy();
    const exit = screen.getByRole("button", { name: "Hold to exit Reporting Mode" });
    expect(screen.getByTestId("reporting-header").contains(exit)).toBe(true);
    expect(exit.className).not.toMatch(/\babsolute\b/);
  });

  it("lets long names wrap and counts warnings it does not list", () => {
    renderPage();
    feedHealthy();
    const many = Array.from({ length: MAX_WARNINGS_SHOWN + 3 }, (_, i) => [1, `source_${i}_with_a_long_name`, "warn"]);
    emit(AppConfig.DIAGNOSTICS_TOPIC, {
      header: { stamp: { sec: 100, nanosec: 0 } },
      status: many.map(([level, name, message]) => ({ level, name, message, hardware_id: "", values: [] })),
    });
    const warnings = screen.getByTestId("reporting-warnings");
    expect(warnings.querySelectorAll("li")).toHaveLength(MAX_WARNINGS_SHOWN + 1);
    expect(screen.getByTestId("reporting-warnings-more").textContent).toBe("3 more not shown");
    expect(warnings.closest("section").getAttribute("style")).toContain("overflow-wrap: anywhere");
  });
});
