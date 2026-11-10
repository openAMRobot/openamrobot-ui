import React from "react";
import { act, fireEvent, render, screen } from "@testing-library/react";

import MotionStopBanner from "./MotionStopBanner";
import { AppConfig } from "../shared/constants";
import { resetMotionGate, setConnection, setLockState } from "../shared/motion/motionGate";
import { installMockRoslib, uninstallMockRoslib } from "../shared/motion/testing/mockRoslib";
import { openMotionGate, unsyncClock } from "../shared/motion/testing/gate";

jest.mock("../app/App", () => ({ useRos: () => ({}) }));
jest.mock("../shared/events/eventLog", () => ({ addEvent: jest.fn() }));

let mock;
beforeEach(() => {
  mock = installMockRoslib();
  mock.onService(AppConfig.STOP_LOCK_RELEASE_SERVICE, (req, ok) => ok({ success: true, message: "released" }));
  resetMotionGate();
});
afterEach(() => {
  resetMotionGate();
  uninstallMockRoslib();
});

const resumeButton = () => screen.getByRole("button", { name: "Resume" });

describe("Stopped (software) banner", () => {
  it("shows while locked, never says E-STOP, and Resume is enabled", () => {
    setConnection("connected");
    setLockState("locked");
    render(<MotionStopBanner />);
    expect(screen.getByText("Stopped (software)")).toBeTruthy();
    expect(document.body.textContent).not.toMatch(/E-STOP/);
    expect(document.body.textContent).not.toMatch(/safety stop/i);
    expect(resumeButton().disabled).toBe(false);
  });

  it("shows while the lock is unknown, and Resume is disabled with the reason", () => {
    setConnection("connected");
    setLockState("unknown");
    render(<MotionStopBanner />);
    expect(screen.getByText("Stopped (software)")).toBeTruthy();
    expect(resumeButton().disabled).toBe(true);
    expect(screen.getByText(/needs a known lock state/)).toBeTruthy();
  });

  it("shows while rosbridge is down, and Resume is disabled with the reason", () => {
    render(<MotionStopBanner />);
    expect(screen.getByText("Stopped (software)")).toBeTruthy();
    expect(resumeButton().disabled).toBe(true);
    expect(screen.getByText(/needs a rosbridge connection/)).toBeTruthy();
  });

  it("is hidden when released and synced, and says when the clock is not synced", () => {
    openMotionGate();
    const { rerender } = render(<MotionStopBanner />);
    expect(screen.queryByText("Stopped (software)")).toBeNull();
    act(() => unsyncClock());
    rerender(<MotionStopBanner />);
    expect(screen.getByText("Robot clock not synced")).toBeTruthy();
  });
});

describe("Resume", () => {
  it("calls release only after confirmation", async () => {
    setConnection("connected");
    setLockState("locked");
    render(<MotionStopBanner />);
    fireEvent.click(resumeButton());
    expect(screen.getByRole("dialog")).toBeTruthy();
    expect(mock.callsTo(AppConfig.STOP_LOCK_RELEASE_SERVICE)).toHaveLength(0);
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "Confirm resume" }));
    });
    expect(mock.callsTo(AppConfig.STOP_LOCK_RELEASE_SERVICE)).toHaveLength(1);
    expect(mock.callsTo(AppConfig.STOP_LOCK_ENGAGE_SERVICE)).toHaveLength(0);
  });

  it("does not release when the confirmation is cancelled", () => {
    setConnection("connected");
    setLockState("locked");
    render(<MotionStopBanner />);
    fireEvent.click(resumeButton());
    fireEvent.click(screen.getByRole("button", { name: "Cancel" }));
    expect(screen.queryByRole("dialog")).toBeNull();
    expect(mock.callsTo(AppConfig.STOP_LOCK_RELEASE_SERVICE)).toHaveLength(0);
  });
});
