import React from "react";
import { act, render, screen } from "@testing-library/react";

import LifecycleStatus from "./LifecycleStatus";
import { useRos } from "../app/App";
import { LIFECYCLE_NODES } from "../shared/constants";

jest.mock("../app/App", () => ({
  useRos: jest.fn(),
}));

let services;

beforeEach(() => {
  jest.useFakeTimers();
  services = [];
  useRos.mockReturnValue({});
  window.ROSLIB = {
    Service: class {
      constructor(opts) {
        this.opts = opts;
        this.callService = jest.fn((req, ok) =>
          ok({ current_state: { label: "active" } }),
        );
        services.push(this);
      }
    },
    ServiceRequest: class {
      constructor(values) {
        Object.assign(this, values);
      }
    },
  };
});

afterEach(() => {
  delete window.ROSLIB;
  jest.useRealTimers();
  jest.clearAllMocks();
});

// The platform lifecycle manager owns transitions. This panel must only read
// state (GetState) and offer no control.
describe.each([false, true])("LifecycleStatus (compact=%s)", (compact) => {
  it("creates only GetState clients, never ChangeState", () => {
    render(<LifecycleStatus compact={compact} />);
    act(() => {
      jest.advanceTimersByTime(3100);
    });
    expect(services.length).toBe(LIFECYCLE_NODES.length);
    services.forEach((s) => {
      expect(s.opts.serviceType).toBe("lifecycle_msgs/srv/GetState");
      expect(s.opts.name).toMatch(/\/get_state$/);
    });
    expect(services.some((s) => /change_state/i.test(s.opts.name))).toBe(false);
  });

  it("shows no lifecycle control buttons and says it is read only", () => {
    render(<LifecycleStatus compact={compact} />);
    ["Prepare", "Prep", "Start", "Pause", "Reset"].forEach((label) => {
      expect(screen.queryByRole("button", { name: label })).toBeNull();
    });
    // The only button left is the legend toggle.
    expect(screen.getAllByRole("button").map((b) => b.getAttribute("aria-label"))).toEqual([
      "What do these states mean?",
    ]);
    expect(screen.getByText(/Read only/)).toBeTruthy();
  });

  it("legend no longer tells the operator to call Configure or Activate", () => {
    render(<LifecycleStatus compact={compact} />);
    act(() => {
      screen.getByRole("button", { name: "What do these states mean?" }).click();
    });
    expect(screen.queryByText(/call (Activate|Configure)/)).toBeNull();
  });
});
