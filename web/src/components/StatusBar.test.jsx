import React from "react";
import { fireEvent, render, screen } from "@testing-library/react";

import StatusBar from "./StatusBar";
import { useRos, useRosStatus } from "../app/App";
import { addEvent } from "../shared/events/eventLog";
import { ACTION_CANCEL_SERVICES, AppConfig } from "../shared/constants";
import { clearLocalStop } from "../shared/motion/motionGate";

jest.mock("../app/App", () => ({
  useRos: jest.fn(),
  useRosStatus: jest.fn(),
}));

jest.mock("../shared/events/eventLog", () => ({
  addEvent: jest.fn(),
}));

let topics;
let services;

beforeEach(() => {
  topics = [];
  services = [];

  useRos.mockReturnValue({});
  useRosStatus.mockReturnValue("connected");

  window.ROSLIB = {
    Topic: class {
      constructor(opts) {
        this.opts = opts;
        this.publish = jest.fn();
        this.subscribe = jest.fn();
        this.unsubscribe = jest.fn();
        topics.push(this);
      }
    },
    Service: class {
      constructor(opts) {
        this.opts = opts;
        this.callService = jest.fn();
        services.push(this);
      }
    },
    Message: class {
      constructor(values) {
        Object.assign(this, values);
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
  clearLocalStop();
  delete window.ROSLIB;
  jest.clearAllMocks();
});

describe("StatusBar software stop button", () => {
  it("is labelled Stop (software) and never E-STOP", () => {
    render(<StatusBar />);

    expect(screen.getByRole("button", { name: /stop \(software\)/i })).toBeTruthy();
    expect(screen.queryByText(/e-stop/i)).toBeNull();
  });

  it("says in its tooltip that it is not the hardwired E-stop", () => {
    render(<StatusBar />);

    const button = screen.getByRole("button", { name: /stop \(software\)/i });
    expect(button.getAttribute("title")).toMatch(/not the hardwired E-stop/i);
  });

  it("engages the stop lock, cancels every Nav2 and docking goal once and logs the event", () => {
    render(<StatusBar />);

    fireEvent.click(screen.getByRole("button", { name: /stop \(software\)/i }));

    const callsTo = (name) =>
      services.filter((s) => s.opts.name === name).flatMap((s) => s.callService.mock.calls);
    expect(callsTo(AppConfig.STOP_LOCK_ENGAGE_SERVICE)).toHaveLength(1);
    ACTION_CANCEL_SERVICES.forEach((name) => expect(callsTo(name)).toHaveLength(1));
    expect(callsTo(AppConfig.STOP_LOCK_RELEASE_SERVICE)).toHaveLength(0);

    expect(addEvent).toHaveBeenCalledWith(
      expect.objectContaining({
        type: "safety",
        message: expect.stringContaining("Stop (software)"),
      }),
    );
  });

  it("publishes no velocity: Stop (software) never touches cmd_vel", () => {
    render(<StatusBar />);
    fireEvent.click(screen.getByRole("button", { name: /stop \(software\)/i }));
    const velocity = topics.filter((topic) => /Twist/.test(topic.opts.messageType));
    expect(velocity).toEqual([]);
  });
});
