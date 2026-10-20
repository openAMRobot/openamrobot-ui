import React from "react";
import { fireEvent, render, screen } from "@testing-library/react";

import StatusBar from "./StatusBar";
import { useRos, useRosStatus } from "../app/App";
import { addEvent } from "../shared/events/eventLog";

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

  it("publishes a zero Twist, cancels the Nav2 goal and logs a software stop event", () => {
    render(<StatusBar />);

    const cmdVel = topics.find((topic) => topic.opts.messageType === "geometry_msgs/Twist");
    expect(cmdVel).toBeTruthy();

    fireEvent.click(screen.getByRole("button", { name: /stop \(software\)/i }));

    expect(cmdVel.publish).toHaveBeenCalledTimes(1);
    const message = cmdVel.publish.mock.calls[0][0];
    expect(message.linear).toEqual({ x: 0, y: 0, z: 0 });
    expect(message.angular).toEqual({ x: 0, y: 0, z: 0 });

    expect(services).toHaveLength(1);
    expect(services[0].callService).toHaveBeenCalledTimes(1);

    expect(addEvent).toHaveBeenCalledWith(
      expect.objectContaining({
        type: "safety",
        message: expect.stringContaining("Software stop"),
      }),
    );
  });
});
