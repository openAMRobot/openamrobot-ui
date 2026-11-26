import React from "react";
import { fireEvent, render, screen } from "@testing-library/react";

import MapsPage from "./MapsPage";
import { useRos, useRosStatus } from "../app/App";

jest.mock("../app/App", () => ({
  useRos: jest.fn(),
  useRosStatus: jest.fn(),
}));

let topics;

beforeEach(() => {
  topics = [];
  useRos.mockReturnValue({});
  useRosStatus.mockReturnValue("connected");
  window.confirm = jest.fn(() => true);
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
    Message: class {
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

// openamrobot-ui#32: mapping belongs to the platform. The Maps page must not
// claim to launch mapping or stop navigation, and needs no confirmation.
describe("MapsPage mapping card", () => {
  it("says mapping runs on the platform", () => {
    render(<MapsPage />);
    expect(screen.queryByText(/Launches mapping mode/i)).toBeNull();
    expect(screen.getByText(/Mapping runs on the robot platform/i)).toBeTruthy();
  });

  it("sends build_map without a confirmation dialog", () => {
    render(<MapsPage />);
    fireEvent.click(screen.getByRole("button", { name: "How to map" }));
    expect(window.confirm).not.toHaveBeenCalled();
    const op = topics.find((t) => t.opts.name === "/ui_operation");
    expect(op.publish).toHaveBeenCalledWith(expect.objectContaining({ data: "build_map" }));
  });
});
