import React from "react";
import { act, render, screen, within } from "@testing-library/react";
import "@testing-library/jest-dom";

import RobotDescriptionPage from "./RobotDescriptionPage";
import { useRos, useRosStatus } from "../app/App";
import useRobotScene from "../features/robotDescription/three/useRobotScene";
import { fetchRobotDescriptionManifest } from "../features/robotDescription/api/robotDescriptionApi";
import { COMBINED_URDF, PLATFORM_ONLY_URDF } from "../features/upperBody/__fixtures__/combinedUrdf";

jest.mock("../app/App", () => ({
  useRos: jest.fn(),
  useRosStatus: jest.fn(),
}));

// three.js needs WebGL, which jsdom does not have; the scene is not under test.
jest.mock("../features/robotDescription/three/useRobotScene", () => jest.fn());

jest.mock("../features/robotDescription/api/robotDescriptionApi", () => ({
  fetchRobotDescriptionManifest: jest.fn(),
  resolveRobotDescriptionUrl: (p) => p,
}));

let topics;
let constructed;

const topic = (name) => [...topics].reverse().find((t) => t.opts.name === name);
const deliver = (name, msg) =>
  act(() => {
    topic(name).subscribe.mock.calls.forEach(([cb]) => cb(msg));
  });

beforeEach(() => {
  jest.useFakeTimers();
  topics = [];
  constructed = { Service: 0, ActionClient: 0, Action: 0, Param: 0 };
  localStorage.clear();

  useRos.mockReturnValue({});
  useRosStatus.mockReturnValue("connected");
  useRobotScene.mockReturnValue({
    containerRef: { current: null },
    status: { stage: "ready", error: null, meshErrors: [], loaded: 0, total: 0 },
    robot: null,
    resetView: jest.fn(),
    fitToScreen: jest.fn(),
    toggleFullscreen: jest.fn(),
    isFullscreen: false,
    setJointValues: jest.fn(),
  });
  fetchRobotDescriptionManifest.mockResolvedValue({
    displayName: "OpenAMRobot",
    available: true,
    urdfUrl: "/api/robot-description/urdf",
    packages: { openamrobot_description: "/api/robot-description/assets" },
  });

  const counting = (kind) =>
    class {
      constructor(opts) {
        constructed[kind] += 1;
        this.opts = opts;
      }
    };
  window.ROSLIB = {
    Topic: class {
      constructor(opts) {
        this.opts = opts;
        this.subscribe = jest.fn();
        this.unsubscribe = jest.fn();
        this.publish = jest.fn();
        this.advertise = jest.fn();
        this.unadvertise = jest.fn();
        topics.push(this);
      }
    },
    Service: counting("Service"),
    ActionClient: counting("ActionClient"),
    Action: counting("Action"),
    Param: counting("Param"),
  };
});

afterEach(() => {
  jest.useRealTimers();
  delete window.ROSLIB;
});

const renderPage = async () => {
  render(<RobotDescriptionPage />);
  await act(async () => {});
};

it("loads the combined model the robot publishes and labels it a candidate", async () => {
  await renderPage();
  expect(screen.queryByText("Candidate model, provisional geometry")).toBeNull();

  deliver("/robot_description", { data: COMBINED_URDF });

  expect(screen.getByText("Candidate model, provisional geometry")).toBeInTheDocument();
  expect(screen.getByText("openAMRobot/openamr-upperbody-sw")).toBeInTheDocument();
  expect(screen.getByText("462907556cbb9c770eb814dda436cc3abca29741")).toBeInTheDocument();
  expect(screen.getByText(/Placeholder positions, pending M-09/)).toBeInTheDocument();
  const lastManifest = useRobotScene.mock.calls[useRobotScene.mock.calls.length - 1][0].manifest;
  expect(lastManifest.urdfText).toBe(COMBINED_URDF);
});

it("keeps the backend model when the published description is not the combined model", async () => {
  await renderPage();
  deliver("/robot_description", { data: PLATFORM_ONLY_URDF });
  expect(screen.queryByText("Candidate model, provisional geometry")).toBeNull();
  const lastManifest = useRobotScene.mock.calls[useRobotScene.mock.calls.length - 1][0].manifest;
  expect(lastManifest.urdfText).toBeUndefined();
});

it("shows configuration identity as unknown when the robot reports none", async () => {
  await renderPage();
  deliver("/robot_description", { data: COMBINED_URDF });
  expect(within(screen.getByTestId("configuration-identity")).getByText("model or configuration unknown")).toBeInTheDocument();
});

it("shows reported arm and lift positions with a per-group freshness from the publish rate", async () => {
  await renderPage();
  deliver("/robot_description", { data: COMBINED_URDF });

  const send = (names, positions) => deliver("/joint_states", { name: names, position: positions });
  // lift and left arm at 10 Hz; right arm never reported
  for (let i = 0; i < 4; i += 1) {
    send(["lift_joint", "openarm_left_joint1"], [0.175, 0.5]);
    act(() => {
      jest.advanceTimersByTime(100);
    });
  }

  const lift = screen.getByTestId("joint-group-lift");
  expect(within(lift).getByText("0.175")).toBeInTheDocument();
  expect(within(lift).getByText(/Updating, measured period 100 ms/)).toBeInTheDocument();
  expect(within(screen.getByTestId("joint-group-rightArm")).getByText("Not reported")).toBeInTheDocument();

  expect(screen.getByTestId("shoulder-height")).toHaveTextContent("1175 mm");
  expect(screen.getByTestId("nearest-stop")).toHaveTextContent("at L1175");
  expect(screen.getByTestId("stop-sensors")).toHaveTextContent("Stop sensors: not reported");

  act(() => {
    jest.advanceTimersByTime(1000);
  });
  expect(within(screen.getByTestId("joint-group-lift")).getByText(/Stale: no update for/)).toBeInTheDocument();
});

const LEFT_ARM = [1, 2, 3, 4, 5, 6, 7].map((i) => `openarm_left_joint${i}`);

const sendAt10Hz = (names, positions, count = 4) => {
  for (let i = 0; i < count; i += 1) {
    deliver("/joint_states", { name: names, position: positions });
    act(() => {
      jest.advanceTimersByTime(100);
    });
  }
};

it("does not show an arm as updating from a partial JointState", async () => {
  await renderPage();
  sendAt10Hz(["openarm_left_joint1"], [0.1]);
  const arm = screen.getByTestId("joint-group-leftArm");
  expect(within(arm).queryByText(/^Updating/)).toBeNull();
  expect(
    within(arm).getByText(
      "Partial (not reported: openarm_left_joint2, openarm_left_joint3, openarm_left_joint4, openarm_left_joint5, openarm_left_joint6, openarm_left_joint7)",
    ),
  ).toBeInTheDocument();
});

it("names a permanently missing arm joint and keeps the arm partial", async () => {
  await renderPage();
  const six = LEFT_ARM.slice(0, 6);
  sendAt10Hz(six, six.map(() => 0.2), 50);
  const arm = screen.getByTestId("joint-group-leftArm");
  expect(within(arm).getByText("Partial (not reported: openarm_left_joint7)")).toBeInTheDocument();
  expect(within(arm).getAllByText("0.200")).toHaveLength(6);
});

it("rejects NaN and Infinity as positions", async () => {
  await renderPage();
  sendAt10Hz(LEFT_ARM, LEFT_ARM.map(() => 0.3));
  expect(within(screen.getByTestId("joint-group-leftArm")).getByText(/^Updating/)).toBeInTheDocument();

  deliver("/joint_states", { name: LEFT_ARM, position: [NaN, Infinity, -Infinity, 0.3, 0.3, 0.3, 0.3] });
  act(() => {
    jest.advanceTimersByTime(250);
  });
  const arm = screen.getByTestId("joint-group-leftArm");
  expect(within(arm).getAllByText("invalid")).toHaveLength(3);
  expect(within(arm).queryByText(/NaN|Infinity/)).toBeNull();
  expect(
    within(arm).getByText("Partial (invalid: openarm_left_joint1, openarm_left_joint2, openarm_left_joint3)"),
  ).toBeInTheDocument();

  // a lift that only ever reports NaN has no height and is never updating
  deliver("/robot_description", { data: COMBINED_URDF });
  sendAt10Hz(["lift_joint"], [NaN]);
  expect(within(screen.getByTestId("lift-panel")).getByText("not reported")).toBeInTheDocument();
  expect(screen.getByTestId("shoulder-height")).toHaveTextContent("unknown");
  expect(within(screen.getByTestId("joint-group-lift")).getByText(/invalid: lift_joint/)).toBeInTheDocument();
});

it("clears positions and sampling history on a reconnect to a different source", async () => {
  const rosA = { id: "A" };
  const rosB = { id: "B" };
  useRos.mockReturnValue(rosA);
  const { rerender } = render(<RobotDescriptionPage />);
  await act(async () => {});
  deliver("/robot_description", { data: COMBINED_URDF });
  sendAt10Hz(["lift_joint"], [0.175]);
  expect(screen.getByTestId("shoulder-height")).toHaveTextContent("1175 mm");
  const oldJointTopic = topics.find((t) => t.opts.name === "/joint_states" && t.opts.ros === rosA);

  useRos.mockReturnValue(rosB);
  rerender(<RobotDescriptionPage />);
  await act(async () => {});

  expect(oldJointTopic.unsubscribe).toHaveBeenCalled();
  const lift = screen.getByTestId("joint-group-lift");
  expect(within(lift).getByText("Not reported")).toBeInTheDocument();
  expect(within(lift).queryByText("0.175")).toBeNull();

  // a late message from the old connection is ignored
  act(() => {
    oldJointTopic.subscribe.mock.calls.forEach(([cb]) => cb({ name: ["lift_joint"], position: [0.35] }));
  });
  expect(within(screen.getByTestId("joint-group-lift")).getByText("Not reported")).toBeInTheDocument();

  // the new source starts its own rate measurement from nothing
  const newJointTopic = topics.find((t) => t.opts.name === "/joint_states" && t.opts.ros === rosB);
  act(() => {
    newJointTopic.subscribe.mock.calls.forEach(([cb]) => cb({ name: ["lift_joint"], position: [0] }));
  });
  act(() => {
    jest.advanceTimersByTime(250);
  });
  expect(within(screen.getByTestId("joint-group-lift")).getByText("Measuring publish rate")).toBeInTheDocument();
  expect(within(screen.getByTestId("joint-group-lift")).getByText("0.000")).toBeInTheDocument();
});

it("says Stop (software) is base only and offers no controls in the upper-body view", async () => {
  await renderPage();
  deliver("/robot_description", { data: COMBINED_URDF });
  for (const id of ["upper-body-panel", "lift-panel"]) {
    const panel = screen.getByTestId(id);
    expect(within(panel).queryAllByRole("button")).toHaveLength(0);
    expect(within(panel).queryAllByRole("slider")).toHaveLength(0);
    expect(panel.querySelectorAll("input, select, button")).toHaveLength(0);
  }
  expect(
    screen.getByText("Stop (software) locks base motion only. It does not stop the arms or the lift."),
  ).toBeInTheDocument();
  expect(screen.queryByText(/ready to move|safe to move|controllers? (ok|healthy)/i)).toBeNull();
});

it.each(["description", "live"])(
  "never advertises, publishes or calls anything (%s mode)",
  async (mode) => {
    localStorage.setItem("robotDescriptionMode", mode);
    await renderPage();
    deliver("/robot_description", { data: COMBINED_URDF });
    deliver("/joint_states", { name: ["lift_joint", "openarm_left_joint1"], position: [0, 0] });
    act(() => {
      jest.advanceTimersByTime(1000);
    });

    expect(topics.map((t) => t.opts.name)).toEqual(expect.arrayContaining(["/robot_description", "/joint_states"]));
    topics.forEach((t) => {
      expect(t.publish).not.toHaveBeenCalled();
      expect(t.advertise).not.toHaveBeenCalled();
    });
    expect(constructed).toEqual({ Service: 0, ActionClient: 0, Action: 0, Param: 0 });
  },
);
