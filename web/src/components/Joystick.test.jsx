import React from "react";
import { act, render, screen } from "@testing-library/react";

import JoystickComponent from "./Joystick";
import { AppConfig } from "../shared/constants";
import { resetMotionGate, setLockState } from "../shared/motion/motionGate";
import { attachRos, releaseAll } from "../shared/motion/uiCommand";
import { installMockRoslib, uninstallMockRoslib } from "../shared/motion/testing/mockRoslib";
import { openMotionGate, unsyncClock } from "../shared/motion/testing/gate";

let joystickProps;
jest.mock("react-joystick-component", () => ({
  Joystick: (props) => {
    joystickProps = props;
    return <div data-testid="stick" data-disabled={String(Boolean(props.disabled))} />;
  },
}));

jest.mock("../app/App", () => ({
  useRuntimeConfig: () => ({ config: { maxLinearSpeed: 0.2, maxAngularSpeed: 2 } }),
}));

let mock;
let cmdVel;
let pads;

beforeEach(() => {
  jest.useFakeTimers();
  mock = installMockRoslib();
  resetMotionGate();
  attachRos({});
  [cmdVel] = mock.topicsNamed(AppConfig.UI_CMD_VEL_TOPIC);
  pads = [null];
  navigator.getGamepads = () => pads;
});

afterEach(() => {
  releaseAll();
  attachRos(null);
  resetMotionGate();
  jest.useRealTimers();
  uninstallMockRoslib();
  delete navigator.getGamepads;
});

const otherVelocityTopics = () =>
  mock.registry.topics.filter((t) => /Twist/.test(t.messageType) && t !== cmdVel);

describe("on-screen joystick", () => {
  it("publishes only on /ui/cmd_vel at 20 Hz while held and one zero on release", () => {
    openMotionGate();
    render(<JoystickComponent />);
    act(() => joystickProps.move({ x: 0, y: 0.5 }));
    act(() => jest.advanceTimersByTime(500));
    expect(cmdVel.published).toHaveLength(11);
    expect(cmdVel.published[0].twist.linear.x).toBeCloseTo(0.1);
    act(() => joystickProps.stop());
    act(() => jest.advanceTimersByTime(500));
    expect(cmdVel.published).toHaveLength(12);
    expect(cmdVel.published[11].twist.linear.x).toBe(0);
    expect(otherVelocityTopics()).toEqual([]);
  });

  it("is disabled, says why, and publishes nothing while the clock is not synced", () => {
    openMotionGate();
    unsyncClock();
    render(<JoystickComponent />);
    expect(screen.getByTestId("stick").dataset.disabled).toBe("true");
    expect(screen.getByTestId("manual-drive-disabled").textContent).toBe("Robot clock not synced");
    act(() => joystickProps.move({ x: 0, y: 1 }));
    act(() => joystickProps.stop());
    expect(cmdVel.published).toHaveLength(0);
  });

  it.each(["locked", "unknown"])("publishes nothing while the lock is %s", (lock) => {
    openMotionGate();
    setLockState(lock);
    render(<JoystickComponent />);
    expect(screen.getByTestId("stick").dataset.disabled).toBe("true");
    act(() => joystickProps.move({ x: 0.3, y: 1 }));
    act(() => jest.advanceTimersByTime(300));
    act(() => joystickProps.stop());
    expect(cmdVel.published).toHaveLength(0);
  });

  it("a drag refused while stopped does not drive after the gate reopens", () => {
    openMotionGate();
    setLockState("locked");
    render(<JoystickComponent />);
    act(() => joystickProps.move({ x: 0, y: 1 }));
    act(() => setLockState("released"));
    act(() => joystickProps.move({ x: 0, y: 1 }));
    expect(cmdVel.published).toHaveLength(0);
    act(() => joystickProps.stop());
    act(() => joystickProps.move({ x: 0, y: 1 }));
    expect(cmdVel.published.length).toBeGreaterThan(0);
  });
});

describe("gamepad", () => {
  const frame = () => act(() => jest.advanceTimersByTime(17));

  it("publishes on /ui/cmd_vel while the stick is held and one zero on release", () => {
    openMotionGate();
    render(<JoystickComponent />);
    pads = [{ axes: [0, -1] }];
    frame();
    act(() => jest.advanceTimersByTime(200));
    const held = cmdVel.published.length;
    expect(held).toBeGreaterThanOrEqual(4);
    expect(cmdVel.published[0].twist.linear.x).toBeCloseTo(0.2);
    pads = [{ axes: [0, 0] }];
    frame();
    const last = cmdVel.published[cmdVel.published.length - 1];
    expect(last.twist.linear.x).toBe(0);
    const afterRelease = cmdVel.published.length;
    act(() => jest.advanceTimersByTime(500));
    expect(cmdVel.published).toHaveLength(afterRelease);
    expect(otherVelocityTopics()).toEqual([]);
  });

  it("must return to neutral after a stop before it drives again", () => {
    openMotionGate();
    setLockState("locked");
    render(<JoystickComponent />);
    pads = [{ axes: [0, -1] }];
    frame();
    act(() => setLockState("released"));
    frame();
    frame();
    expect(cmdVel.published).toHaveLength(0);
    pads = [{ axes: [0, 0] }];
    frame();
    pads = [{ axes: [0, -1] }];
    frame();
    expect(cmdVel.published.length).toBeGreaterThan(0);
  });
});
