import { AppConfig } from "../constants";
import {
  resetMotionGate,
  setClockSource,
  setClockStatus,
  setConnection,
  setLockState,
} from "./motionGate";
import { attachRos, clampTwist, holdVelocity, releaseAll, releaseVelocity } from "./uiCommand";
import { installMockRoslib, uninstallMockRoslib } from "./testing/mockRoslib";

const STAMP = { sec: 1234, nanosec: 5000 };

const openGate = () => {
  setConnection("connected");
  setLockState("released");
  setClockSource({ getStamp: () => STAMP });
  setClockStatus({ mode: "sim", synced: true, label: "Robot clock synced", detail: "" });
};

let mock;
let cmdVel;

beforeEach(() => {
  jest.useFakeTimers();
  mock = installMockRoslib();
  resetMotionGate();
  attachRos({ id: Math.random() });
  [cmdVel] = mock.topicsNamed(AppConfig.UI_CMD_VEL_TOPIC);
});

afterEach(() => {
  releaseAll();
  attachRos(null);
  resetMotionGate();
  jest.useRealTimers();
  uninstallMockRoslib();
});

describe("uiCommand publisher", () => {
  it("is one TwistStamped publisher on /ui/cmd_vel", () => {
    expect(AppConfig.UI_CMD_VEL_TOPIC).toBe("/ui/cmd_vel");
    expect(cmdVel.messageType).toBe("geometry_msgs/TwistStamped");
    const velocityTopics = mock.registry.topics.filter((t) => /Twist/.test(t.messageType));
    expect(velocityTopics).toEqual([cmdVel]);
  });

  it("publishes at 20 Hz while held, then exactly one zero on release", () => {
    openGate();
    expect(holdVelocity("joystick", { linear: 0.1, angular: 0.2 }).ok).toBe(true);
    expect(cmdVel.published).toHaveLength(1);
    jest.advanceTimersByTime(1000);
    expect(cmdVel.published).toHaveLength(21);
    releaseVelocity("joystick");
    expect(cmdVel.published).toHaveLength(22);
    const zero = cmdVel.published[21];
    expect(zero.twist.linear).toEqual({ x: 0, y: 0, z: 0 });
    expect(zero.twist.angular).toEqual({ x: 0, y: 0, z: 0 });
    jest.advanceTimersByTime(1000);
    expect(cmdVel.published).toHaveLength(22);
  });

  it("stamps every message with the robot clock stamp", () => {
    openGate();
    holdVelocity("joystick", { linear: 0.1, angular: 0 });
    const message = cmdVel.published[0];
    expect(message.header.stamp).toEqual(STAMP);
    expect(message.twist.linear.x).toBeCloseTo(0.1);
  });

  it("updates the held command without restarting the 20 Hz timer", () => {
    openGate();
    holdVelocity("joystick", { linear: 0.1, angular: 0 });
    holdVelocity("joystick", { linear: 0.05, angular: 0 });
    jest.advanceTimersByTime(50);
    expect(cmdVel.published).toHaveLength(2);
    expect(cmdVel.published[1].twist.linear.x).toBeCloseTo(0.05);
  });

  it("clamps to MAX_LINEAR_SPEED and MAX_ANGULAR_SPEED", () => {
    expect(clampTwist({ linear: 5, angular: -50 })).toEqual({
      linear: { x: AppConfig.MAX_LINEAR_SPEED, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: -AppConfig.MAX_ANGULAR_SPEED },
    });
    expect(clampTwist({ linear: NaN, angular: undefined }).linear.x).toBe(0);
  });

  it.each([
    ["locked", () => setLockState("locked")],
    ["unknown", () => setLockState("unknown")],
    [
      "clock not synced",
      () => setClockStatus({ mode: "real", synced: false, label: "Robot clock not synced", detail: "x" }),
    ],
    ["disconnected", () => setConnection("disconnected")],
  ])("publishes nothing while %s", (_label, closeGate) => {
    openGate();
    closeGate();
    const result = holdVelocity("joystick", { linear: 0.1, angular: 0 });
    expect(result.ok).toBe(false);
    jest.advanceTimersByTime(500);
    releaseVelocity("joystick");
    expect(cmdVel.published).toHaveLength(0);
  });

  it("drops a hold the moment the gate closes, with no zero and no further samples", () => {
    openGate();
    holdVelocity("joystick", { linear: 0.1, angular: 0 });
    jest.advanceTimersByTime(100);
    const before = cmdVel.published.length;
    setLockState("locked");
    jest.advanceTimersByTime(500);
    releaseVelocity("joystick");
    expect(cmdVel.published).toHaveLength(before);
  });

  it("publishes nothing when the clock source has no stamp", () => {
    openGate();
    setClockSource({ getStamp: () => null });
    expect(holdVelocity("joystick", { linear: 0.1 }).ok).toBe(false);
    expect(cmdVel.published).toHaveLength(0);
  });

  it("only the source that owns the output can release it", () => {
    openGate();
    holdVelocity("gamepad", { linear: 0.1 });
    releaseVelocity("joystick");
    jest.advanceTimersByTime(50);
    expect(cmdVel.published).toHaveLength(2);
    expect(cmdVel.published[1].twist.linear.x).toBeCloseTo(0.1);
  });
});
