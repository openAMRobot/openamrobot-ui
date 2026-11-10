import { createRobotActionClient, executeRobotPlan } from "./robotActions";
import { AppConfig } from "../../shared/constants";
import { resetMotionGate, setLockState } from "../../shared/motion/motionGate";
import { attachRos, releaseAll } from "../../shared/motion/uiCommand";
import { installMockRoslib, uninstallMockRoslib } from "../../shared/motion/testing/mockRoslib";
import { openMotionGate, unsyncClock } from "../../shared/motion/testing/gate";

let mock;
let cmdVel;
const ros = {};

beforeEach(() => {
  jest.useFakeTimers();
  mock = installMockRoslib();
  resetMotionGate();
  attachRos(ros);
  [cmdVel] = mock.topicsNamed(AppConfig.UI_CMD_VEL_TOPIC);
});

afterEach(() => {
  releaseAll();
  attachRos(null);
  resetMotionGate();
  jest.useRealTimers();
  uninstallMockRoslib();
});

const velocityTopicsOtherThanUi = () =>
  mock.registry.topics.filter((t) => /Twist/.test(t.messageType) && t.name !== AppConfig.UI_CMD_VEL_TOPIC);

// Jest 27 (react-scripts 5) has no advanceTimersByTimeAsync: step the fake
// timers and let pending promises settle between steps.
const advance = async (ms) => {
  for (let t = 0; t < ms; t += 10) {
    jest.advanceTimersByTime(10);
    // eslint-disable-next-line no-await-in-loop
    await Promise.resolve();
  }
};

describe("Blockly drive, rotate and stop blocks", () => {
  it("drive_for publishes on /ui/cmd_vel at 20 Hz for its duration, then one zero", async () => {
    openMotionGate();
    const client = createRobotActionClient(ros);
    const done = client.runAction({ type: "drive_for", linear: 0.1, seconds: 1 });
    await advance(1000);
    await done;
    const linear = cmdVel.published.map((m) => m.twist.linear.x);
    expect(linear.filter((x) => x === 0.1).length).toBeGreaterThanOrEqual(20);
    expect(linear[linear.length - 1]).toBe(0);
    const count = cmdVel.published.length;
    await advance(500);
    expect(cmdVel.published).toHaveLength(count);
    expect(velocityTopicsOtherThanUi()).toEqual([]);
  });

  it("rotate_for publishes angular velocity on /ui/cmd_vel, then one zero", async () => {
    openMotionGate();
    const client = createRobotActionClient(ros);
    const done = client.runAction({ type: "rotate_for", angular: 0.5, seconds: 0.5 });
    await advance(500);
    await done;
    expect(cmdVel.published[0].twist.angular.z).toBe(0.5);
    expect(cmdVel.published[cmdVel.published.length - 1].twist.angular.z).toBe(0);
  });

  it("set_speed holds until stop_movement", async () => {
    openMotionGate();
    const client = createRobotActionClient(ros);
    await client.runAction({ type: "set_speed", linear: 0.05, angular: 0 });
    await advance(500);
    expect(cmdVel.published.length).toBeGreaterThanOrEqual(10);
    await client.runAction({ type: "stop_movement" });
    const count = cmdVel.published.length;
    expect(cmdVel.published[count - 1].twist.linear.x).toBe(0);
    await advance(500);
    expect(cmdVel.published).toHaveLength(count);
  });

  it("the stop block sends one zero and cancels navigation, without the stop lock", async () => {
    openMotionGate();
    const client = createRobotActionClient(ros);
    await client.runAction({ type: "set_speed", linear: 0.05, angular: 0 });
    await client.runAction({ type: "stop" });
    expect(cmdVel.published[cmdVel.published.length - 1].twist.linear.x).toBe(0);
    expect(mock.callsTo(AppConfig.NAV_CANCEL_GOAL_SERVICE)).toHaveLength(1);
    expect(mock.callsTo(AppConfig.STOP_LOCK_ENGAGE_SERVICE)).toHaveLength(0);
  });

  it.each([
    ["locked", () => setLockState("locked")],
    ["unknown", () => setLockState("unknown")],
    ["clock not synced", unsyncClock],
  ])("a drive block fails and publishes nothing while %s", async (_label, close) => {
    openMotionGate();
    close();
    const client = createRobotActionClient(ros);
    await expect(client.runAction({ type: "drive_for", linear: 0.1, seconds: 1 })).rejects.toThrow(
      /Manual drive is disabled/,
    );
    expect(cmdVel.published).toHaveLength(0);
  });
});

describe("programs and voice-planned drive actions", () => {
  // Voice plans are loaded into Blockly and run through executeRobotPlan, so
  // a voice-planned drive is this same plan shape.
  it("a program never leaves a held command after it ends", async () => {
    openMotionGate();
    const done = executeRobotPlan(ros, [
      { type: "set_speed", linear: 0.1, angular: 0 },
      { type: "wait", seconds: 0.2 },
    ]);
    await advance(300);
    await done;
    const count = cmdVel.published.length;
    expect(cmdVel.published[count - 1].twist.linear.x).toBe(0);
    await advance(500);
    expect(cmdVel.published).toHaveLength(count);
    expect(velocityTopicsOtherThanUi()).toEqual([]);
  });

  it("a voice-planned drive publishes only on /ui/cmd_vel", async () => {
    openMotionGate();
    const voicePlan = [
      { type: "drive_for", linear: 0.1, seconds: 0.2 },
      { type: "rotate_for", angular: 0.3, seconds: 0.2 },
    ];
    const done = executeRobotPlan(ros, voicePlan);
    await advance(500);
    await done;
    expect(cmdVel.published.length).toBeGreaterThan(4);
    expect(velocityTopicsOtherThanUi()).toEqual([]);
  });
});
