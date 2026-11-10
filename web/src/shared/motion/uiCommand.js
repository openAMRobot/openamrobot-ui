import { AppConfig } from "../constants";
import { getCommandStamp, getMotionGate, subscribeMotionGate } from "./motionGate";
import { MANUAL_PUBLISH_PERIOD_MS } from "./timing";

/**
 * The only velocity publisher in the UI.
 *
 * Every manual source (joystick, gamepad, map page drive controls, Blockly
 * drive/rotate/stop blocks, voice-planned drive actions run through Blockly)
 * goes through holdVelocity/releaseVelocity here. It publishes
 * geometry_msgs/TwistStamped on AppConfig.UI_CMD_VEL_TOPIC only; the platform
 * relay forwards it to twist_mux. Navigation and docking never come through
 * here: they stay on their Nav2 and docking interfaces.
 *
 * While a source is held: one message at once, then every 50 ms (20 Hz).
 * On release: one zero, then nothing. Nothing at all is published while the
 * motion gate is closed (rosbridge down, stop lock locked or unknown, robot
 * clock not synced); a hold that loses the gate is dropped and must be
 * pressed again.
 *
 * Speed clamps use MAX_LINEAR_SPEED and MAX_ANGULAR_SPEED as usability limits
 * only. They are not a safety function.
 */

let ros = null;
let topic = null;
let active = null; // { source, twist }
let timer = null;

const finiteOr0 = (value) => (Number.isFinite(Number(value)) ? Number(value) : 0);
const clamp = (value, limit) => Math.max(-limit, Math.min(limit, finiteOr0(value)));

export const clampTwist = ({ linear = 0, angular = 0 } = {}) => ({
  linear: { x: clamp(linear, AppConfig.MAX_LINEAR_SPEED), y: 0, z: 0 },
  angular: { x: 0, y: 0, z: clamp(angular, AppConfig.MAX_ANGULAR_SPEED) },
});

const ZERO = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };

/** Bind to the shared ROSLIB.Ros instance. Called by MotionBridge.jsx. */
export const attachRos = (nextRos) => {
  if (nextRos === ros) return;
  stopTimer();
  active = null;
  ros = nextRos;
  topic =
    ros && window.ROSLIB
      ? new window.ROSLIB.Topic({
          ros,
          name: AppConfig.UI_CMD_VEL_TOPIC,
          messageType: "geometry_msgs/TwistStamped",
        })
      : null;
};

function stopTimer() {
  if (timer) {
    clearInterval(timer);
    timer = null;
  }
}

/** Publish one stamped message. Returns false when nothing was published. */
const publish = (twist) => {
  if (!topic || !getMotionGate().canDrive) return false;
  const stamp = getCommandStamp();
  if (!stamp) return false;
  topic.publish(
    new window.ROSLIB.Message({
      header: { stamp, frame_id: "" },
      twist,
    }),
  );
  return true;
};

const tick = () => {
  if (!active || !publish(active.twist)) {
    // Gate closed or stamp unavailable: drop the hold, publish nothing.
    active = null;
    stopTimer();
  }
};

/**
 * Start or update a held manual command for `source`. The most recent
 * caller owns the output. Returns { ok, reason }.
 */
export const holdVelocity = (source, command) => {
  const gate = getMotionGate();
  if (!gate.canDrive || !topic) {
    active = null;
    stopTimer();
    return { ok: false, reason: gate.reason || "rosbridge not available" };
  }
  const twist = clampTwist(command);
  const isNew = !active;
  active = { source, twist };
  if (isNew) {
    if (!publish(twist)) {
      active = null;
      return { ok: false, reason: getMotionGate().reason || "Robot clock not synced" };
    }
    timer = setInterval(tick, MANUAL_PUBLISH_PERIOD_MS);
  }
  return { ok: true, reason: "" };
};

/** Release `source`: one zero if it owned the output, then stop. */
export const releaseVelocity = (source) => {
  if (!active || active.source !== source) return;
  active = null;
  stopTimer();
  publish(ZERO);
};

/** Drop every hold. Sends one zero only if a hold was active and the gate is open. */
export const releaseAll = () => {
  if (!active) return;
  releaseVelocity(active.source);
};

export const activeSource = () => active?.source ?? null;

// Closing the gate ends any hold at once, without waiting for the next tick.
subscribeMotionGate((gate) => {
  if (!gate.canDrive && active) {
    active = null;
    stopTimer();
  }
});
