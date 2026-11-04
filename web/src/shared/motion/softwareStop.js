import { ACTION_CANCEL_SERVICES } from "../constants";
import { addEvent } from "../events/eventLog";
import { abortUiMotion } from "./abortBus";
import { clearLocalStop, engageLocalStop } from "./motionGate";
import { engageStopLock, releaseStopLock } from "./stopLock";
import { releaseAll } from "./uiCommand";

/**
 * Stop (software): a functional stop, not a safety function (E-01, issue
 * #34). The physical E-stops are hardware and are not touched here.
 *
 * 0. Latch a local inhibit at once (motionGate localStop), before any reply.
 * 1. Engage the platform stop lock (/safety/stop_lock_engage).
 * 2. Cancel every active goal of every Nav2 and docking action the UI starts.
 * 3. Abort UI-side programs and missions and drop any held manual command.
 * 4. Log the event.
 *
 * Safe to press repeatedly and while already locked: engage is idempotent on
 * the platform side and a cancel-all with nothing active is a no-op.
 */

const cancelAllGoals = (ros, name) => {
  if (!ros || !window.ROSLIB) return;
  const service = new window.ROSLIB.Service({
    ros,
    name,
    serviceType: "action_msgs/CancelGoal",
  });
  service.callService(
    new window.ROSLIB.ServiceRequest({
      goal_info: { goal_id: { uuid: new Array(16).fill(0) }, stamp: { sec: 0, nanosec: 0 } },
    }),
    () => {},
    () => {},
  );
};

export const softwareStop = (ros, origin) => {
  // Synchronous part first: end any held command (one zero) and latch the
  // local inhibit, so no held input, program step or goal gets through
  // while the engage request below is pending, fails, or the lock topic has
  // not updated yet. Only a confirmed Resume clears the latch.
  releaseAll();
  engageLocalStop();
  const engaged = engageStopLock(ros);
  ACTION_CANCEL_SERVICES.forEach((name) => cancelAllGoals(ros, name));
  abortUiMotion("Stop (software)");
  addEvent({
    type: "safety",
    severity: "error",
    message: `Stop (software) pressed (${origin})`,
  });
  return engaged.then((result) => {
    if (!result.success) {
      addEvent({
        type: "safety",
        severity: "error",
        message: `Stop lock engage did not confirm: ${result.message || "no response"}`,
      });
    }
    return result;
  });
};

/** Resume: the only call to /safety/stop_lock_release in the UI. */
export const resumeFromSoftwareStop = (ros) =>
  releaseStopLock(ros).then((result) => {
    // The local latch clears only when the platform accepted the release.
    if (result.success) clearLocalStop();
    addEvent({
      type: "safety",
      severity: result.success ? "info" : "error",
      message: result.success
        ? "Stop lock released (Resume)"
        : `Stop lock release failed: ${result.message || "no response"}`,
    });
    return result;
  });
