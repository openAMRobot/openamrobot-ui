import { getMotionGate } from "./motionGate";
import { LOCK_LOCKED, LOCK_RELEASED } from "./stopLock";

/**
 * Navigation goals while the software stop lock is engaged or unknown.
 *
 * The lock masks motion in the platform, but a Nav2 goal accepted while it
 * is engaged would still be active, and would drive the robot as soon as
 * Resume releases the lock. So no UI path sends a goal (map page, missions,
 * routes, programs, docking) unless the lock is known and released.
 *
 * Returns the reason goals are blocked, or "" when they may be sent.
 */
export const navGoalBlockedReason = (gate = getMotionGate()) => {
  // Stop (software) pressed here: blocked until Resume, whatever the lock
  // topic says while the engage request is pending or failed.
  if (gate.localStop) return "Stopped (software). Goals are disabled until Resume";
  if (gate.connection !== "connected") return "rosbridge disconnected";
  if (gate.lock === LOCK_RELEASED) return "";
  if (gate.lock === LOCK_LOCKED) return "Stopped (software). Goals are disabled until Resume";
  return "Stop lock state unknown. Goals are disabled";
};
