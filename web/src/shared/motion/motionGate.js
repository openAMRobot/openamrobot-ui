import { useSyncExternalStore } from "react";

import { NOT_SYNCED_LABEL } from "./robotClock";
import { LOCK_RELEASED, LOCK_UNKNOWN } from "./stopLock";

/**
 * One store for whether manual drive may publish: rosbridge connected, the
 * stop lock known and released, and the robot clock synced. MotionBridge.jsx
 * feeds it; uiCommand.js and every manual control read it.
 */

const initialClock = {
  mode: "unknown",
  synced: false,
  label: NOT_SYNCED_LABEL,
  detail: "Clock mode unknown: waiting for the relay's use_sim_time",
};

let state = {
  connection: "disconnected",
  lock: LOCK_UNKNOWN,
  clock: initialClock,
  // Local Stop (software) latch. Set synchronously when Stop is pressed,
  // before the engage service answers, so no held input or goal gets through
  // while the request is pending, fails, or the lock topic has not updated.
  // Cleared only by a confirmed Resume whose release call succeeded; a
  // released lock sample (fresh or stale) and a reconnect do not clear it.
  localStop: false,
};
let clockSource = null;
const listeners = new Set();

const deriveGate = (s) => {
  let reason = "";
  if (s.localStop) reason = "Stopped (software)";
  else if (s.connection !== "connected") reason = "rosbridge disconnected";
  else if (s.lock === LOCK_UNKNOWN) reason = "Stop lock state unknown";
  else if (s.lock !== LOCK_RELEASED) reason = "Stopped (software)";
  else if (!s.clock.synced) reason = NOT_SYNCED_LABEL;
  return { ...s, canDrive: reason === "", reason };
};

let gate = deriveGate(state);

const emit = (next) => {
  state = next;
  gate = deriveGate(state);
  listeners.forEach((fn) => fn(gate));
};

export const getMotionGate = () => gate;

export const subscribeMotionGate = (fn) => {
  listeners.add(fn);
  return () => listeners.delete(fn);
};

export const setConnection = (connection) => {
  if (connection !== state.connection) emit({ ...state, connection });
};

export const setLockState = (lock) => {
  if (lock !== state.lock) emit({ ...state, lock });
};

export const setClockStatus = (clock) => emit({ ...state, clock });

/** Stop (software) pressed: latch locally at once. */
export const engageLocalStop = () => {
  if (!state.localStop) emit({ ...state, localStop: true });
};

/** Only the confirmed Resume path (after a successful release) calls this. */
export const clearLocalStop = () => {
  if (state.localStop) emit({ ...state, localStop: false });
};

/** The live robot clock that stamps commands (null when disconnected). */
export const setClockSource = (source) => {
  clockSource = source;
};

export const getCommandStamp = () => clockSource?.getStamp() ?? null;

/**
 * Back to the start-up state: what every new connection begins with. The
 * local Stop latch is kept: a reconnect is not a Resume.
 */
export const resetMotionGate = (connection = "disconnected") => {
  clockSource = null;
  emit({ connection, lock: LOCK_UNKNOWN, clock: initialClock, localStop: state.localStop });
};

export const useMotionGate = () =>
  useSyncExternalStore(subscribeMotionGate, getMotionGate, getMotionGate);
