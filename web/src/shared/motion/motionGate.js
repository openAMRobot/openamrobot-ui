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
};
let clockSource = null;
const listeners = new Set();

const deriveGate = (s) => {
  let reason = "";
  if (s.connection !== "connected") reason = "rosbridge disconnected";
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

/** The live robot clock that stamps commands (null when disconnected). */
export const setClockSource = (source) => {
  clockSource = source;
};

export const getCommandStamp = () => clockSource?.getStamp() ?? null;

/** Back to the start-up state: what every new connection begins with. */
export const resetMotionGate = (connection = "disconnected") => {
  clockSource = null;
  emit({ connection, lock: LOCK_UNKNOWN, clock: initialClock });
};

export const useMotionGate = () =>
  useSyncExternalStore(subscribeMotionGate, getMotionGate, getMotionGate);
