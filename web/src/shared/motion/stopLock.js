import { AppConfig } from "../constants";
import { monotonicNowMs } from "./robotClock";
import { GATE_EVALUATE_PERIOD_MS, STOP_LOCK_STALE_AFTER_MS } from "./timing";

/**
 * Stop lock state from /safety/stop_lock (std_msgs/Bool, true = engaged),
 * published by safety_stop_gate_node with RELIABLE, TRANSIENT_LOCAL, depth 1.
 *
 * roslibjs cannot request a QoS profile. rosbridge_suite 2.7.1 matches it:
 * when every publisher on the topic is TRANSIENT_LOCAL it subscribes
 * RELIABLE + TRANSIENT_LOCAL, so the retained sample arrives on subscribe.
 * The gate also republishes the lock on every 10 Hz tick, which covers a
 * subscription made before the gate started.
 *
 * States: "unknown" (no sample yet, sample stale, or rosbridge down),
 * "locked", "released". Unknown is never treated as released.
 */

export const LOCK_UNKNOWN = "unknown";
export const LOCK_LOCKED = "locked";
export const LOCK_RELEASED = "released";

export const evaluateLock = ({ lastValue, lastSampleMs }, nowMs) => {
  if (lastValue === null || lastSampleMs === null) return LOCK_UNKNOWN;
  // Fail closed: without a usable local time staleness cannot be checked.
  if (!Number.isFinite(nowMs) || !Number.isFinite(lastSampleMs)) return LOCK_UNKNOWN;
  if (nowMs - lastSampleMs > STOP_LOCK_STALE_AFTER_MS) return LOCK_UNKNOWN;
  return lastValue ? LOCK_LOCKED : LOCK_RELEASED;
};

/** Live subscription for one rosbridge connection; a reconnect makes a new one. */
export const createStopLock = ({
  ros,
  ROSLIB = window.ROSLIB,
  now = monotonicNowMs,
  onChange = () => {},
}) => {
  const sample = { lastValue: null, lastSampleMs: null };
  let state = LOCK_UNKNOWN;
  let stopped = false;

  const publishState = () => {
    const next = evaluateLock(sample, now());
    if (next !== state) {
      state = next;
      onChange(state);
    }
  };

  const topic = new ROSLIB.Topic({
    ros,
    name: AppConfig.STOP_LOCK_TOPIC,
    messageType: "std_msgs/Bool",
    queue_length: 1,
  });
  topic.subscribe((msg) => {
    if (stopped || typeof msg?.data !== "boolean") return;
    sample.lastValue = msg.data;
    sample.lastSampleMs = now();
    publishState();
  });
  const timer = setInterval(publishState, GATE_EVALUATE_PERIOD_MS);

  return {
    getState: () => evaluateLock(sample, now()),
    stop: () => {
      stopped = true;
      clearInterval(timer);
      topic.unsubscribe();
    },
  };
};

const callTrigger = (ros, name, ROSLIB = window.ROSLIB) =>
  new Promise((resolve) => {
    if (!ros || !ROSLIB) {
      resolve({ success: false, message: "rosbridge not available" });
      return;
    }
    const service = new ROSLIB.Service({ ros, name, serviceType: "std_srvs/Trigger" });
    service.callService(
      new ROSLIB.ServiceRequest({}),
      (response) =>
        resolve({ success: Boolean(response?.success), message: response?.message || "" }),
      (error) => resolve({ success: false, message: String(error || "service call failed") }),
    );
  });

export const engageStopLock = (ros) => callTrigger(ros, AppConfig.STOP_LOCK_ENGAGE_SERVICE);

// The only release path in the UI. Called from the Resume confirmation only.
export const releaseStopLock = (ros) => callTrigger(ros, AppConfig.STOP_LOCK_RELEASE_SERVICE);
