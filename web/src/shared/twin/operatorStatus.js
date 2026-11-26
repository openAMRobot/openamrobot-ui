import { AppConfig } from "../constants";
import { monotonicNowMs } from "../motion/robotClock";
import { LOCK_LOCKED, LOCK_UNKNOWN } from "../motion/stopLock";
import { STOP_LOCK_STALE_AFTER_MS } from "../motion/timing";
import { reasonName } from "./navigationReasons";

/**
 * Read-only operator status: what the robot (or the digital twin) reports
 * about readiness, the stop path, the current task and the mission. It only
 * subscribes. It never publishes, calls a service or changes the motion
 * gate; Stop (software) and Resume stay in softwareStop.js and
 * MotionStopBanner.jsx.
 *
 * Sources
 * - /navigation/status (openamr_nav_msgs/NavigationStatus): readiness,
 *   health, not-ready reasons, Nav2 stack state, current navigation task.
 * - /safety/stop_state (openamr_nav_msgs/StopState, platform-sw#90): gate
 *   state and the reason of a refused release. Absent without #90.
 * - /baseline1/mission_status (std_msgs/String JSON): Proposed contract
 *   (openamr-upperbody-sw#82). Published only while a Baseline 1 mission runs.
 * - /odom (nav_msgs/Odometry), throttled: only its age, for "pose stale".
 *
 * The stale limits are display values in wall time of this browser. They
 * are not safety limits.
 */

// navigation_status_node publishes on change and at least at heartbeat_hz
// (2 Hz on the simulation clock). At a real time factor of 0.2 that is one
// message every 2.5 s of wall time; 6 s allows two missed heartbeats.
export const NAVIGATION_STATUS_STALE_AFTER_MS = 6000;
// The gate publishes /safety/stop_state on the same 20 Hz tick as
// /safety/stop_lock, so it goes stale on the same limit.
export const STOP_STATE_STALE_AFTER_MS = STOP_LOCK_STALE_AFTER_MS;
export const ODOM_STALE_AFTER_MS = 3000;
export const ODOM_THROTTLE_MS = 500;

export const DISPLAY_DISCONNECTED = "DISCONNECTED";
export const DISPLAY_STOP_UNKNOWN = "STOP STATE UNKNOWN";
export const DISPLAY_STOPPED = "STOPPED";
export const DISPLAY_WAITING = "WAITING FOR STATUS";
export const DISPLAY_STALE = "STALE";
export const DISPLAY_FAULTED = "FAULTED";
export const DISPLAY_NOT_READY = "NOT READY";
export const DISPLAY_READY = "READY";

const HEALTH = ["UNKNOWN", "OK", "DEGRADED", "FAULT"];
const HEALTH_FAULT = 3;
const READINESS_READY = 1;
const TASK_STATES = [
  "UNKNOWN",
  "IDLE",
  "ACTIVE",
  "RECOVERING",
  "SUCCEEDED",
  "ABORTED",
  "CANCELED",
];
const STACK_STATES = ["UNKNOWN", "NOT_STARTED", "STARTING", "ACTIVE", "RESETTING", "FAILED"];
const STOP_STATES = ["LOCKED", "RELEASE_REFUSED", "RUNNING"];
const STOP_LOCKED = 0;
const STOP_REFUSED = 1;

const nameOf = (names, value) =>
  Number.isInteger(value) && value >= 0 && value < names.length ? names[value] : `code ${value}`;

/** "none" (never received), "fresh" or "stale" for one sample {msg, atMs}. */
export const freshness = (sample, nowMs, staleAfterMs) => {
  if (!sample || sample.atMs === null || sample.atMs === undefined) return "none";
  if (!Number.isFinite(nowMs) || nowMs - sample.atMs > staleAfterMs) return "stale";
  return "fresh";
};

const ageText = (sample, nowMs) =>
  sample && Number.isFinite(sample.atMs) ? `${((nowMs - sample.atMs) / 1000).toFixed(1)} s ago` : "";

/** Parses the Proposed #82 payload; null when it is not the expected JSON. */
export const parseMissionStatus = (data) => {
  if (typeof data !== "string") return null;
  try {
    const value = JSON.parse(data);
    if (!value || typeof value !== "object") return null;
    const phases = Array.isArray(value.phases) ? value.phases : [];
    return {
      phase: typeof value.phase === "string" ? value.phase : null,
      state: typeof value.state === "string" ? value.state : "unknown",
      sim: Number.isFinite(value.sim) ? value.sim : null,
      phases: phases
        .filter((p) => p && typeof p.phase === "string")
        .map((p) => ({ phase: p.phase, verdict: String(p.verdict || "") })),
    };
  } catch {
    return null;
  }
};

/**
 * Pure: everything OperatorStatusPanel shows, from the motion gate fields and
 * the samples. `gate` is {connection, lock, localStop} from motionGate.js.
 */
export const deriveOperatorStatus = ({ gate, navigation, stopState, mission, odom }, nowMs) => {
  const connected = gate?.connection === "connected";
  const nav = freshness(navigation, nowMs, NAVIGATION_STATUS_STALE_AFTER_MS);
  const stop = freshness(stopState, nowMs, STOP_STATE_STALE_AFTER_MS);
  const pose = freshness(odom, nowMs, ODOM_STALE_AFTER_MS);
  const navMsg = nav === "fresh" ? navigation.msg : null;
  const stopMsg = stop === "fresh" ? stopState.msg : null;

  // Readiness and controller state, from /navigation/status.
  let readiness = { text: "unknown", reasons: [], health: "unknown", stack: "unknown" };
  if (navMsg) {
    readiness = {
      text: navMsg.navigation_readiness === READINESS_READY ? "READY" : "NOT READY",
      reasons: (navMsg.not_ready_reasons || []).map(reasonName),
      active: (navMsg.active_reasons || []).map(reasonName),
      health: nameOf(HEALTH, navMsg.health),
      stack: nameOf(STACK_STATES, navMsg.stack?.state),
      inactiveNodes: navMsg.stack?.inactive_nodes || [],
    };
  }

  // Stop path, from the gate's own state topic (display only).
  let stopRow;
  if (stopMsg) {
    stopRow = {
      text: nameOf(STOP_STATES, stopMsg.state),
      reason: stopMsg.state === STOP_REFUSED ? stopMsg.reason || "no reason given" : "",
    };
  } else if (stop === "stale") {
    stopRow = { text: `stale (${ageText(stopState, nowMs)})`, reason: "" };
  } else {
    stopRow = {
      text: "no sample",
      reason:
        "No /safety/stop_state on this connection. Stop (software) and Resume need " +
        "openamr-platform-sw#90 on the running stack.",
    };
  }

  // Current task: the Baseline 1 mission when it reports (Proposed #82),
  // otherwise the navigation task from /navigation/status.
  const missionValue = mission?.msg ? parseMissionStatus(mission.msg.data) : null;
  let task;
  if (missionValue) {
    task = {
      source: "/baseline1/mission_status (Proposed, openamr-upperbody-sw#82)",
      text: `Baseline 1: ${missionValue.phase || "no phase"} (${missionValue.state})`,
      phases: missionValue.phases,
    };
  } else if (navMsg) {
    const t = navMsg.task || {};
    const distance = Number.isFinite(t.distance_remaining)
      ? `, ${t.distance_remaining.toFixed(2)} m remaining`
      : "";
    task = {
      source: "/navigation/status task",
      text: `Navigation ${nameOf(TASK_STATES, t.state)}${t.state === 2 ? distance : ""}${
        t.reason ? `, ${reasonName(t.reason)}` : ""
      }`,
      phases: [],
    };
  } else {
    task = { source: "none", text: "unknown", phases: [] };
  }

  const navStateText =
    nav === "none" ? "no /navigation/status yet" : nav === "stale" ? `stale (${ageText(navigation, nowMs)})` : "fresh";
  const poseText =
    pose === "none" ? "no /odom yet" : pose === "stale" ? `stale (${ageText(odom, nowMs)})` : "fresh";

  // One display state, most severe first.
  let display;
  let detail;
  if (!connected) {
    display = DISPLAY_DISCONNECTED;
    detail = "rosbridge is disconnected. Nothing below is live.";
  } else if (gate.localStop || gate.lock === LOCK_LOCKED || stopMsg?.state === STOP_LOCKED || stopMsg?.state === STOP_REFUSED) {
    display = DISPLAY_STOPPED;
    detail =
      stopMsg?.state === STOP_REFUSED
        ? `Resume was refused: ${stopRow.reason}`
        : "Stop (software) is engaged. Resume is the only way out.";
  } else if (gate.lock === LOCK_UNKNOWN) {
    display = DISPLAY_STOP_UNKNOWN;
    detail = "No recent /safety/stop_lock sample. Treated as stopped; manual drive and goals stay disabled.";
  } else if (nav === "none") {
    display = DISPLAY_WAITING;
    detail = "No /navigation/status received yet.";
  } else if (nav === "stale" || pose === "stale") {
    display = DISPLAY_STALE;
    detail = `Navigation status ${navStateText}; robot pose ${poseText}.`;
  } else if (navMsg.health === HEALTH_FAULT) {
    display = DISPLAY_FAULTED;
    detail = `Health FAULT: ${readiness.active.join(", ") || readiness.reasons.join(", ") || "no reason given"}.`;
  } else if (navMsg.navigation_readiness !== READINESS_READY) {
    display = DISPLAY_NOT_READY;
    detail = readiness.reasons.length ? readiness.reasons.join(", ") : "No reason given.";
  } else {
    display = DISPLAY_READY;
    detail = "Navigation reports ready.";
  }

  return {
    display,
    detail,
    readiness,
    stop: stopRow,
    task,
    sources: { navigation: navStateText, pose: poseText, stopState: stop },
  };
};

/**
 * Live subscriptions for one rosbridge connection. Calls onSample(key,
 * sample) with key "navigation", "stopState", "mission" or "odom". Returns
 * stop(), which unsubscribes.
 */
export const createOperatorStatusFeed = ({
  ros,
  ROSLIB = window.ROSLIB,
  now = monotonicNowMs,
  onSample = () => {},
}) => {
  let stopped = false;
  const specs = [
    ["navigation", AppConfig.NAVIGATION_STATUS_TOPIC, "openamr_nav_msgs/msg/NavigationStatus", 0],
    ["stopState", AppConfig.STOP_STATE_TOPIC, "openamr_nav_msgs/msg/StopState", 0],
    ["mission", AppConfig.MISSION_STATUS_TOPIC, "std_msgs/msg/String", 0],
    ["odom", AppConfig.ROBOT_POSE_TOPIC, "nav_msgs/msg/Odometry", ODOM_THROTTLE_MS],
  ];
  const topics = specs.map(([key, name, messageType, throttleRate]) => {
    const topic = new ROSLIB.Topic({
      ros,
      name,
      messageType,
      queue_length: 1,
      ...(throttleRate ? { throttle_rate: throttleRate } : {}),
    });
    topic.subscribe((msg) => {
      if (stopped) return;
      onSample(key, { msg, atMs: now() });
    });
    return topic;
  });
  return {
    stop: () => {
      stopped = true;
      topics.forEach((t) => t.unsubscribe());
    },
  };
};
