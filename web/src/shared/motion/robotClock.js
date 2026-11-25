import { AppConfig } from "../constants";
import {
  CLOCK_RESYNC_PERIOD_MS,
  CLOCK_SAMPLE_WINDOW,
  GATE_EVALUATE_PERIOD_MS,
  MAX_STAMP_AGE_AT_CREATION_S,
  MAX_SYNC_AGE_MS,
  MODE_RETRY_PERIOD_MS,
  SIM_CLOCK_MAX_SILENCE_MS,
  SIM_CLOCK_THROTTLE_MS,
  STAMP_EXTRA_MARGIN_S,
} from "./timing";

/**
 * Robot clock for stamping /ui/cmd_vel.
 *
 * ui_cmd_vel_relay_node judges each stamp against its own ROS clock, and the
 * browser runs on another device, so Date.now() alone is wrong. The mode
 * follows the relay's own use_sim_time, read through
 * /ui_cmd_vel_relay_node/get_parameters; until that read succeeds the mode
 * is unknown and manual drive stays disabled.
 *
 * Simulation: each command carries the latest /clock value received, never
 * extrapolated. A stamp can then only be older than robot time, also when
 * sim time runs slower than real time or is paused. /rosapi/get_time is not
 * used here: in rosbridge_suite 2.7.1 it returns wall time even with
 * use_sim_time true.
 *
 * Real robot: the offset to the robot clock comes from /rosapi/get_time
 * samples. With send time t0, receive time t1 and robot time R, the offset
 * estimate R - (t0 + t1) / 2 is wrong by at most RTT/2 in either direction,
 * however the delay splits between the two legs. Subtracting RTT/2 plus
 * 5 ms from the estimate therefore never yields a stamp ahead of robot time.
 */

export const NOT_SYNCED_LABEL = "Robot clock not synced";

// rcl_interfaces/msg/ParameterType.PARAMETER_BOOL
const PARAMETER_BOOL = 1;

// performance.timeOrigin + performance.now() does not jump with wall-clock
// changes. Falls back to Date.now() where either is missing or not a number.
export const monotonicNowMs = () => {
  if (
    typeof performance !== "undefined" &&
    typeof performance.timeOrigin === "number" &&
    typeof performance.now === "function"
  ) {
    const value = performance.timeOrigin + performance.now();
    if (Number.isFinite(value)) return value;
  }
  return Date.now();
};

export const toSeconds = (time) =>
  Number(time?.sec ?? 0) + Number(time?.nanosec ?? 0) * 1e-9;

export const toStamp = (seconds) => {
  let sec = Math.floor(seconds);
  let nanosec = Math.round((seconds - sec) * 1e9);
  if (nanosec >= 1e9) {
    sec += 1;
    nanosec -= 1e9;
  }
  return { sec, nanosec };
};

/** One /rosapi/get_time round trip, reduced to its RTT and offset estimate. */
export const makeSample = ({ sentAtMs, receivedAtMs, robotTime }) => {
  const rttS = Math.max(0, (receivedAtMs - sentAtMs) / 1000);
  const midpointS = (sentAtMs + receivedAtMs) / 2000;
  return { rttS, offsetS: toSeconds(robotTime) - midpointS, receivedAtMs };
};

export const pushSample = (samples, sample, window = CLOCK_SAMPLE_WINDOW) =>
  [...samples, sample].slice(-window);

export const bestSample = (samples) =>
  samples.reduce((best, s) => (best === null || s.rttS < best.rttS ? s : best), null);

/** Margin subtracted from the estimated robot time: RTT/2 plus 5 ms. */
export const stampMarginS = (sample) => sample.rttS / 2 + STAMP_EXTRA_MARGIN_S;

export const realStampSeconds = (sample, nowMs) =>
  nowMs / 1000 + sample.offsetS - stampMarginS(sample);

const notSynced = (mode, detail) => ({
  mode,
  synced: false,
  label: NOT_SYNCED_LABEL,
  detail,
});

export const evaluateReal = ({ samples, lastGoodSyncMs, nowMs }) => {
  const best = bestSample(samples);
  if (!best || lastGoodSyncMs === null) {
    return notSynced("real", "No robot time sample yet");
  }
  if (nowMs - lastGoodSyncMs > MAX_SYNC_AGE_MS) {
    return notSynced("real", "Last robot time sync is older than 6 s");
  }
  if (best.rttS / 2 + stampMarginS(best) > MAX_STAMP_AGE_AT_CREATION_S) {
    return notSynced("real", "Network delay too high for a fresh stamp");
  }
  return { mode: "real", synced: true, label: "Robot clock synced", detail: "" };
};

export const evaluateSim = ({ lastClock, lastClockReceivedMs, nowMs }) => {
  if (!lastClock || lastClockReceivedMs === null) {
    return notSynced("sim", "No /clock received yet");
  }
  if (nowMs - lastClockReceivedMs > SIM_CLOCK_MAX_SILENCE_MS) {
    return notSynced("sim", "No /clock for more than 0.15 s");
  }
  // The relay drops a literal zero stamp, so a sim clock still at zero
  // cannot stamp a command the relay would forward.
  if (Number(lastClock.sec) === 0 && Number(lastClock.nanosec) === 0) {
    return notSynced("sim", "Simulation clock is at zero");
  }
  return { mode: "sim", synced: true, label: "Robot clock synced", detail: "" };
};

export const evaluateClock = (state, nowMs) => {
  // Fail closed: without a usable local time no age rule can be checked.
  if (!Number.isFinite(nowMs)) return notSynced(state.mode, "Local time unavailable");
  if (state.mode === "sim") return evaluateSim({ ...state, nowMs });
  if (state.mode === "real") return evaluateReal({ ...state, nowMs });
  return notSynced("unknown", "Clock mode unknown: waiting for the relay's use_sim_time");
};

/** Stamp for a command sent now, or null when manual drive must stay disabled. */
export const stampFor = (state, nowMs) => {
  if (!evaluateClock(state, nowMs).synced) return null;
  if (state.mode === "sim") {
    return { sec: Number(state.lastClock.sec), nanosec: Number(state.lastClock.nanosec) };
  }
  return toStamp(realStampSeconds(bestSample(state.samples), nowMs));
};

const sameStatus = (a, b) =>
  a.mode === b.mode && a.synced === b.synced && a.detail === b.detail;

/**
 * Live controller for one rosbridge connection. A reconnect creates a new
 * one, so every connection starts in the unknown mode.
 */
export const createRobotClock = ({
  ros,
  ROSLIB = window.ROSLIB,
  now = monotonicNowMs,
  onChange = () => {},
}) => {
  const state = {
    mode: "unknown",
    samples: [],
    lastGoodSyncMs: null,
    lastClock: null,
    lastClockReceivedMs: null,
  };
  let stopped = false;
  let status = evaluateClock(state, now());
  let modeRetry = null;
  let resyncTimer = null;
  let clockTopic = null;
  let syncInFlight = false;

  const publishStatus = () => {
    const next = evaluateClock(state, now());
    if (!sameStatus(next, status)) {
      status = next;
      onChange(status);
    }
  };
  const evaluateTimer = setInterval(publishStatus, GATE_EVALUATE_PERIOD_MS);

  const startSim = () => {
    clockTopic = new ROSLIB.Topic({
      ros,
      name: AppConfig.CLOCK_TOPIC,
      messageType: "rosgraph_msgs/Clock",
      throttle_rate: SIM_CLOCK_THROTTLE_MS,
      queue_length: 1,
    });
    clockTopic.subscribe((msg) => {
      if (stopped || !msg?.clock) return;
      state.lastClock = { sec: Number(msg.clock.sec), nanosec: Number(msg.clock.nanosec) };
      state.lastClockReceivedMs = now();
      publishStatus();
    });
  };

  const startReal = () => {
    const timeService = new ROSLIB.Service({
      ros,
      name: AppConfig.ROSAPI_GET_TIME_SERVICE,
      serviceType: "rosapi_msgs/srv/GetTime",
    });
    const sync = () => {
      if (stopped || syncInFlight) return;
      syncInFlight = true;
      const sentAtMs = now();
      timeService.callService(
        new ROSLIB.ServiceRequest({}),
        (response) => {
          syncInFlight = false;
          if (stopped || !response?.time) return;
          const robotTime = response.time;
          if (Number(robotTime.sec) === 0 && Number(robotTime.nanosec) === 0) return;
          const receivedAtMs = now();
          state.samples = pushSample(
            state.samples,
            makeSample({ sentAtMs, receivedAtMs, robotTime }),
          );
          state.lastGoodSyncMs = receivedAtMs;
          publishStatus();
        },
        () => {
          syncInFlight = false;
        },
      );
    };
    sync();
    resyncTimer = setInterval(() => {
      // A call that never answered must not block resyncs forever; its late
      // answer would only add a high-RTT sample, which is never the best.
      syncInFlight = false;
      sync();
    }, CLOCK_RESYNC_PERIOD_MS);
  };

  const paramsService = new ROSLIB.Service({
    ros,
    name: AppConfig.UI_CMD_VEL_RELAY_GET_PARAMETERS_SERVICE,
    serviceType: "rcl_interfaces/srv/GetParameters",
  });
  let attempt = 0;
  const detectMode = () => {
    if (stopped || state.mode !== "unknown") return;
    attempt += 1;
    const thisAttempt = attempt;
    const retry = () => {
      if (stopped || state.mode !== "unknown" || thisAttempt !== attempt) return;
      clearTimeout(modeRetry);
      modeRetry = setTimeout(detectMode, MODE_RETRY_PERIOD_MS);
    };
    // Also retry if the call never answers.
    clearTimeout(modeRetry);
    modeRetry = setTimeout(detectMode, MODE_RETRY_PERIOD_MS);
    paramsService.callService(
      new ROSLIB.ServiceRequest({ names: ["use_sim_time"] }),
      (response) => {
        if (stopped || state.mode !== "unknown") return;
        const value = response?.values?.[0];
        if (!value || Number(value.type) !== PARAMETER_BOOL) {
          retry();
          return;
        }
        clearTimeout(modeRetry);
        state.mode = value.bool_value ? "sim" : "real";
        if (state.mode === "sim") startSim();
        else startReal();
        publishStatus();
      },
      retry,
    );
  };
  detectMode();

  return {
    getStatus: () => evaluateClock(state, now()),
    getStamp: () => stampFor(state, now()),
    stop: () => {
      stopped = true;
      clearInterval(evaluateTimer);
      clearInterval(resyncTimer);
      clearTimeout(modeRetry);
      clockTopic?.unsubscribe();
    },
  };
};
