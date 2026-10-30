import { useEffect, useState } from "react";

import { useRos, useRosStatus } from "../../app/App";
import { AppConfig } from "../constants";
import { addEvent } from "../events/eventLog";
import { getRun, subscribeRun } from "../missions/missionRunner";
import { getMission } from "../missions/missions";
import { monotonicNowMs } from "../motion/robotClock";
import { getUiRun, subscribeUiRuns } from "../runs/uiRunStore";
import { deriveReport, initialReportingState, reduceReporting } from "./reportingAdapter";

// How often the view is re-derived so staleness shows without new data.
// Proposed default: a quarter of the shortest stale threshold (1 s).
export const REPORT_REFRESH_MS = 250;

/**
 * Live, read-only feed for the reporting adapter: it only subscribes to
 * ROS topics and browser-side run stores. It never publishes, advertises
 * or calls a service or action.
 */
// `wallNow` gives the wall-clock time shown for a latched diagnostics fault;
// freshness always uses the monotonic `now`.
export const createReportingSource = ({ ros, ROSLIB = window.ROSLIB, now = monotonicNowMs, wallNow = Date.now }) => {
  let state = initialReportingState();
  let view = deriveReport(state, now());
  const listeners = new Set();
  const cleanups = [];

  const emit = () => {
    view = deriveReport(state, now());
    listeners.forEach((fn) => fn(view));
  };

  const dispatch = (event) => {
    const { state: next, logs } = reduceReporting(state, event, now(), wallNow());
    state = next;
    logs.forEach((entry) => addEvent(entry));
    emit();
  };

  const subscribe = (name, messageType, type, extra = {}) => {
    if (!ros || !ROSLIB) return;
    const topic = new ROSLIB.Topic({ ros, name, messageType, ...extra });
    const handler = (msg) => dispatch({ type, msg });
    topic.subscribe(handler);
    cleanups.push(() => topic.unsubscribe(handler));
  };

  subscribe(AppConfig.STOP_LOCK_TOPIC, "std_msgs/Bool", "lock", { queue_length: 1 });
  subscribe(AppConfig.NAVIGATION_STATUS_TOPIC, "openamr_nav_msgs/msg/NavigationStatus", "navigationStatus", { queue_length: 1 });
  subscribe(AppConfig.BATTERY_TOPIC, "std_msgs/Float32", "battery", { queue_length: 1 });
  subscribe(AppConfig.DIAGNOSTICS_TOPIC, "diagnostic_msgs/DiagnosticArray", "diagnostics");
  subscribe(AppConfig.NAV_STATUS_TOPIC, "action_msgs/GoalStatusArray", "goalStatus");
  subscribe(AppConfig.NAV_FEEDBACK_TOPIC, "nav2_msgs/action/NavigateToPose_FeedbackMessage", "navFeedback", { queue_length: 1 });

  // Missions: the runner's store has no run id, so one is made per start.
  let missionCount = 0;
  let missionRunId = null;
  let lastMission = null;
  const onMission = (run) => {
    if (!run) return;
    const started =
      run.status === "running" &&
      (!lastMission || lastMission.status !== "running" || lastMission.missionId !== run.missionId);
    if (started) {
      missionCount += 1;
      missionRunId = `mission:${run.missionId}:${missionCount}`;
    }
    lastMission = run;
    if (!missionRunId) return;
    const mission = getMission(run.missionId);
    const step = mission?.steps?.[run.stepIndex];
    const waitingOn =
      run.status === "running" && step?.type === "wait"
        ? `Mission step ${run.stepIndex + 1}: waiting ${step.seconds || 0} s`
        : "";
    dispatch({
      type: "mission",
      name: mission?.name || "Mission",
      run: {
        runId: missionRunId,
        step: Number.isInteger(run.stepIndex) ? run.stepIndex + 1 : null,
        total: mission?.steps?.length ?? null,
        status: run.status,
        reason: run.status === "failed" ? "A mission step failed" : run.status === "stopped" ? "Mission stopped" : "",
        waitingOn,
      },
    });
  };
  onMission(getRun());
  cleanups.push(subscribeRun(onMission));

  ["program", "route"].forEach((kind) => {
    const run = getUiRun(kind);
    if (run) dispatch({ type: "uiRun", kind, run });
  });
  cleanups.push(subscribeUiRuns((kind, run) => dispatch({ type: "uiRun", kind, run })));

  const timer = setInterval(emit, REPORT_REFRESH_MS);

  return {
    setConnection: (status) => dispatch({ type: "connection", status }),
    getView: () => view,
    subscribe: (fn) => {
      listeners.add(fn);
      return () => listeners.delete(fn);
    },
    stop: () => {
      clearInterval(timer);
      cleanups.forEach((fn) => fn());
    },
  };
};

// One shared source per ROS connection object, for every consumer.
let shared = null; // { ros, source, users }

const acquire = (ros) => {
  if (!shared || shared.ros !== ros) {
    shared?.source.stop();
    shared = { ros, source: createReportingSource({ ros }), users: 0 };
  }
  shared.users += 1;
  return shared.source;
};

const release = (source) => {
  if (!shared || shared.source !== source) return;
  shared.users -= 1;
  if (shared.users <= 0) {
    shared.source.stop();
    shared = null;
  }
};

/** Hook used by every reporting consumer (Reporting Mode, operator views). */
export const useReporting = () => {
  const ros = useRos();
  const rosStatus = useRosStatus();
  const [source, setSource] = useState(null);
  const [view, setView] = useState(null);

  useEffect(() => {
    const s = acquire(ros);
    setSource(s);
    setView(s.getView());
    const unsubscribe = s.subscribe(setView);
    return () => {
      unsubscribe();
      release(s);
    };
  }, [ros]);

  useEffect(() => {
    source?.setConnection(rosStatus === "connected" ? "connected" : "disconnected");
  }, [source, rosStatus]);

  return view;
};
