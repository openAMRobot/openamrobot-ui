import React, { useEffect, useRef } from "react";
import { toast } from "react-toastify";

import { useRos } from "../app/App";
import { AppConfig } from "../shared/constants";
import { addEvent } from "../shared/events/eventLog";
import { loadWaypoints } from "../shared/hooks/useSavedWaypoints";
import { getMission } from "../shared/missions/missions";
import { getRun, setRun, subscribeCommands } from "../shared/missions/missionRunner";

// Generous timeouts so a silent Nav2/dock stack can't hang a mission forever —
// these are "something is clearly wrong" backstops, not tuned durations.
const NAV_TIMEOUT_MS = 10 * 60 * 1000;
const DOCK_TIMEOUT_MS = 2 * 60 * 1000;

const stepLabel = (step, waypointName) => {
  switch (step.type) {
    case "waypoint":
      return `Go to "${waypointName || "unknown waypoint"}"`;
    case "home":
      return "Go home (0, 0)";
    case "wait":
      return `Wait ${step.seconds}s`;
    case "dock":
      return "Dock";
    case "undock":
      return "Undock";
    default:
      return step.type;
  }
};

/**
 * Headless, always-mounted (see AppLayout.jsx) executor for missions defined
 * on the Missions page — same reasoning as SchedulerRunner/NotificationsWatcher:
 * the ROS subscriptions this needs to exist exactly once regardless of which
 * page is currently open, and regardless of whether the run was started by
 * clicking "Run" on /missions or by a time-triggered schedule.
 *
 * Each step type resolves to a boolean success via its own one-shot ROS
 * subscription: nav steps (waypoint/home) watch /ui/navigate_to_pose/status
 * for a terminal GoalStatus (4=succeeded → true, 5/6=canceled/failed →
 * false) — the same status codes NavStatus.jsx already explains to
 * operators. Dock/undock steps watch dock_trigger_status the same way
 * DockingControl.jsx does. A mission only ever drives one step at a time, so
 * these subscriptions don't collide with the equivalent live-page components
 * — rosbridge topics support multiple independent subscribers.
 */
const MissionRunner = () => {
  const ros = useRos();
  const rosRef = useRef(ros);
  useEffect(() => {
    rosRef.current = ros;
  }, [ros]);

  const stopRequestedRef = useRef(false);
  const activeCancelRef = useRef(null);

  const stopMission = () => {
    stopRequestedRef.current = true;
    activeCancelRef.current?.();
  };

  const waitFor = (ms) =>
    new Promise((resolve) => {
      const id = setTimeout(() => resolve(true), ms);
      activeCancelRef.current = () => {
        clearTimeout(id);
        resolve(false);
      };
    });

  const runNavStep = (pose) =>
    new Promise((resolve) => {
      const activeRos = rosRef.current;
      if (!activeRos || !window.ROSLIB) {
        resolve(false);
        return;
      }

      const goalTopic = new window.ROSLIB.Topic({
        ros: activeRos,
        name: AppConfig.GOAL_POSE_TOPIC,
        messageType: "geometry_msgs/PoseStamped",
      });
      const statusTopic = new window.ROSLIB.Topic({
        ros: activeRos,
        name: AppConfig.NAV_STATUS_TOPIC,
        messageType: "action_msgs/GoalStatusArray",
      });
      const cancelClient = new window.ROSLIB.Service({
        ros: activeRos,
        name: AppConfig.NAV_CANCEL_GOAL_SERVICE,
        serviceType: "action_msgs/CancelGoal",
      });

      let settled = false;
      let timeoutId;
      const finish = (ok) => {
        if (settled) return;
        settled = true;
        clearTimeout(timeoutId);
        statusTopic.unsubscribe();
        resolve(ok);
      };
      timeoutId = setTimeout(() => finish(false), NAV_TIMEOUT_MS);

      statusTopic.subscribe((msg) => {
        if (!msg.status_list?.length) return;
        const latest = msg.status_list[msg.status_list.length - 1];
        if (latest.status === 4) finish(true);
        else if (latest.status === 5 || latest.status === 6) finish(false);
      });

      activeCancelRef.current = () => {
        try {
          cancelClient.callService(
            new window.ROSLIB.ServiceRequest({
              goal_info: {
                goal_id: { uuid: new Array(16).fill(0) },
                stamp: { sec: 0, nanosec: 0 },
              },
            }),
            () => {},
            () => {},
          );
        } catch {
          // best-effort — the mission is stopping either way
        }
        finish(false);
      };

      goalTopic.publish(
        new window.ROSLIB.Message({
          header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
          pose: {
            position: { x: pose.x, y: pose.y, z: 0 },
            orientation: { x: 0, y: 0, z: pose.z ?? 0, w: pose.w ?? 1 },
          },
        }),
      );
    });

  const runDockStep = (undo) =>
    new Promise((resolve) => {
      const activeRos = rosRef.current;
      if (!activeRos || !window.ROSLIB) {
        resolve(false);
        return;
      }

      const triggerTopic = new window.ROSLIB.Topic({
        ros: activeRos,
        name: undo ? AppConfig.UNDOCK_TRIGGER_TOPIC : AppConfig.DOCK_TRIGGER_TOPIC,
        messageType: "std_msgs/Bool",
      });
      const statusTopic = new window.ROSLIB.Topic({
        ros: activeRos,
        name: AppConfig.DOCK_TRIGGER_STATUS_TOPIC,
        messageType: "std_msgs/String",
      });

      let settled = false;
      let timeoutId;
      const finish = (ok) => {
        if (settled) return;
        settled = true;
        clearTimeout(timeoutId);
        statusTopic.unsubscribe();
        resolve(ok);
      };
      timeoutId = setTimeout(() => finish(false), DOCK_TIMEOUT_MS);

      statusTopic.subscribe((msg) => {
        if (undo) {
          if (msg.data === "idle") finish(true);
          else if (msg.data === "failed") finish(false);
        } else if (msg.data === "docked") {
          finish(true);
        } else if (msg.data === "failed") {
          finish(false);
        }
      });

      activeCancelRef.current = () => finish(false);

      triggerTopic.publish(new window.ROSLIB.Message({ data: true }));
    });

  const runStep = (step) => {
    if (step.type === "wait") return waitFor((step.seconds || 0) * 1000);
    if (step.type === "home") return runNavStep({ x: 0, y: 0, z: 0, w: 1 });
    if (step.type === "waypoint") {
      const wp = loadWaypoints().find((w) => w.id === step.waypointId);
      if (!wp) return Promise.resolve(false);
      return runNavStep(wp);
    }
    if (step.type === "dock") return runDockStep(false);
    if (step.type === "undock") return runDockStep(true);
    return Promise.resolve(false);
  };

  const startMission = async (missionId) => {
    if (getRun()?.status === "running") {
      toast.warn("A mission is already running — stop it first.");
      return;
    }
    const mission = getMission(missionId);
    if (!mission || !mission.steps.length) {
      toast.error("That mission has no steps to run.");
      return;
    }

    stopRequestedRef.current = false;
    const waypoints = loadWaypoints();
    const labelFor = (step) =>
      stepLabel(
        step,
        step.type === "waypoint"
          ? waypoints.find((w) => w.id === step.waypointId)?.name
          : null,
      );

    setRun({ missionId, stepIndex: 0, status: "running", log: [] });
    addEvent({
      type: "navigation",
      severity: "info",
      message: `Mission "${mission.name}" started`,
    });

    for (let i = 0; i < mission.steps.length; i++) {
      if (stopRequestedRef.current) {
        setRun({ ...getRun(), status: "stopped" });
        addEvent({
          type: "navigation",
          severity: "warning",
          message: `Mission "${mission.name}" stopped`,
        });
        return;
      }

      const step = mission.steps[i];
      setRun({ ...getRun(), stepIndex: i });
      // Steps are inherently sequential — each depends on the previous one finishing.
      const ok = await runStep(step);
      const label = labelFor(step);
      setRun({ ...getRun(), log: [...getRun().log, { label, ok, time: Date.now() }] });

      if (!ok) {
        const status = stopRequestedRef.current ? "stopped" : "failed";
        setRun({ ...getRun(), status });
        if (status === "failed") {
          toast.error(`Mission "${mission.name}" failed at step ${i + 1}: ${label}`);
          addEvent({
            type: "navigation",
            severity: "error",
            message: `Mission "${mission.name}" failed at "${label}"`,
          });
        }
        return;
      }
    }

    setRun({ ...getRun(), status: "succeeded" });
    toast.success(`Mission "${mission.name}" complete`);
    addEvent({
      type: "navigation",
      severity: "success",
      message: `Mission "${mission.name}" completed`,
    });
  };

  useEffect(() => {
    // startMission/stopMission close over refs (not React state), so they're
    // effectively stable — intentionally omitted from the dependency list.
    const unsubscribe = subscribeCommands((cmd) => {
      if (cmd.type === "start") startMission(cmd.missionId);
      else if (cmd.type === "stop") stopMission();
    });
    return unsubscribe;
  }, []);

  return null;
};

export default MissionRunner;
