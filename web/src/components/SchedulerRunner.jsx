import { useEffect, useRef } from "react";
import { toast } from "react-toastify";

import { useRos } from "../app/App";
import { AppConfig } from "../shared/constants";
import { getSchedules, updateSchedule } from "../shared/schedules/schedules";
import { addEvent } from "../shared/events/eventLog";
import { loadWaypoints as readWaypoints } from "../shared/hooks/useSavedWaypoints";
import { requestStart as requestMissionStart } from "../shared/missions/missionRunner";

// Minute-resolution key so a schedule fires once per matching minute even
// though we poll more often than that.
const minuteKey = (d) =>
  `${d.getFullYear()}-${d.getMonth()}-${d.getDate()}-${d.getHours()}-${d.getMinutes()}`;
const hhmm = (d) =>
  `${String(d.getHours()).padStart(2, "0")}:${String(d.getMinutes()).padStart(2, "0")}`;

/**
 * Headless (AppLayout) runner that fires due mission schedules while the tab
 * is open. Each schedule triggers a navigation goal (a saved waypoint, or
 * home) by publishing to /goal_pose — the same publish the Map page uses.
 */
const SchedulerRunner = () => {
  const ros = useRos();
  const goalRef = useRef(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return undefined;
    goalRef.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.GOAL_POSE_TOPIC,
      messageType: "geometry_msgs/PoseStamped",
    });
    return () => {
      goalRef.current = null;
    };
  }, [ros]);

  useEffect(() => {
    const publishGoal = (x, y, z = 0, w = 1) => {
      if (!goalRef.current) return false;
      goalRef.current.publish(
        new window.ROSLIB.Message({
          header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
          pose: {
            position: { x, y, z: 0 },
            orientation: { x: 0, y: 0, z, w },
          },
        }),
      );
      return true;
    };

    const runAction = (s) => {
      if (s.action?.type === "mission") {
        requestMissionStart(s.action.missionId);
        toast.info(`Scheduled: running mission "${s.name}"`);
        addEvent({ type: "system", severity: "info", message: `Schedule "${s.name}" fired → mission run started` });
        return;
      }

      let ok = false;
      let where = "";
      if (s.action?.type === "home") {
        ok = publishGoal(0, 0);
        where = "home (0, 0)";
      } else if (s.action?.type === "waypoint") {
        const wp = readWaypoints().find((w) => w.id === s.action.waypointId);
        if (wp) {
          ok = publishGoal(wp.x, wp.y, wp.z, wp.w);
          where = `"${wp.name}"`;
        }
      }
      if (ok) {
        toast.info(`Scheduled: navigating to ${where}`);
        addEvent({ type: "system", severity: "info", message: `Schedule "${s.name}" fired → ${where}` });
      } else {
        addEvent({ type: "system", severity: "warning", message: `Schedule "${s.name}" could not run (target missing or disconnected)` });
      }
    };

    const tick = () => {
      const now = new Date();
      const key = minuteKey(now);
      const target = hhmm(now);
      getSchedules().forEach((s) => {
        if (!s.enabled) return;
        if (s.time !== target) return;
        if (s.lastRunKey === key) return; // already fired this minute
        updateSchedule(s.id, {
          lastRunKey: key,
          ...(s.repeat === "once" ? { enabled: false } : {}),
        });
        runAction(s);
      });
    };

    const id = setInterval(tick, 15000);
    tick();
    return () => clearInterval(id);
  }, []);

  return null;
};

export default SchedulerRunner;
