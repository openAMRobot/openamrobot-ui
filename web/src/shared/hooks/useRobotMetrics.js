import { useEffect, useRef, useState } from "react";

import { useRos } from "../../app/App";
import { AppConfig } from "../constants";

const STORAGE_KEY = "openamrMetrics";

// Odometry can jump discontinuously when localization is reset or the robot is
// teleported in sim — ignore single-step deltas larger than this (metres) so a
// jump doesn't inflate the distance-travelled odometer.
const MAX_STEP_M = 1.0;

const emptyMetrics = () => ({
  distance: 0, // metres travelled (integrated odom)
  goalsSucceeded: 0,
  goalsFailed: 0,
  goalsCanceled: 0,
  dockSuccess: 0,
  dockFail: 0,
  maxSpeed: 0, // m/s
  since: Date.now(), // when counting started (persisted)
});

const load = () => {
  try {
    return { ...emptyMetrics(), ...JSON.parse(localStorage.getItem(STORAGE_KEY) || "{}") };
  } catch {
    return emptyMetrics();
  }
};

/**
 * Derives a lightweight "track record" for the robot from telemetry it already
 * publishes: distance travelled (integrated odom), current/peak speed, goal
 * outcomes and docking outcomes. Cumulative counters persist to localStorage
 * (survive reloads) and are flushed at 1 Hz so per-message odom updates don't
 * thrash React. Speed is live-only.
 */
export default function useRobotMetrics() {
  const ros = useRos();
  const [metrics, setMetrics] = useState(load);
  const [speed, setSpeed] = useState(0);

  const accRef = useRef(load());
  const lastPosRef = useRef(null);
  const lastGoalKeyRef = useRef(null);
  const dirtyRef = useRef(false);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return undefined;

    const odom = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.ROBOT_POSE_TOPIC,
      messageType: "nav_msgs/Odometry",
    });
    odom.subscribe((msg) => {
      const p = msg?.pose?.pose?.position;
      const v = msg?.twist?.twist?.linear;
      if (v) {
        const s = Math.hypot(v.x || 0, v.y || 0);
        setSpeed(s);
        if (s > accRef.current.maxSpeed) {
          accRef.current.maxSpeed = s;
          dirtyRef.current = true;
        }
      }
      if (p) {
        const last = lastPosRef.current;
        if (last) {
          const step = Math.hypot(p.x - last.x, p.y - last.y);
          if (step > 0.002 && step < MAX_STEP_M) {
            accRef.current.distance += step;
            dirtyRef.current = true;
          }
        }
        lastPosRef.current = { x: p.x, y: p.y };
      }
    });

    const navStatus = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.NAV_STATUS_TOPIC,
      messageType: "action_msgs/GoalStatusArray",
    });
    navStatus.subscribe((msg) => {
      if (!msg.status_list?.length) return;
      const latest = msg.status_list[msg.status_list.length - 1];
      if (![4, 5, 6].includes(latest.status)) return;
      const key = `${latest.goal_info?.goal_id?.uuid?.join?.("-")}-${latest.status}`;
      if (lastGoalKeyRef.current === key) return;
      lastGoalKeyRef.current = key;
      if (latest.status === 4) accRef.current.goalsSucceeded += 1;
      else if (latest.status === 5) accRef.current.goalsCanceled += 1;
      else if (latest.status === 6) accRef.current.goalsFailed += 1;
      dirtyRef.current = true;
    });

    const dock = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.DOCK_TRIGGER_STATUS_TOPIC,
      messageType: "std_msgs/String",
    });
    let lastDock = null;
    dock.subscribe((msg) => {
      if (msg.data === lastDock) return;
      lastDock = msg.data;
      if (msg.data === "docked") accRef.current.dockSuccess += 1;
      else if (msg.data === "failed") accRef.current.dockFail += 1;
      else return;
      dirtyRef.current = true;
    });

    return () => {
      odom.unsubscribe();
      navStatus.unsubscribe();
      dock.unsubscribe();
    };
  }, [ros]);

  // Flush accumulator → state + localStorage at 1 Hz.
  useEffect(() => {
    const id = setInterval(() => {
      if (!dirtyRef.current) return;
      dirtyRef.current = false;
      const snapshot = { ...accRef.current };
      setMetrics(snapshot);
      try {
        localStorage.setItem(STORAGE_KEY, JSON.stringify(snapshot));
      } catch {
        // ignore quota errors
      }
    }, 1000);
    return () => clearInterval(id);
  }, []);

  const reset = () => {
    const fresh = emptyMetrics();
    accRef.current = fresh;
    lastPosRef.current = null;
    setMetrics(fresh);
    setSpeed(0);
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(fresh));
    } catch {
      // ignore
    }
  };

  return { metrics, speed, reset };
}
