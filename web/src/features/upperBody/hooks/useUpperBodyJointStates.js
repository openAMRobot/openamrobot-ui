import { useEffect, useRef, useState } from "react";

import { AppConfig } from "../../../shared/constants";
import { JOINT_GROUPS } from "../upperBodyModel";
import { createFreshnessTracker } from "../jointGroupFreshness";

const TICK_MS = 250;
const UPPER_BODY_JOINTS = new Set(JOINT_GROUPS.flatMap((g) => g.joints));

/**
 * Reported upper-body joint positions from /joint_states, with per-group
 * freshness measured from the arrival rate. Subscribe only: this hook never
 * advertises, publishes or calls anything, in any mode.
 */
export default function useUpperBodyJointStates({ ros, now = () => Date.now() }) {
  const [positions, setPositions] = useState({});
  const [freshness, setFreshness] = useState({});
  const trackerRef = useRef(createFreshnessTracker(JOINT_GROUPS));
  const nowRef = useRef(now);
  nowRef.current = now;

  useEffect(() => {
    if (!ros || !window.ROSLIB) return undefined;
    const topic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.JOINT_STATES_TOPIC,
      messageType: "sensor_msgs/JointState",
    });
    topic.subscribe((msg) => {
      const names = msg?.name || [];
      const values = msg?.position || [];
      trackerRef.current.record(names, nowRef.current());
      const update = {};
      names.forEach((name, i) => {
        if (UPPER_BODY_JOINTS.has(name) && typeof values[i] === "number") update[name] = values[i];
      });
      if (Object.keys(update).length) setPositions((prev) => ({ ...prev, ...update }));
    });
    return () => {
      topic.unsubscribe();
    };
  }, [ros]);

  useEffect(() => {
    const tick = () => {
      const t = nowRef.current();
      const next = {};
      JOINT_GROUPS.forEach((g) => {
        next[g.id] = trackerRef.current.status(g.id, t);
      });
      setFreshness(next);
    };
    tick();
    const id = setInterval(tick, TICK_MS);
    return () => clearInterval(id);
  }, []);

  return { positions, freshness };
}
