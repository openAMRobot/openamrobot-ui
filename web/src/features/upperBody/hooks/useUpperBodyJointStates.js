import { useEffect, useRef, useState } from "react";

import { AppConfig } from "../../../shared/constants";
import { JOINT_GROUPS } from "../upperBodyModel";
import { createFreshnessTracker, readJointState } from "../jointGroupFreshness";

const TICK_MS = 250;
const UPPER_BODY_JOINTS = new Set(JOINT_GROUPS.flatMap((g) => g.joints));

const snapshot = (tracker, t) => {
  const next = {};
  JOINT_GROUPS.forEach((g) => {
    next[g.id] = tracker.status(g.id, t);
  });
  return next;
};

/**
 * Reported upper-body joint positions from /joint_states, with freshness
 * tracked per joint from the arrival rate. Only finite positions are kept; a
 * NaN, Infinity or unmatched position removes that joint's value. A new ROS
 * connection starts from nothing: old positions and sampling history are
 * dropped, and messages from the previous connection are ignored.
 * Subscribe only: this hook never advertises, publishes or calls anything.
 */
export default function useUpperBodyJointStates({ ros, now = () => Date.now() }) {
  const [positions, setPositions] = useState({});
  const [freshness, setFreshness] = useState({});
  const trackerRef = useRef(createFreshnessTracker(JOINT_GROUPS));
  const nowRef = useRef(now);
  nowRef.current = now;

  useEffect(() => {
    const tracker = createFreshnessTracker(JOINT_GROUPS);
    trackerRef.current = tracker;
    setPositions({});
    setFreshness(snapshot(tracker, nowRef.current()));
    if (!ros || !window.ROSLIB) return undefined;

    let active = true;
    const topic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.JOINT_STATES_TOPIC,
      messageType: "sensor_msgs/JointState",
    });
    topic.subscribe((msg) => {
      if (!active) return;
      const { valid, invalid } = readJointState(msg);
      const ownValid = {};
      Object.keys(valid).forEach((name) => {
        if (UPPER_BODY_JOINTS.has(name)) ownValid[name] = valid[name];
      });
      const ownInvalid = invalid.filter((name) => UPPER_BODY_JOINTS.has(name));
      if (!Object.keys(ownValid).length && !ownInvalid.length) return;
      tracker.record({ valid: ownValid, invalid: ownInvalid }, nowRef.current());
      setPositions((prev) => {
        const next = { ...prev, ...ownValid };
        ownInvalid.forEach((name) => delete next[name]);
        return next;
      });
    });
    return () => {
      active = false;
      topic.unsubscribe();
    };
  }, [ros]);

  useEffect(() => {
    const tick = () => setFreshness(snapshot(trackerRef.current, nowRef.current()));
    const id = setInterval(tick, TICK_MS);
    return () => clearInterval(id);
  }, []);

  return { positions, freshness };
}
