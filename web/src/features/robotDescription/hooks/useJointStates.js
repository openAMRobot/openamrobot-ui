import { useCallback, useEffect, useRef, useState } from "react";

import { AppConfig } from "../../../shared/constants";

/**
 * Owns joint position values for the Robot Description page.
 *
 * Description Mode: values are local UI state driven by the sliders — never
 * published anywhere.
 *
 * Live Mode: values come from AppConfig.JOINT_STATES_TOPIC
 * (sensor_msgs/JointState) and are read-only. This robot has no per-joint
 * position command interface (only /cmd_vel, which drives both wheels
 * together), so there is no real command a slider could send in Live Mode —
 * setJointValue/resetJoints are no-ops while live, by design, not an
 * oversight. A future robot with real per-joint actuation topics could wire
 * a publisher in here without changing any component that consumes this
 * hook.
 */
export default function useJointStates({ mode, ros, jointNames }) {
  const [values, setValues] = useState({});
  const [liveConnected, setLiveConnected] = useState(false);
  const topicRef = useRef(null);

  useEffect(() => {
    setValues((prev) => {
      const next = {};
      jointNames.forEach((name) => {
        next[name] = name in prev ? prev[name] : 0;
      });
      return next;
    });
  }, [jointNames]);

  useEffect(() => {
    if (mode !== "live" || !ros || !window.ROSLIB) {
      setLiveConnected(false);
      return undefined;
    }

    const topic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.JOINT_STATES_TOPIC,
      messageType: "sensor_msgs/JointState",
    });
    topicRef.current = topic;

    topic.subscribe((msg) => {
      setLiveConnected(true);
      const names = msg.name || [];
      const positions = msg.position || [];
      setValues((prev) => {
        const next = { ...prev };
        names.forEach((name, index) => {
          if (name in next && typeof positions[index] === "number") {
            next[name] = positions[index];
          }
        });
        return next;
      });
    });

    return () => {
      topic.unsubscribe();
      topicRef.current = null;
      setLiveConnected(false);
    };
  }, [mode, ros]);

  const setJointValue = useCallback(
    (name, value) => {
      if (mode === "live") return;
      setValues((prev) => ({ ...prev, [name]: value }));
    },
    [mode],
  );

  const resetJoints = useCallback(() => {
    if (mode === "live") return;
    setValues((prev) => {
      const next = {};
      Object.keys(prev).forEach((name) => {
        next[name] = 0;
      });
      return next;
    });
  }, [mode]);

  return { values, setJointValue, resetJoints, liveConnected };
}
