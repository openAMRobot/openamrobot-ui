import { useEffect, useRef, useState } from "react";

import { AppConfig } from "../../../shared/constants";

const quatToYaw = (q) =>
  Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

/**
 * Robot's real-world pose in the map frame, Live Mode only.
 *
 * Mirrors the AMCL/odom source split already used by RobotState.jsx: AMCL
 * is the map-corrected pose but only publishes on localization updates,
 * while odom is continuous but frame-local — so odom is used only until
 * the first AMCL pose arrives, then AMCL takes over as source of truth.
 */
export default function useRobotPose({ mode, ros }) {
  const [pose, setPose] = useState(null);
  const [source, setSource] = useState(null);
  const amclActiveRef = useRef(false);

  useEffect(() => {
    amclActiveRef.current = false;
    if (mode !== "live" || !ros || !window.ROSLIB) {
      setPose(null);
      setSource(null);
      return undefined;
    }

    const amclTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.AMCL_POSE_TOPIC,
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });
    amclTopic.subscribe((msg) => {
      const pos = msg?.pose?.pose?.position;
      const ori = msg?.pose?.pose?.orientation;
      if (!pos || !ori) return;
      amclActiveRef.current = true;
      setSource("amcl");
      setPose({ x: pos.x, y: pos.y, yaw: quatToYaw(ori) });
    });

    const odomTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.ROBOT_POSE_TOPIC,
      messageType: "nav_msgs/Odometry",
    });
    odomTopic.subscribe((msg) => {
      if (amclActiveRef.current) return;
      const pos = msg?.pose?.pose?.position;
      const ori = msg?.pose?.pose?.orientation;
      if (!pos || !ori) return;
      setSource("odom");
      setPose({ x: pos.x, y: pos.y, yaw: quatToYaw(ori) });
    });

    return () => {
      amclTopic.unsubscribe();
      odomTopic.unsubscribe();
      setPose(null);
      setSource(null);
    };
  }, [mode, ros]);

  return { pose, source };
}
