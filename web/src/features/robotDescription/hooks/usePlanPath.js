import { useEffect, useState } from "react";

import { AppConfig } from "../../../shared/constants";

/**
 * Nav2's current planned path (nav_msgs/Path), Live Mode only. Poses are in
 * the map frame — the same frame useRobotPose's AMCL source uses — so both
 * can be drawn in the viewer's world space with no extra transform.
 */
export default function usePlanPath({ mode, ros }) {
  const [points, setPoints] = useState(null);

  useEffect(() => {
    if (mode !== "live" || !ros || !window.ROSLIB) {
      setPoints(null);
      return undefined;
    }

    const topic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.PLAN_TOPIC,
      messageType: "nav_msgs/Path",
    });
    topic.subscribe((msg) => {
      const poses = msg?.poses || [];
      setPoints(
        poses
          .map((p) => p?.pose?.position)
          .filter(Boolean)
          .map((pos) => ({ x: pos.x, y: pos.y })),
      );
    });

    return () => {
      topic.unsubscribe();
      setPoints(null);
    };
  }, [mode, ros]);

  return points;
}
