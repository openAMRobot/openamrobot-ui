import { useEffect, useState } from "react";

import { AppConfig } from "../../../shared/constants";

/**
 * Subscribes (read-only) to the robot_description that robot_state_publisher
 * publishes. Returns the latest URDF text or null. Nothing is advertised,
 * published or called.
 */
export default function useRobotDescriptionTopic({ ros }) {
  const [urdfText, setUrdfText] = useState(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) {
      setUrdfText(null);
      return undefined;
    }
    const topic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.ROBOT_DESCRIPTION_TOPIC,
      messageType: "std_msgs/String",
    });
    topic.subscribe((msg) => {
      if (typeof msg?.data === "string") {
        setUrdfText((prev) => (prev === msg.data ? prev : msg.data));
      }
    });
    return () => {
      topic.unsubscribe();
    };
  }, [ros]);

  return urdfText;
}
