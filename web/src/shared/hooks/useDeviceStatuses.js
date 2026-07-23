import { useEffect, useRef, useState } from "react";

const TIMEOUT_MS = 6000;

/**
 * Live status for registered devices, derived the same way
 * SystemHealth.jsx derives topic health: "online" if a message on the
 * device's configured status topic arrived within TIMEOUT_MS, "offline"
 * once it goes quiet. Devices with no status topic configured (most manual
 * registrations, at least at first) report "unmonitored" rather than a
 * guessed online/offline — there's nothing to actually check.
 */
export default function useDeviceStatuses(ros, devices) {
  const lastSeen = useRef({});
  const [statuses, setStatuses] = useState({});

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    const monitored = devices.filter((device) => device.statusTopic?.trim());
    const subs = monitored.map((device) => {
      const topic = new window.ROSLIB.Topic({
        ros,
        name: device.statusTopic.trim(),
        messageType: device.statusMessageType?.trim() || "std_msgs/String",
        queue_length: 1,
      });
      topic.subscribe(() => {
        lastSeen.current[device.id] = Date.now();
      });
      return topic;
    });

    return () => subs.forEach((topic) => topic.unsubscribe());
  }, [ros, devices]);

  useEffect(() => {
    const id = setInterval(() => {
      const now = Date.now();
      setStatuses(
        Object.fromEntries(
          devices.map((device) => {
            if (!device.statusTopic?.trim()) return [device.id, "unmonitored"];
            const seen = lastSeen.current[device.id];
            return [
              device.id,
              seen && now - seen < TIMEOUT_MS ? "online" : "offline",
            ];
          }),
        ),
      );
    }, 1000);
    return () => clearInterval(id);
  }, [devices]);

  return statuses;
}
