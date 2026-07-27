import { useEffect, useRef } from "react";

import { useRos } from "../app/App";
import { AppConfig } from "../shared/constants";
import { addEvent } from "../shared/events/eventLog";

const NAV_TERMINAL = {
  4: { severity: "success", message: "Navigation goal succeeded" },
  5: { severity: "warning", message: "Navigation goal canceled" },
  6: { severity: "error", message: "Navigation goal failed" },
};

const LOW_BATTERY = 20; // percent; matches NotificationsWatcher's default band

/**
 * Headless, always-mounted (AppLayout) recorder that turns key robot events
 * into a persisted, reviewable timeline (see shared/events/eventLog.js).
 * Unlike NotificationsWatcher — which fires ephemeral desktop notifications
 * and is gated on an opt-in — this always records, so an operator can review
 * what happened after the fact. It writes to the shared event log; the
 * Events page reads it.
 */
const EventRecorder = () => {
  const ros = useRos();
  const lowBatteryArmedRef = useRef(true);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return undefined;

    const subs = [];
    const sub = (name, messageType, handler) => {
      const topic = new window.ROSLIB.Topic({ ros, name, messageType });
      topic.subscribe(handler);
      subs.push(topic);
    };

    sub(AppConfig.NAV_STATUS_TOPIC, "action_msgs/GoalStatusArray", (msg) => {
      if (!msg.status_list?.length) return;
      const latest = msg.status_list[msg.status_list.length - 1];
      const info = NAV_TERMINAL[latest.status];
      if (!info) return;
      const key = latest.goal_info?.goal_id?.uuid?.join?.("-") || latest.status;
      addEvent({
        type: "navigation",
        severity: info.severity,
        message: info.message,
        dedupeKey: `nav-${key}-${latest.status}`,
      });
    });

    sub(AppConfig.DOCK_TRIGGER_STATUS_TOPIC, "std_msgs/String", (msg) => {
      if (msg.data === "docked") {
        addEvent({ type: "docking", severity: "success", message: "Docking complete", dedupeKey: "dock-docked" });
      } else if (msg.data === "failed") {
        addEvent({ type: "docking", severity: "error", message: "Docking failed", dedupeKey: "dock-failed" });
      } else if (msg.data === "undocked") {
        addEvent({ type: "docking", severity: "info", message: "Undocked", dedupeKey: "dock-undocked" });
      }
    });

    sub(AppConfig.BATTERY_TOPIC, "std_msgs/Float32", ({ data }) => {
      if (data <= LOW_BATTERY && lowBatteryArmedRef.current) {
        lowBatteryArmedRef.current = false;
        addEvent({
          type: "battery",
          severity: "warning",
          message: `Battery low: ${Math.round(data)}%`,
        });
      } else if (data > LOW_BATTERY + 5) {
        lowBatteryArmedRef.current = true;
      }
    });

    return () => subs.forEach((t) => t.unsubscribe());
  }, [ros]);

  return null;
};

export default EventRecorder;
