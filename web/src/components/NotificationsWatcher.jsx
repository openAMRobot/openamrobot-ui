import { useEffect, useRef } from "react";

import { useRos, useRuntimeConfig } from "../app/App";
import { AppConfig } from "../shared/constants";

const NAV_TERMINAL_LABELS = {
  4: "Navigation succeeded",
  5: "Navigation canceled",
  6: "Navigation failed",
};

const notify = (title, body) => {
  if (typeof Notification === "undefined" || Notification.permission !== "granted") return;
  try {
    // Fire-and-forget — nothing needs to hold a reference to the instance.
    const notification = new Notification(title, { body });
    return notification;
  } catch {
    // Some platforms (e.g. Android Chrome) only allow notifications via a
    // service worker and throw on the plain constructor — non-fatal, just skip.
  }
};

/**
 * Headless, always-mounted (in AppLayout) watcher that pings the operator
 * via the browser Notification API on key events — nav goal completion,
 * docking finishing/failing, and battery dropping below a threshold — so
 * they don't have to keep this tab focused on any particular page. Gated
 * on config.notificationsEnabled (Config page), off by default.
 */
const NotificationsWatcher = () => {
  const ros = useRos();
  const { config } = useRuntimeConfig();
  const lastNavTerminalRef = useRef(null);
  const lowBatteryNotifiedRef = useRef(false);

  useEffect(() => {
    if (!config.notificationsEnabled || !ros || !window.ROSLIB) return;

    const navStatusTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.NAV_STATUS_TOPIC,
      messageType: "action_msgs/GoalStatusArray",
    });
    navStatusTopic.subscribe((msg) => {
      if (!msg.status_list?.length) return;
      const latest = msg.status_list[msg.status_list.length - 1];
      if (![4, 5, 6].includes(latest.status)) return;
      const key = latest.goal_info?.goal_id?.uuid?.join?.("-") || latest.status;
      if (lastNavTerminalRef.current === key) return;
      lastNavTerminalRef.current = key;
      notify("OpenAMR", NAV_TERMINAL_LABELS[latest.status]);
    });

    const dockStatusTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.DOCK_TRIGGER_STATUS_TOPIC,
      messageType: "std_msgs/String",
    });
    dockStatusTopic.subscribe((msg) => {
      if (msg.data === "docked") notify("OpenAMR", "Docking complete");
      else if (msg.data === "failed") notify("OpenAMR", "Docking failed — check the dock tag and logs");
    });

    const batteryTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.BATTERY_TOPIC,
      messageType: "std_msgs/Float32",
    });
    batteryTopic.subscribe(({ data }) => {
      const threshold = config.lowBatteryThreshold;
      if (data <= threshold) {
        if (!lowBatteryNotifiedRef.current) {
          lowBatteryNotifiedRef.current = true;
          notify("OpenAMR", `Battery at ${Math.round(data)}% — below ${threshold}%`);
        }
      } else if (data > threshold + 5) {
        // Hysteresis: only re-arm once it's recovered a few points past the
        // threshold, so it doesn't re-fire every message while hovering at it.
        lowBatteryNotifiedRef.current = false;
      }
    });

    return () => {
      navStatusTopic.unsubscribe();
      dockStatusTopic.unsubscribe();
      batteryTopic.unsubscribe();
    };
  }, [ros, config.notificationsEnabled, config.lowBatteryThreshold]);

  return null;
};

export default NotificationsWatcher;
