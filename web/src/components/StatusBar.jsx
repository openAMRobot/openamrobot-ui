import React, { useCallback, useEffect, useState } from "react";

import { useRos, useRosStatus } from "../app/App";
import { AppConfig } from "../shared/constants";
import { softwareStop } from "../shared/motion/softwareStop";
import { useT } from "../shared/i18n/i18n";

const CONN = {
  connected: { label: "Connected", color: "text-statusGreen", dot: "bg-statusGreen" },
  disconnected: { label: "Offline", color: "text-statusRed", dot: "bg-statusRed" },
  error: { label: "Error", color: "text-statusRed", dot: "bg-statusRed" },
};

const batteryColor = (pct) =>
  pct == null
    ? "text-themeTextGray"
    : pct <= 20
      ? "text-statusRed"
      : pct <= 40
        ? "text-statusYellow"
        : "text-statusGreen";

/**
 * Always-visible bar (mounted in AppLayout, above the routed page) surfacing
 * the three things an operator needs at a glance from anywhere: rosbridge
 * connection, battery level, and a one-tap "Stop (software)" button.
 *
 * Stop (software) engages the platform stop lock, cancels every active Nav2
 * and docking goal and aborts UI programs and missions (shared/motion/
 * softwareStop.js), the same call the Map page's stop button makes, so it
 * works no matter which page is open. It is a software stop only. It is NOT
 * the hardwired E-stop and must never be labelled as one: the hardwired
 * E-stop stays the safety function.
 */
const StatusBar = () => {
  const ros = useRos();
  const rosStatus = useRosStatus();
  const { t } = useT();
  const [battery, setBattery] = useState(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return undefined;

    const batteryTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.BATTERY_TOPIC,
      messageType: "std_msgs/Float32",
    });
    batteryTopic.subscribe(({ data }) => setBattery(data));

    return () => batteryTopic.unsubscribe();
  }, [ros]);

  const onSoftwareStop = useCallback(() => {
    softwareStop(ros, "status bar");
  }, [ros]);

  const conn = CONN[rosStatus] || CONN.disconnected;
  const pct = battery == null ? null : Math.round(battery);

  return (
    <div className="sticky top-0 z-40 flex items-center gap-4 border-b border-borderSubtle bg-bgCard/90 px-3 py-1.5 font-[RobotoMono] backdrop-blur sm:px-4">
      <div className="flex items-center gap-2">
        <span className={`h-2 w-2 rounded-full ${conn.dot} ${rosStatus === "connected" ? "" : "animate-pulse"}`} />
        <span className={`text-xs ${conn.color}`}>{t(conn.label)}</span>
      </div>

      <div className="flex items-center gap-2">
        <span className="text-[10px] uppercase tracking-wider text-themeTextGray">{t("Batt")}</span>
        <span className={`text-xs font-semibold ${batteryColor(pct)}`}>
          {pct == null ? "—" : `${pct}%`}
        </span>
        <div className="hidden h-2 w-16 overflow-hidden rounded-full bg-bgSurface sm:block">
          <div
            className={`h-full rounded-full ${
              pct == null
                ? "bg-themeTextGray"
                : pct <= 20
                  ? "bg-statusRed"
                  : pct <= 40
                    ? "bg-statusYellow"
                    : "bg-statusGreen"
            }`}
            style={{ width: `${pct == null ? 0 : Math.max(0, Math.min(100, pct))}%` }}
          />
        </div>
      </div>

      <button
        onClick={onSoftwareStop}
        title="Stop (software): engages the platform stop lock and cancels active goals. This is not the hardwired E-stop."
        className="ml-auto rounded-lg border-2 border-statusRed bg-statusRed/10 px-3 py-1 text-xs font-bold text-statusRed transition-colors hover:bg-statusRed hover:text-white"
      >
        {t("Stop (software)")}
      </button>
    </div>
  );
};

export default StatusBar;
