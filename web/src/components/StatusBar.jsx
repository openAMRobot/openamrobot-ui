import React, { useCallback, useEffect, useRef, useState } from "react";

import { useRos, useRosStatus } from "../app/App";
import { AppConfig } from "../shared/constants";
import { addEvent } from "../shared/events/eventLog";
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
 * connection, battery level, and a one-tap software E-stop. The E-stop
 * publishes a zero Twist and cancels the active Nav2 goal — the same halt the
 * Map page's stop button does — so it works no matter which page is open.
 */
const StatusBar = () => {
  const ros = useRos();
  const rosStatus = useRosStatus();
  const { t } = useT();
  const [battery, setBattery] = useState(null);

  const cmdVelRef = useRef(null);
  const cancelRef = useRef(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return undefined;

    cmdVelRef.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.CMD_VEL_TOPIC,
      messageType: "geometry_msgs/Twist",
    });
    cancelRef.current = new window.ROSLIB.Service({
      ros,
      name: AppConfig.NAV_CANCEL_GOAL_SERVICE,
      serviceType: "action_msgs/CancelGoal",
    });

    const batteryTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.BATTERY_TOPIC,
      messageType: "std_msgs/Float32",
    });
    batteryTopic.subscribe(({ data }) => setBattery(data));

    return () => batteryTopic.unsubscribe();
  }, [ros]);

  const emergencyStop = useCallback(() => {
    cmdVelRef.current?.publish(
      new window.ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      }),
    );
    cancelRef.current?.callService(
      new window.ROSLIB.ServiceRequest({
        goal_info: { goal_id: { uuid: new Array(16).fill(0) }, stamp: { sec: 0, nanosec: 0 } },
      }),
      () => {},
      () => {},
    );
    addEvent({ type: "safety", severity: "error", message: "Emergency stop (status bar)" });
  }, []);

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
        onClick={emergencyStop}
        title="Emergency stop — halt the robot"
        className="ml-auto rounded-lg border-2 border-statusRed bg-statusRed/10 px-3 py-1 text-xs font-bold text-statusRed transition-colors hover:bg-statusRed hover:text-white"
      >
        {t("E-STOP")}
      </button>
    </div>
  );
};

export default StatusBar;
