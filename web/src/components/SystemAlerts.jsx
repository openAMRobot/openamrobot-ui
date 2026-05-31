import React, { useContext, useEffect, useRef, useState } from "react";
import { RosContext, RosStatusContext } from "../app/App";
import { AppConfig } from "../shared/constants";

const WATCHED = [
  {
    key: "map",
    label: "Map not received",
    topic: AppConfig.MAP_TOPIC,
    type: "nav_msgs/OccupancyGrid",
    timeout: 12000,
  },
  {
    key: "amcl",
    label: "Localization pose missing",
    topic: AppConfig.AMCL_POSE_TOPIC,
    type: "geometry_msgs/PoseWithCovarianceStamped",
    timeout: 8000,
  },
  {
    key: "plan",
    label: "No planned path yet",
    topic: AppConfig.PLAN_TOPIC,
    type: "nav_msgs/Path",
    timeout: 20000,
    warningOnly: true,
  },
];

const SystemAlerts = () => {
  const ros = useContext(RosContext);
  const rosbridgeStatus = useContext(RosStatusContext);
  const seen = useRef({});
  const [alerts, setAlerts] = useState([]);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;
    const subs = WATCHED.map(({ key, topic, type }) => {
      const sub = new window.ROSLIB.Topic({ ros, name: topic, messageType: type });
      sub.subscribe(() => {
        seen.current[key] = Date.now();
      });
      return sub;
    });
    return () => subs.forEach((sub) => sub.unsubscribe());
  }, [ros]);

  useEffect(() => {
    const id = setInterval(() => {
      const now = Date.now();
      const next = [];
      if (rosbridgeStatus !== "connected") {
        next.push({ label: "Rosbridge disconnected", severity: "error" });
      }
      WATCHED.forEach((item) => {
        const last = seen.current[item.key];
        if (!last || now - last > item.timeout) {
          next.push({
            label: item.label,
            severity: item.warningOnly ? "warn" : "error",
          });
        }
      });
      setAlerts(next.slice(0, 4));
    }, 1000);
    return () => clearInterval(id);
  }, [rosbridgeStatus]);

  if (!alerts.length) return null;

  const hasError = alerts.some((alert) => alert.severity === "error");

  return (
    <div
      className={`flex min-h-[30px] items-center gap-2 overflow-hidden rounded-xl border px-3 py-1 font-[RobotoMono] text-xs ${
        hasError
          ? "border-statusRed bg-statusRed/10 text-statusRed"
          : "border-statusYellow bg-statusYellow/10 text-statusYellow"
      }`}
    >
      <span className="shrink-0 font-semibold uppercase tracking-wider">
        {hasError ? "Alerts" : "Notice"}
      </span>
      <div className="flex min-w-0 flex-1 items-center gap-2 overflow-hidden">
        {alerts.map((alert, index) => (
          <React.Fragment key={alert.label}>
            {index > 0 && <span className="shrink-0 opacity-50">/</span>}
            <span className="truncate">{alert.label}</span>
          </React.Fragment>
        ))}
      </div>
    </div>
  );
};

export default SystemAlerts;
