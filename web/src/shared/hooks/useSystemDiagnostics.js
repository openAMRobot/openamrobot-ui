import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import { useRos, useRosStatus, useRuntimeConfig } from "../../app/App";
import { AppConfig, LIFECYCLE_NODES } from "../constants";
import { fetchRobotDescriptionManifest } from "../../features/robotDescription/api/robotDescriptionApi";
import useDevices from "./useDevices";
import useDeviceStatuses from "./useDeviceStatuses";

// Topics this app's own pages already depend on being alive — a reasonable,
// honest definition of "expected", since it's exactly what SystemHealth,
// InfoPage, and the Robot Description page already assume is publishing.
const EXPECTED_TOPICS = [
  { topic: AppConfig.SCAN_TOPIC, label: "Laser scan" },
  { topic: AppConfig.ROBOT_POSE_TOPIC, label: "Odometry" },
  { topic: AppConfig.MAP_TOPIC, label: "Map" },
  { topic: AppConfig.AMCL_POSE_TOPIC, label: "AMCL pose" },
  { topic: AppConfig.NAV_STATUS_TOPIC, label: "Nav status" },
  { topic: AppConfig.BATTERY_TOPIC, label: "Battery" },
  { topic: AppConfig.JOINT_STATES_TOPIC, label: "Joint states" },
];

const CRITICAL_TOPIC_KEYS = new Set(["odom", "scan", "map", "amcl"]);

const DIAGNOSTIC_LEVEL_LABEL = { 1: "WARN", 2: "ERROR", 3: "STALE" };

export const OVERALL_LABELS = {
  0: "Ready",
  1: "Ready with warnings",
  2: "Partially connected",
  3: "Not ready",
};

const RECONNECT_TOPICS_INTERVAL_MS = 20000;
const FAULT_LOG_LIMIT = 30;

/**
 * Aggregates every health signal this app already computes elsewhere
 * (SystemHealth's topic/TF checks, LifecycleStatus's nav2 states, battery,
 * registered-device status, URDF availability, /diagnostics, and a
 * best-effort rosapi topic-graph check) into one overall Ready / Ready with
 * warnings / Partially connected / Not ready rollup, plus a list of the
 * specific issues driving that rollup so the Health Centre page can link
 * each one to where it can actually be fixed.
 *
 * SystemHealth/LifecycleStatus own their own ROS subscriptions already —
 * this hook doesn't re-subscribe to the same topics/services a second time,
 * it just receives their computed state via the reportHealth/reportLifecycle
 * callbacks the Health Centre page wires up.
 */
export default function useSystemDiagnostics() {
  const ros = useRos();
  const rosbridgeStatus = useRosStatus();
  const { config } = useRuntimeConfig();
  const { devices } = useDevices();
  const deviceStatuses = useDeviceStatuses(ros, devices);

  const [health, setHealth] = useState({});
  const [tfLinks, setTfLinks] = useState({});
  const [lifecycle, setLifecycle] = useState({});
  const [battery, setBattery] = useState({ pct: null, charging: false });
  const [urdf, setUrdf] = useState({ checked: false, available: null });
  const [diagnosticsMsgs, setDiagnosticsMsgs] = useState([]);
  const [missingTopics, setMissingTopics] = useState([]);
  const [faultLog, setFaultLog] = useState([]);

  const reportHealth = useCallback(({ health: h, tfLinks: tl }) => {
    setHealth(h);
    setTfLinks(tl);
  }, []);

  const reportLifecycle = useCallback((states) => {
    setLifecycle(states);
  }, []);

  // Battery + charging — same topics InfoPage already subscribes to.
  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    const batteryTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.BATTERY_TOPIC,
      messageType: "std_msgs/Float32",
    });
    const chargeTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.CHARGE_STATION_CONNECTED,
      messageType: "std_msgs/Bool",
    });

    batteryTopic.subscribe(({ data }) =>
      setBattery((prev) => ({ ...prev, pct: Math.round(data) })),
    );
    chargeTopic.subscribe(({ data }) =>
      setBattery((prev) => ({ ...prev, charging: data })),
    );

    return () => {
      batteryTopic.unsubscribe();
      chargeTopic.unsubscribe();
    };
  }, [ros]);

  // Standard ROS2 diagnostics aggregator, if anything in the stack publishes
  // to it — best-effort, absence just means "no diagnostic_updater sources",
  // not an error.
  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    const topic = new window.ROSLIB.Topic({
      ros,
      name: "/diagnostics",
      messageType: "diagnostic_msgs/DiagnosticArray",
      queue_length: 1,
    });
    topic.subscribe((msg) => {
      setDiagnosticsMsgs(
        (msg?.status || [])
          .filter((entry) => entry.level >= 1)
          .map((entry) => ({
            name: entry.name,
            message: entry.message,
            level: entry.level,
          })),
      );
    });

    return () => topic.unsubscribe();
  }, [ros]);

  // Robot description availability — one-shot HTTP check, independent of
  // rosbridge (it's served by Flask directly).
  useEffect(() => {
    let cancelled = false;
    fetchRobotDescriptionManifest()
      .then((data) => {
        if (!cancelled) setUrdf({ checked: true, available: Boolean(data.available) });
      })
      .catch(() => {
        if (!cancelled) setUrdf({ checked: true, available: false });
      });
    return () => {
      cancelled = true;
    };
  }, []);

  // Best-effort rosapi check for whether this app's expected topics are
  // currently in the ROS graph at all. If rosapi itself isn't reachable,
  // this just quietly reports nothing missing rather than treating that as
  // a fault of its own — it's a bonus check, not a required one.
  useEffect(() => {
    if (!ros || rosbridgeStatus !== "connected") {
      setMissingTopics([]);
      return;
    }

    let cancelled = false;
    const check = () => {
      ros.getTopics(
        (result) => {
          if (cancelled) return;
          const present = new Set(result?.topics || []);
          setMissingTopics(
            EXPECTED_TOPICS.filter(({ topic }) => !present.has(topic)),
          );
        },
        () => {
          if (!cancelled) setMissingTopics([]);
        },
      );
    };

    check();
    const id = setInterval(check, RECONNECT_TOPICS_INTERVAL_MS);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, [ros, rosbridgeStatus]);

  const issues = useMemo(() => {
    const list = [];

    if (rosbridgeStatus !== "connected") {
      list.push({
        id: "rosbridge",
        severity: 3,
        message: "Rosbridge is disconnected — nothing else here can be verified.",
        linkTo: "/config",
      });
    }

    if (tfLinks && Object.keys(tfLinks).length && health.tfChain === "offline") {
      list.push({
        id: "tf-chain",
        severity: 2,
        message: "TF chain is broken (map → odom → base_link → lidar_link) — localization and navigation can't work without it.",
        linkTo: "/info",
      });
    }

    Object.entries(health).forEach(([key, state]) => {
      if (key === "tfChain" || state !== "offline") return;
      list.push({
        id: `topic-${key}`,
        severity: CRITICAL_TOPIC_KEYS.has(key) ? 2 : 1,
        message: `${key} topic has gone silent.`,
        linkTo: "/info",
      });
    });

    Object.entries(lifecycle).forEach(([name, state]) => {
      if (state === "active") return;
      list.push({
        id: `lifecycle-${name}`,
        severity: state === "inactive" ? 1 : 2,
        message: `${name} lifecycle node is ${state}.`,
        linkTo: "/health",
      });
    });

    if (battery.pct !== null && !battery.charging) {
      if (battery.pct <= 10) {
        list.push({
          id: "battery",
          severity: 2,
          message: `Battery critically low (${battery.pct}%).`,
          linkTo: "/info",
        });
      } else if (battery.pct <= (config.lowBatteryThreshold ?? 20)) {
        list.push({
          id: "battery",
          severity: 1,
          message: `Battery low (${battery.pct}%).`,
          linkTo: "/info",
        });
      }
    }

    devices.forEach((device) => {
      if (deviceStatuses[device.id] === "offline") {
        list.push({
          id: `device-${device.id}`,
          severity: 1,
          message: `${device.name} is offline.`,
          linkTo: "/devices",
        });
      }
    });

    if (urdf.checked && urdf.available === false) {
      list.push({
        id: "urdf",
        severity: 1,
        message: "Robot description (URDF) isn't available on this install.",
        linkTo: "/robot",
      });
    }

    diagnosticsMsgs.forEach((entry, index) => {
      list.push({
        id: `diagnostic-${index}-${entry.name}`,
        severity: entry.level >= 2 ? 2 : 1,
        message: `${entry.name}: ${entry.message} (${DIAGNOSTIC_LEVEL_LABEL[entry.level] || entry.level})`,
        linkTo: null,
      });
    });

    missingTopics.forEach(({ topic, label }) => {
      list.push({
        id: `missing-${topic}`,
        severity: 1,
        message: `${label} (${topic}) isn't currently in the ROS graph.`,
        linkTo: "/info",
      });
    });

    return list.sort((a, b) => b.severity - a.severity);
  }, [
    rosbridgeStatus,
    health,
    tfLinks,
    lifecycle,
    battery,
    config.lowBatteryThreshold,
    devices,
    deviceStatuses,
    urdf,
    diagnosticsMsgs,
    missingTopics,
  ]);

  const overall = issues.length ? Math.max(...issues.map((i) => i.severity)) : 0;

  // Recent faults: log an entry the first time an issue appears that wasn't
  // present a moment ago — a session-only record of "what newly went wrong
  // and when", not a persisted fault database.
  const knownIssueIds = useRef(new Set());
  useEffect(() => {
    const currentIds = new Set(issues.map((i) => i.id));
    const newOnes = issues.filter((i) => !knownIssueIds.current.has(i.id));
    knownIssueIds.current = currentIds;
    if (newOnes.length === 0) return;

    setFaultLog((prev) =>
      [
        ...newOnes.map((issue) => ({
          time: Date.now(),
          message: issue.message,
          severity: issue.severity,
        })),
        ...prev,
      ].slice(0, FAULT_LOG_LIMIT),
    );
  }, [issues]);

  return {
    reportHealth,
    reportLifecycle,
    battery,
    urdf,
    diagnosticsMsgs,
    missingTopics,
    devices,
    deviceStatuses,
    issues,
    overall,
    overallLabel: OVERALL_LABELS[overall],
    faultLog,
  };
}
