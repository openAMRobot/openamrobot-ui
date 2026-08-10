import React, { useEffect, useRef, useState } from "react";
import { useRos, useRosStatus } from "../app/App";
import { AppConfig } from "../shared/constants";

// Streaming topics: health = message received within timeout ms
const STREAMING = [
  {
    key: "scan",
    label: "Laser Scan",
    topic: AppConfig.SCAN_TOPIC,
    type: "sensor_msgs/LaserScan",
    timeout: 3000,
  },
  {
    key: "odom",
    label: "Odometry",
    topic: AppConfig.ROBOT_POSE_TOPIC,
    type: "nav_msgs/Odometry",
    timeout: 2000,
  },
  {
    key: "amcl",
    label: "Localization",
    topic: AppConfig.AMCL_POSE_TOPIC,
    type: "geometry_msgs/PoseWithCovarianceStamped",
    timeout: 8000,
  },
  {
    key: "nav2",
    label: "Nav2",
    topic: AppConfig.NAV_STATUS_TOPIC,
    type: "action_msgs/GoalStatusArray",
    timeout: 8000,
  },
  {
    key: "map",
    label: "Map",
    topic: AppConfig.MAP_TOPIC,
    type: "nav_msgs/OccupancyGrid",
    timeout: 15000,
  },
  {
    key: "costmap",
    label: "Global Costmap",
    topic: AppConfig.GLOBAL_COSTMAP_TOPIC,
    type: "nav_msgs/OccupancyGrid",
    timeout: 15000,
  },
  {
    key: "plan",
    label: "Plan",
    topic: AppConfig.PLAN_TOPIC,
    type: "nav_msgs/Path",
    timeout: 15000,
  },
];

const TF_LINKS = [
  ["map", "odom"],
  ["odom", "base_link"],
  ["base_link", "lidar_link"],
];

// Plain-language names for each position-tracking link, keyed the same way
// as TF_LINKS above. The raw frame names stay available as a hover tooltip.
const FRIENDLY_TF_LABELS = {
  "map->odom": "Map alignment",
  "odom->base_link": "Motion tracking",
  "base_link->lidar_link": "Sensor mounting",
};

const DOT = {
  online: "bg-statusGreen",
  offline: "bg-statusRed",
  unknown: "bg-themeTextGray",
};
const TEXT = {
  online: "text-statusGreen",
  offline: "text-statusRed",
  unknown: "text-themeTextGray",
};

const normalizeFrame = (frame) => (frame || "").replace(/^\/+/, "");
const hasTfEdge = (edges, from, to) =>
  edges.has(`${from}->${to}`) || edges.has(`${to}->${from}`);

const SystemHealth = ({ compact = false, onHealthChange }) => {
  const ros = useRos();
  const rosbridgeStatus = useRosStatus();
  const topicStats = useRef({});
  const tfEdges = useRef(new Set());
  const onHealthChangeRef = useRef(onHealthChange);
  onHealthChangeRef.current = onHealthChange;
  const [health, setHealth] = useState(() =>
    Object.fromEntries([
      ...STREAMING.map(({ key }) => [key, "unknown"]),
      ["tfChain", "unknown"],
    ]),
  );
  const [stats, setStats] = useState({});
  const [tfLinks, setTfLinks] = useState(() =>
    Object.fromEntries(
      TF_LINKS.map(([from, to]) => [`${from}->${to}`, "unknown"]),
    ),
  );

  // Subscribe to streaming topics and stamp lastSeen on each message
  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    const subs = STREAMING.map(({ key, topic, type }) => {
      const t = new window.ROSLIB.Topic({
        ros,
        name: topic,
        messageType: type,
        throttle_rate:
          key === "scan"
            ? 1000
            : key === "costmap" || key === "map"
            ? 3000
            : 500,
        queue_length: 1,
      });
      t.subscribe(() => {
        const now = Date.now();
        const prev = topicStats.current[key] || {
          count: 0,
          windowStart: now,
          hz: 0,
        };
        const elapsed = now - prev.windowStart;
        const nextCount = prev.count + 1;
        topicStats.current[key] = {
          last: now,
          count: elapsed > 3000 ? 1 : nextCount,
          windowStart: elapsed > 3000 ? now : prev.windowStart,
          hz:
            elapsed > 3000
              ? prev.hz
              : nextCount / Math.max(elapsed / 1000, 0.1),
        };
      });
      return t;
    });

    return () => subs.forEach((t) => t.unsubscribe());
  }, [ros]);

  // Track the frame chain needed by Nav2 and the map view.
  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    const updateTf = (msg) => {
      (msg?.transforms || []).forEach((tf) => {
        const parent = normalizeFrame(tf.header?.frame_id);
        const child = normalizeFrame(tf.child_frame_id);
        if (parent && child) tfEdges.current.add(`${parent}->${child}`);
      });
    };

    const tfTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.TF_TOPIC,
      messageType: "tf2_msgs/TFMessage",
      throttle_rate: 1000,
      queue_length: 1,
    });
    const tfStaticTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.TF_STATIC_TOPIC,
      messageType: "tf2_msgs/TFMessage",
      throttle_rate: 1000,
      queue_length: 1,
    });

    tfTopic.subscribe(updateTf);
    tfStaticTopic.subscribe(updateTf);

    return () => {
      tfTopic.unsubscribe();
      tfStaticTopic.unsubscribe();
    };
  }, [ros]);

  // Every second: evaluate streaming health from timestamps
  useEffect(() => {
    const id = setInterval(() => {
      const now = Date.now();
      setHealth((prev) => {
        const next = { ...prev };
        STREAMING.forEach(({ key, timeout }) => {
          const last = topicStats.current[key]?.last;
          next[key] = last
            ? now - last < timeout
              ? "online"
              : "offline"
            : "unknown";
        });
        const edges = tfEdges.current;
        const nextLinks = Object.fromEntries(
          TF_LINKS.map(([from, to]) => [
            `${from}->${to}`,
            hasTfEdge(edges, from, to)
              ? "online"
              : edges.size
              ? "offline"
              : "unknown",
          ]),
        );
        setTfLinks(nextLinks);
        next.tfChain = Object.values(nextLinks).every(
          (state) => state === "online",
        )
          ? "online"
          : edges.size
          ? "offline"
          : "unknown";
        const nextStats = Object.fromEntries(
          STREAMING.map(({ key }) => {
            const topic = topicStats.current[key];
            return [
              key,
              {
                age: topic?.last ? (now - topic.last) / 1000 : null,
                hz: topic?.hz || 0,
              },
            ];
          }),
        );
        setStats(nextStats);
        onHealthChangeRef.current?.({ health: next, tfLinks: nextLinks, stats: nextStats });
        return next;
      });
    }, 1000);
    return () => clearInterval(id);
  }, []);

  if (compact) {
    const compactItems = [
      ["tfChain", "Position", "Position tracking (TF)"],
      ["odom", "Motion", "Odometry"],
      ["amcl", "Locate", "Localization (AMCL)"],
      ["nav2", "Navigate", "Navigation (Nav2)"],
      ["map", "Map", "Map"],
      ["scan", "Laser", "Laser Scan"],
      ["costmap", "Obstacles", "Global Costmap"],
      ["plan", "Plan", "Path Plan"],
    ];

    return (
      <div className="flex h-full min-h-0 flex-col font-[RobotoMono]">
        <div className="mb-2 flex items-center justify-between gap-3">
          <p className="text-xs uppercase tracking-wider text-themeTextGray">
            Health
          </p>
          <div className="flex items-center gap-2">
            <span
              className={`h-2 w-2 rounded-full ${
                rosbridgeStatus === "connected" ? DOT.online : DOT.offline
              }`}
            />
            <span
              className={`text-xs ${
                rosbridgeStatus === "connected" ? TEXT.online : TEXT.offline
              }`}
            >
              Robot {rosbridgeStatus}
            </span>
          </div>
        </div>

        <div className="grid min-h-0 flex-1 grid-cols-2 content-start gap-2">
          {compactItems.map(([key, label, title]) => {
            const state = health[key] || "unknown";
            const isOnline = state === "online";
            return (
              <div
                key={key}
                title={title}
                className={`flex min-h-[34px] items-center gap-2 rounded-lg border px-3 text-xs ${
                  isOnline
                    ? "border-statusGreen/30 bg-statusGreen/10 text-statusGreen"
                    : state === "offline"
                    ? "border-statusRed/30 bg-statusRed/10 text-statusRed"
                    : "border-borderSubtle bg-bgSurface text-themeTextGray"
                }`}
              >
                <span
                  className={`h-2 w-2 shrink-0 rounded-full ${DOT[state]}`}
                />
                <span className="truncate">{label}</span>
              </div>
            );
          })}
        </div>
      </div>
    );
  }

  return (
    <div
      className={`dashboard-card font-[RobotoMono] ${compact ? "p-3" : "p-4"}`}
    >
      <div className="mb-2 flex items-center justify-between gap-3">
        <p className="text-xs uppercase tracking-wider text-themeTextGray">
          System Health
        </p>
        <div className="flex items-center gap-2">
          <span
            className={`h-2 w-2 rounded-full ${
              rosbridgeStatus === "connected" ? DOT.online : DOT.offline
            }`}
          />
          <span
            className={`text-xs ${
              rosbridgeStatus === "connected" ? TEXT.online : TEXT.offline
            }`}
          >
            Robot {rosbridgeStatus}
          </span>
        </div>
      </div>

      <div
        className={`grid grid-cols-1 gap-1.5 ${
          compact ? "mb-2 sm:grid-cols-3" : "mb-3"
        }`}
      >
        {TF_LINKS.map(([from, to]) => {
          const key = `${from}->${to}`;
          const state = tfLinks[key] || "unknown";
          return (
            <div key={key} className="flex items-center justify-between gap-3">
              <span className="truncate text-xs text-textWhiteHover" title={key}>
                {FRIENDLY_TF_LABELS[key] || key}
              </span>
              <span className={`shrink-0 text-xs ${TEXT[state]}`}>{state}</span>
            </div>
          );
        })}
      </div>

      <div
        className={`grid gap-x-4 gap-y-1.5 ${
          compact ? "grid-cols-2" : "grid-cols-1 sm:grid-cols-2"
        }`}
      >
        <div className="flex items-center gap-2">
          <div className="relative flex h-2 w-2 shrink-0">
            {health.tfChain === "online" && (
              <span
                className={`absolute inline-flex h-full w-full animate-ping rounded-full ${DOT.online} opacity-50`}
              />
            )}
            <span
              className={`relative inline-flex h-2 w-2 rounded-full ${
                DOT[health.tfChain]
              }`}
            />
          </div>
          <span
            className={`truncate text-xs ${TEXT[health.tfChain]}`}
            title="TF chain"
          >
            Position tracking
          </span>
        </div>
        {STREAMING.map(({ key, label }) => {
          const state = health[key] || "unknown";
          const isActive = state === "online";
          const displayLabel = key === "nav2" ? "Navigation" : label;
          return (
            <div key={key} className="flex min-w-0 items-center gap-2">
              <div className="relative flex h-2 w-2 shrink-0">
                {isActive && (
                  <span
                    className={`absolute inline-flex h-full w-full animate-ping rounded-full ${DOT.online} opacity-50`}
                  />
                )}
                <span
                  className={`relative inline-flex h-2 w-2 rounded-full ${DOT[state]}`}
                />
              </div>
              <span className={`truncate text-xs ${TEXT[state]}`}>{displayLabel}</span>
              {!compact && (
                <span
                  className="ml-auto text-[10px] text-themeTextGray"
                  title="Time since the last update / how often it's updating"
                >
                  {stats[key]?.age === null || stats[key] === undefined
                    ? "--"
                    : `${stats[key].age.toFixed(1)}s / ${stats[key].hz.toFixed(
                        1,
                      )}Hz`}
                </span>
              )}
            </div>
          );
        })}
      </div>
    </div>
  );
};

export default SystemHealth;
