import { useContext, useRef, useEffect, useState } from "react";
import { RosContext } from "../app/App";
import { AppConfig } from "../shared/constants";

// Position the robot moves to after undocking (map frame, metres)
const STANDBY_POSE = { x: 0.0, y: 0.0, yaw: 0.0 };

const STATUSES = {
  idle: {
    label: "Idle",
    detail: "Ready",
    textCls: "text-themeTextGray",
    dotCls: "bg-themeTextGray",
    pulse: false,
  },
  docking: {
    label: "Final approach",
    detail: "Aligning to dock",
    textCls: "text-statusYellow",
    dotCls: "bg-statusYellow",
    pulse: true,
  },
  docked: {
    label: "Docked",
    detail: "Sequence complete",
    textCls: "text-statusGreen",
    dotCls: "bg-statusGreen",
    pulse: false,
  },
  undocking: {
    label: "Undocking…",
    detail: "Backing out",
    textCls: "text-statusYellow",
    dotCls: "bg-statusYellow",
    pulse: true,
  },
  standby: {
    label: "To standby…",
    detail: "Navigating clear",
    textCls: "text-statusBlue",
    dotCls: "bg-statusBlue",
    pulse: true,
  },
  failed: {
    label: "Failed",
    detail: "Check tag and logs",
    textCls: "text-statusRed",
    dotCls: "bg-statusRed",
    pulse: false,
  },
};

const DockingControl = ({ compact = false }) => {
  const ros = useContext(RosContext);
  const [status, setStatus] = useState("idle");
  const [events, setEvents] = useState([]);
  const dockTriggerRef = useRef(null);
  const undockTriggerRef = useRef(null);
  const goalPoseTopicRef = useRef(null);
  const statusRef = useRef(status);

  useEffect(() => {
    statusRef.current = status;
  }, [status]);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    dockTriggerRef.current = new window.ROSLIB.Topic({
      ros,
      name: "/dock_trigger",
      messageType: "std_msgs/Bool",
    });

    undockTriggerRef.current = new window.ROSLIB.Topic({
      ros,
      name: "/undock_robot",
      messageType: "std_msgs/Bool",
    });

    goalPoseTopicRef.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.GOAL_POSE_TOPIC,
      messageType: "geometry_msgs/PoseStamped",
    });

    // dock_trigger.py publishes one of: idle | docking | docked | undocking | failed
    const dockStatusTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.DOCK_TRIGGER_STATUS_TOPIC,
      messageType: "std_msgs/String",
    });

    const navStatusTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.NAV_STATUS_TOPIC,
      messageType: "action_msgs/GoalStatusArray",
    });

    const applyStatus = (nextStatus) => {
      setStatus(nextStatus);
      const statusInfo = STATUSES[nextStatus] || STATUSES.idle;
      setEvents((prev) =>
        [
          {
            label: statusInfo.label,
            detail: statusInfo.detail,
            stamp: new Date().toLocaleTimeString(),
          },
          ...prev,
        ].slice(0, 5),
      );
    };

    dockStatusTopic.subscribe((msg) => {
      const s = msg.data;
      if (s === "docking") applyStatus("docking");
      else if (s === "docked") applyStatus("docked");
      else if (s === "undocking") applyStatus("undocking");
      else if (s === "failed") applyStatus("failed");
      else if (s === "idle") {
        if (statusRef.current === "undocking") {
          // robot just finished undocking — navigate to standby pose
          applyStatus("standby");
          sendStandbyGoal();
        } else {
          applyStatus("idle");
        }
      }
    });

    navStatusTopic.subscribe((msg) => {
      if (!msg.status_list?.length) return;
      const s = msg.status_list[msg.status_list.length - 1].status;
      const cur = statusRef.current;
      if (s === 4 && cur === "standby") applyStatus("idle");
      else if (s === 6 && cur === "standby") applyStatus("failed");
    });

    return () => {
      dockStatusTopic.unsubscribe();
      navStatusTopic.unsubscribe();
    };
  }, [ros]);

  const sendStandbyGoal = () => {
    if (!goalPoseTopicRef.current) return;
    const yaw = STANDBY_POSE.yaw;
    goalPoseTopicRef.current.publish(
      new window.ROSLIB.Message({
        header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
        pose: {
          position: { x: STANDBY_POSE.x, y: STANDBY_POSE.y, z: 0.0 },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: Math.sin(yaw / 2),
            w: Math.cos(yaw / 2),
          },
        },
      }),
    );
  };

  const handleDock = () => {
    if (!dockTriggerRef.current) return;
    dockTriggerRef.current.publish(new window.ROSLIB.Message({ data: true }));
    setStatus("docking");
    setEvents((prev) =>
      [
        {
          label: "Searching tag",
          detail: "Dock requested",
          stamp: new Date().toLocaleTimeString(),
        },
        ...prev,
      ].slice(0, 5),
    );
  };

  const handleUndock = () => {
    if (!undockTriggerRef.current) return;
    undockTriggerRef.current.publish(new window.ROSLIB.Message({ data: true }));
    setStatus("undocking");
    setEvents((prev) =>
      [
        {
          label: "Undocking…",
          detail: "Reverse maneuver requested",
          stamp: new Date().toLocaleTimeString(),
        },
        ...prev,
      ].slice(0, 5),
    );
  };

  const { label, detail, textCls, dotCls, pulse } = STATUSES[status];
  const busy =
    status === "docking" || status === "undocking" || status === "standby";

  if (compact) {
    return (
      <div className="flex min-h-0 flex-col justify-between font-[RobotoMono]">
        <div className="mb-2 flex items-center justify-between gap-3">
          <p className="text-xs uppercase tracking-wider text-themeTextGray">
            Docking
          </p>
          <div className="flex items-center gap-2">
            <span className="relative flex h-2 w-2">
              {pulse && (
                <span
                  className={`absolute inline-flex h-full w-full animate-ping rounded-full opacity-75 ${dotCls}`}
                />
              )}
              <span
                className={`relative inline-flex h-2 w-2 rounded-full ${dotCls}`}
              />
            </span>
            <span className={`text-xs ${textCls}`}>{label}</span>
          </div>
        </div>
        <div className="mb-3 rounded-lg bg-bgSurface px-3 py-2">
          <p className="text-xs text-themeTextGray">{detail}</p>
        </div>

        <div className="grid grid-cols-2 gap-2">
          <button
            onClick={handleDock}
            disabled={busy || status === "docked"}
            className="rounded-lg border border-themeBlue bg-themeBlue/10 py-2 text-xs font-semibold text-themeBlue transition-colors hover:bg-themeBlue hover:text-white disabled:cursor-not-allowed disabled:opacity-40"
          >
            Dock
          </button>
          <button
            onClick={handleUndock}
            disabled={busy || status === "idle"}
            className="rounded-lg border border-borderSubtle bg-bgCard py-2 text-xs font-semibold text-textWhiteHover transition-colors hover:border-themeBlue hover:text-themeBlue disabled:cursor-not-allowed disabled:opacity-40"
          >
            Undock
          </button>
        </div>
      </div>
    );
  }

  return (
    <div
      className={`rounded-xl border border-borderSubtle bg-bgCard font-[RobotoMono] ${
        compact ? "p-3" : "p-4"
      }`}
    >
      <div className="mb-2 flex items-center justify-between">
        <p className="text-xs uppercase tracking-wider text-themeTextGray">
          Docking
        </p>
        <div className="flex items-center gap-2">
          <span className="relative flex h-2 w-2">
            {pulse && (
              <span
                className={`absolute inline-flex h-full w-full animate-ping rounded-full opacity-75 ${dotCls}`}
              />
            )}
            <span
              className={`relative inline-flex h-2 w-2 rounded-full ${dotCls}`}
            />
          </span>
          <span className={`text-xs ${textCls}`}>{label}</span>
        </div>
      </div>
      <p className="mb-2 text-xs text-themeTextGray">{detail}</p>

      <div className="flex gap-2">
        <button
          onClick={handleDock}
          disabled={busy || status === "docked"}
          className="flex-1 rounded-lg border border-themeBlue bg-themeBlue/10 py-1.5 text-xs font-semibold text-themeBlue transition-colors hover:bg-themeBlue hover:text-white disabled:cursor-not-allowed disabled:opacity-40"
        >
          ⚓ Dock
        </button>
        <button
          onClick={handleUndock}
          disabled={busy || status === "idle"}
          className="flex-1 rounded-lg border border-borderSubtle bg-bgCard py-1.5 text-xs font-semibold text-textWhiteHover transition-colors hover:border-themeBlue hover:text-themeBlue disabled:cursor-not-allowed disabled:opacity-40"
        >
          ↩ Undock
        </button>
      </div>

      {status === "failed" && (
        <button
          onClick={() => setStatus("idle")}
          className="mt-2 w-full text-center text-[10px] text-statusRed opacity-70 hover:opacity-100"
        >
          Dismiss
        </button>
      )}

      {!compact && events.length > 0 && (
        <div className="mt-3 border-t border-borderSubtle pt-2">
          <p className="mb-1 text-[10px] uppercase tracking-wider text-themeTextGray">
            Recent Dock Events
          </p>
          <div className="space-y-1">
            {events.map((event, index) => (
              <div
                key={`${event.stamp}-${index}`}
                className="flex justify-between gap-3 text-[10px]"
              >
                <span className="text-textWhiteHover">{event.label}</span>
                <span className="text-themeTextGray">{event.stamp}</span>
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
};

export default DockingControl;
