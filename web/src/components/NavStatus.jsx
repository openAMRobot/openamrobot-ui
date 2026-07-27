import React, { useEffect, useRef, useState } from "react";
import { useRos } from "../app/App";
import { AppConfig } from "../shared/constants";

const STATUS_CODES = {
  0: {
    label: "Unknown",
    color: "text-themeTextGray",
    dot: "bg-themeTextGray",
    explain: "No goal has been sent yet, or its status hasn't arrived. Send a goal from the map.",
  },
  1: {
    label: "Accepted",
    color: "text-statusBlue",
    dot: "bg-statusBlue",
    explain: "The robot's navigation system accepted the goal and is about to start planning and moving.",
  },
  2: {
    label: "Navigating",
    color: "text-statusGreen",
    dot: "bg-statusGreen",
    explain: "The robot is actively following a path toward the goal.",
  },
  3: {
    label: "Canceling",
    color: "text-statusYellow",
    dot: "bg-statusYellow",
    explain: "A cancel request was sent; the robot is stopping the current goal.",
  },
  4: {
    label: "Succeeded",
    color: "text-statusGreen",
    dot: "bg-statusGreen",
    explain: "The robot reached the goal — within the distance that counts as \"close enough.\"",
  },
  5: {
    label: "Canceled",
    color: "text-themeTextGray",
    dot: "bg-themeTextGray",
    explain: "The goal was stopped before completion — by the Cancel button or a new goal overriding it. Not an error; send a new goal to continue.",
  },
  6: {
    label: "Failed",
    color: "text-statusRed",
    dot: "bg-statusRed",
    explain: "The robot's navigation system couldn't complete the goal — usually no valid path was found, it tried to recover and couldn't, or the robot wasn't sure enough of its position to navigate safely. Check that the robot's position estimate looks accurate and the goal isn't inside an obstacle, then retry.",
  },
};

const NavStatus = ({ onCancelGoal }) => {
  const ros = useRos();
  const [navStatus, setNavStatus] = useState(0);
  const [distRemaining, setDistRemaining] = useState(null);
  const [history, setHistory] = useState([]);
  const [showExplain, setShowExplain] = useState(false);

  useEffect(() => setShowExplain(false), [navStatus]);
  const lastTerminalRef = useRef(null);
  const statusTopicRef = useRef(null);
  const feedbackTopicRef = useRef(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    statusTopicRef.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.NAV_STATUS_TOPIC,
      messageType: "action_msgs/GoalStatusArray",
    });

    feedbackTopicRef.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.NAV_FEEDBACK_TOPIC,
      messageType: "nav2_msgs/action/NavigateToPose_FeedbackMessage",
    });

    statusTopicRef.current.subscribe((msg) => {
      if (msg.status_list && msg.status_list.length > 0) {
        const latest = msg.status_list[msg.status_list.length - 1];
        setNavStatus(latest.status);
        if ([4, 5, 6].includes(latest.status)) {
          const key =
            latest.goal_info?.goal_id?.uuid?.join?.("-") || Date.now();
          if (lastTerminalRef.current !== key) {
            lastTerminalRef.current = key;
            const statusInfo = STATUS_CODES[latest.status] || STATUS_CODES[0];
            setHistory((prev) =>
              [
                {
                  label: statusInfo.label,
                  status: latest.status,
                  stamp: new Date().toLocaleTimeString(),
                  distance: distRemaining,
                },
                ...prev,
              ].slice(0, 5),
            );
          }
        }
        if (latest.status === 1 || latest.status === 2 || latest.status === 3) {
          setDistRemaining(null);
        }
        if (latest.status === 4 || latest.status === 5 || latest.status === 6) {
          setTimeout(() => window.NAV2D?.clearPath?.(), 1000);
          setTimeout(() => setNavStatus(0), 3000);
          setTimeout(() => setDistRemaining(null), 3000);
        }
      } else {
        setNavStatus(0);
      }
    });

    feedbackTopicRef.current.subscribe((msg) => {
      const dist = msg?.feedback?.distance_remaining;
      if (dist !== undefined) setDistRemaining(dist.toFixed(2));
    });

    return () => {
      statusTopicRef.current?.unsubscribe();
      feedbackTopicRef.current?.unsubscribe();
    };
  }, [ros]);

  const statusInfo = STATUS_CODES[navStatus] || STATUS_CODES[0];
  const { label, color, dot, explain } = statusInfo;
  const isActive = navStatus === 1 || navStatus === 2 || navStatus === 3;

  return (
    <div className="dashboard-card px-4 py-1.5 font-[RobotoMono]">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-3">
          <div className="relative flex h-2.5 w-2.5">
            {isActive && (
              <span
                className={`absolute inline-flex h-full w-full animate-ping rounded-full ${dot} opacity-60`}
              />
            )}
            <span
              className={`relative inline-flex h-2.5 w-2.5 rounded-full ${dot}`}
            />
          </div>
          <div>
            <span className="text-xs uppercase tracking-wider text-themeTextGray">
              Navigation{" "}
            </span>
            <span className={`text-sm font-semibold ${color}`}>{label}</span>
            <button
              onClick={() => setShowExplain((v) => !v)}
              aria-label="What does this status mean?"
              title="What does this status mean?"
              className="ml-1.5 inline-flex h-4 w-4 items-center justify-center rounded-full border border-borderSubtle text-[10px] text-themeTextGray hover:border-themeBlue hover:text-themeBlue"
            >
              ?
            </button>
            {isActive && distRemaining !== null && (
              <span className="ml-3 text-xs text-themeTextGray">
                {distRemaining}m remaining
              </span>
            )}
          </div>
        </div>

        {isActive && (
          <button
            onClick={onCancelGoal}
            className="rounded-lg border border-statusRed px-3 py-1 text-xs text-statusRed transition-colors hover:bg-statusRed hover:text-white"
          >
            Cancel
          </button>
        )}
      </div>

      {showExplain && (
        <p className="mt-1 border-t border-borderSubtle pt-1 text-[11px] leading-snug text-themeTextGray">
          {explain}
        </p>
      )}

      {history.length > 0 && (
        <div className="mt-1 flex max-h-[22px] flex-wrap gap-2 overflow-hidden border-t border-borderSubtle pt-1">
          {history.map((item, index) => {
            const info = STATUS_CODES[item.status] || STATUS_CODES[0];
            return (
              <span
                key={`${item.stamp}-${index}`}
                className={`rounded border border-borderSubtle bg-bgSurface px-2 py-0.5 text-[10px] ${info.color}`}
              >
                {item.stamp} {item.label}
              </span>
            );
          })}
        </div>
      )}
    </div>
  );
};

export default NavStatus;
