import React, { useEffect, useRef, useState } from "react";
import { useRos } from "../app/App";
import { LIFECYCLE_NODES as NODES } from "../shared/constants";

const COLORS = {
  active: "text-statusGreen",
  inactive: "text-statusYellow",
  unconfigured: "text-statusRed",
  unknown: "text-themeTextGray",
};

const DOTS = {
  active: "bg-statusGreen",
  inactive: "bg-statusYellow",
  unconfigured: "bg-statusRed",
  unknown: "bg-themeTextGray",
};

// Read only. The platform's lifecycle manager configures, activates and
// deactivates these nodes; this panel only shows the state it reports
// through get_state and never calls change_state.
const EXPLANATIONS = {
  active: "Configured and running normally.",
  inactive: "Configured but idle. The platform lifecycle manager activates it; this panel cannot.",
  unconfigured: "Not configured yet. Normal right after the platform starts navigation, before its lifecycle manager configures it.",
  unknown: "The state could not be read. The node may be off, or the connection to the robot may be down.",
};

export const READ_ONLY_NOTE =
  "Read only. The platform lifecycle manager controls these systems.";

// Plain-language names for the internal ROS node each row represents.
// Exported so other places that reference these same node names (e.g. the
// Health Centre's issue list) can show the same friendly wording.
export const FRIENDLY_NAMES = {
  map_server: "Map data",
  amcl: "Position tracking",
  controller: "Driving control",
  planner: "Path planning",
  bt_navigator: "Navigation logic",
};

const normalizeState = (label) => (label || "unknown").toLowerCase();

const LifecycleStatus = ({ compact = false, onStatesChange }) => {
  const ros = useRos();
  const clientsRef = useRef({});
  const [states, setStates] = useState(() =>
    Object.fromEntries(NODES.map(({ name }) => [name, "unknown"])),
  );
  const [showLegend, setShowLegend] = useState(false);
  const onStatesChangeRef = useRef(onStatesChange);
  onStatesChangeRef.current = onStatesChange;

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    clientsRef.current = Object.fromEntries(
      NODES.map(({ name, base }) => [
        name,
        new window.ROSLIB.Service({
          ros,
          name: `${base}/get_state`,
          serviceType: "lifecycle_msgs/srv/GetState",
        }),
      ]),
    );

    const poll = () => {
      NODES.forEach(({ name }) => {
        const client = clientsRef.current[name];
        if (!client) return;
        client.callService(
          new window.ROSLIB.ServiceRequest({}),
          (response) => {
            const nextState = normalizeState(response?.current_state?.label);
            setStates((prev) => {
              const next = { ...prev, [name]: nextState };
              onStatesChangeRef.current?.(next);
              return next;
            });
          },
          () =>
            setStates((prev) => {
              const next = { ...prev, [name]: "unknown" };
              onStatesChangeRef.current?.(next);
              return next;
            }),
        );
      });
    };

    poll();
    const id = setInterval(poll, 3000);
    return () => clearInterval(id);
  }, [ros]);

  if (compact) {
    return (
      <div className="flex min-h-0 flex-col font-[RobotoMono]">
        <div className="mb-2 flex items-center justify-between gap-3">
          <p className="text-xs uppercase tracking-wider text-themeTextGray">
            Lifecycle
          </p>
          <button
            onClick={() => setShowLegend((v) => !v)}
            aria-label="What do these states mean?"
            title="What do these states mean?"
            className="inline-flex h-4 w-4 items-center justify-center rounded-full border border-borderSubtle text-[10px] text-themeTextGray hover:border-themeBlue hover:text-themeBlue"
          >
            ?
          </button>
        </div>

        {showLegend && (
          <div className="mb-2 space-y-1 rounded-lg bg-bgSurface p-2 text-[10px] leading-snug text-themeTextGray">
            {Object.entries(EXPLANATIONS).map(([state, text]) => (
              <p key={state}>
                <span className={`font-semibold ${COLORS[state]}`}>{state}</span>: {text}
              </p>
            ))}
          </div>
        )}

        <p className="mb-2 text-[10px] text-themeTextGray">{READ_ONLY_NOTE}</p>
        <div className="grid grid-cols-1 gap-1.5">
          {NODES.map(({ name }) => {
            const state = states[name] || "unknown";
            return (
              <div
                key={name}
                className="flex items-center justify-between gap-2 rounded-md bg-bgSurface px-2 py-1 text-[11px]"
              >
                <span className="flex min-w-0 items-center gap-1.5">
                  <span
                    className={`h-1.5 w-1.5 shrink-0 rounded-full ${
                      DOTS[state] || DOTS.unknown
                    }`}
                  />
                  <span className="truncate text-textWhiteHover" title={name}>
                    {FRIENDLY_NAMES[name] || name}
                  </span>
                </span>
                <span className={`shrink-0 ${COLORS[state] || COLORS.unknown}`}>
                  {state}
                </span>
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
          Lifecycle
        </p>
        <button
          onClick={() => setShowLegend((v) => !v)}
          aria-label="What do these states mean?"
          title="What do these states mean?"
          className="inline-flex h-4 w-4 items-center justify-center rounded-full border border-borderSubtle text-[10px] text-themeTextGray hover:border-themeBlue hover:text-themeBlue"
        >
          ?
        </button>
      </div>

      {showLegend && (
        <div className="mb-2 space-y-1 rounded-lg bg-bgSurface p-2 text-[11px] leading-snug text-themeTextGray">
          {Object.entries(EXPLANATIONS).map(([state, text]) => (
            <p key={state}>
              <span className={`font-semibold ${COLORS[state]}`}>{state}</span>: {text}
            </p>
          ))}
        </div>
      )}

      <p className="mb-2 text-[11px] text-themeTextGray">{READ_ONLY_NOTE}</p>
      <div
        className={`grid gap-1.5 ${
          compact ? "grid-cols-1" : "grid-cols-1 sm:grid-cols-2"
        }`}
      >
        {NODES.map(({ name }) => {
          const state = states[name] || "unknown";
          return (
            <div key={name} className="flex items-center justify-between gap-3">
              <div className="flex items-center gap-2">
                <span
                  className={`h-2 w-2 rounded-full ${
                    DOTS[state] || DOTS.unknown
                  }`}
                />
                <span className="text-xs text-textWhiteHover" title={name}>
                  {FRIENDLY_NAMES[name] || name}
                </span>
              </div>
              <span className={`text-xs ${COLORS[state] || COLORS.unknown}`}>
                {state}
              </span>
            </div>
          );
        })}
      </div>
    </div>
  );
};

export default LifecycleStatus;
