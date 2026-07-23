import React, { useEffect, useRef, useState } from "react";
import { useRos } from "../app/App";
import { LIFECYCLE_NODES as NODES } from "../shared/constants";

const TRANSITIONS = {
  configure: 1,
  cleanup: 2,
  activate: 3,
  deactivate: 4,
};

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

const EXPLANATIONS = {
  active: "Configured and running normally.",
  inactive: "Configured but idle — call Activate to bring it online.",
  unconfigured: "Not configured yet — call Configure, or it's a fresh restart. Normal right after Nav2 launches, before activation.",
  unknown: "get_state didn't respond — the node may not be running, rosbridge may be disconnected, or the service name doesn't match.",
};

const normalizeState = (label) => (label || "unknown").toLowerCase();

const LifecycleStatus = ({ compact = false }) => {
  const ros = useRos();
  const clientsRef = useRef({});
  const changeClientsRef = useRef({});
  const [states, setStates] = useState(() =>
    Object.fromEntries(NODES.map(({ name }) => [name, "unknown"])),
  );
  const [showLegend, setShowLegend] = useState(false);

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
    changeClientsRef.current = Object.fromEntries(
      NODES.map(({ name, base }) => [
        name,
        new window.ROSLIB.Service({
          ros,
          name: `${base}/change_state`,
          serviceType: "lifecycle_msgs/srv/ChangeState",
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
            setStates((prev) => ({ ...prev, [name]: nextState }));
          },
          () => setStates((prev) => ({ ...prev, [name]: "unknown" })),
        );
      });
    };

    poll();
    const id = setInterval(poll, 3000);
    return () => clearInterval(id);
  }, [ros]);

  const changeAll = (transitionName) => {
    const id = TRANSITIONS[transitionName];
    if (!id || !window.ROSLIB) return;
    NODES.forEach(({ name }) => {
      const client = changeClientsRef.current[name];
      if (!client) return;
      client.callService(
        new window.ROSLIB.ServiceRequest({
          transition: { id, label: transitionName },
        }),
        () => {},
        () => {},
      );
    });
  };

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

        <div className="mb-2 grid grid-cols-2 gap-1.5">
          <button
            onClick={() => changeAll("configure")}
            className="rounded-md border border-borderSubtle px-2 py-1 text-[11px] text-themeBlue hover:border-themeBlue"
          >
            Config
          </button>
          <button
            onClick={() => changeAll("activate")}
            className="rounded-md border border-borderSubtle px-2 py-1 text-[11px] text-statusGreen hover:border-statusGreen"
          >
            Active
          </button>
          <button
            onClick={() => changeAll("deactivate")}
            className="rounded-md border border-borderSubtle px-2 py-1 text-[11px] text-statusYellow hover:border-statusYellow"
          >
            Deact
          </button>
          <button
            onClick={() => changeAll("cleanup")}
            className="rounded-md border border-borderSubtle px-2 py-1 text-[11px] text-statusRed hover:border-statusRed"
          >
            Clean
          </button>
        </div>

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
                  <span className="truncate text-textWhiteHover">
                    {name.replace("_server", "").replace("bt_navigator", "bt")}
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
      <p className="mb-2 text-xs uppercase tracking-wider text-themeTextGray">
        Lifecycle
      </p>
      <div className="mb-2 grid grid-cols-2 gap-1.5">
        <button
          onClick={() => changeAll("configure")}
          className="rounded-lg border border-borderSubtle px-2 py-1 text-xs text-themeBlue hover:border-themeBlue"
        >
          Configure
        </button>
        <button
          onClick={() => changeAll("activate")}
          className="rounded-lg border border-borderSubtle px-2 py-1 text-xs text-statusGreen hover:border-statusGreen"
        >
          Activate
        </button>
        <button
          onClick={() => changeAll("deactivate")}
          className="rounded-lg border border-borderSubtle px-2 py-1 text-xs text-statusYellow hover:border-statusYellow"
        >
          Deactivate
        </button>
        <button
          onClick={() => changeAll("cleanup")}
          className="rounded-lg border border-borderSubtle px-2 py-1 text-xs text-statusRed hover:border-statusRed"
        >
          Cleanup
        </button>
      </div>
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
                <span className="text-xs text-textWhiteHover">{name}</span>
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
