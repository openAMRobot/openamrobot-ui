import React, { useEffect, useMemo, useRef, useState } from "react";
import { useRos } from "../app/App";

// rcl_interfaces/msg/Log severity levels (ROS 2). Anything below the selected
// minimum is filtered out client-side.
const LEVELS = {
  10: { label: "DEBUG", color: "text-themeTextGray", chip: "border-borderSubtle text-themeTextGray" },
  20: { label: "INFO", color: "text-statusGreen", chip: "border-statusGreen/40 text-statusGreen" },
  30: { label: "WARN", color: "text-statusYellow", chip: "border-statusYellow/40 text-statusYellow" },
  40: { label: "ERROR", color: "text-statusRed", chip: "border-statusRed/40 text-statusRed" },
  50: { label: "FATAL", color: "text-statusRed", chip: "border-statusRed/60 text-statusRed" },
};

const LEVEL_OPTIONS = [
  [10, "Debug+"],
  [20, "Info+"],
  [30, "Warn+"],
  [40, "Error+"],
  [50, "Fatal"],
];

const MAX_LOGS = 1000;

const fmtTime = (stamp) => {
  if (!stamp) return "";
  const ms = stamp.sec * 1000 + Math.round((stamp.nanosec || 0) / 1e6);
  const d = new Date(ms);
  if (Number.isNaN(d.getTime())) return "";
  return d.toLocaleTimeString(undefined, { hour12: false }) +
    "." + String(d.getMilliseconds()).padStart(3, "0");
};

/**
 * Live /rosout console. Subscribes to rcl_interfaces/msg/Log and renders a
 * filterable, pausable stream so operators can debug from the browser instead
 * of a sourced terminal. Incoming messages are buffered in a ref and flushed
 * to React state a few times a second — /rosout can burst well past a
 * reasonable re-render rate during startup or a fault.
 */
const RosoutConsole = () => {
  const ros = useRos();
  const [logs, setLogs] = useState([]);
  const [minLevel, setMinLevel] = useState(20);
  const [nodeFilter, setNodeFilter] = useState("");
  const [textFilter, setTextFilter] = useState("");
  const [paused, setPaused] = useState(false);
  const [autoscroll, setAutoscroll] = useState(true);

  const bufferRef = useRef([]);
  const pausedRef = useRef(paused);
  const seqRef = useRef(0);
  const listRef = useRef(null);

  useEffect(() => {
    pausedRef.current = paused;
  }, [paused]);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return undefined;

    const topic = new window.ROSLIB.Topic({
      ros,
      name: "/rosout",
      messageType: "rcl_interfaces/msg/Log",
    });

    const handler = (msg) => {
      if (pausedRef.current) return;
      bufferRef.current.push({
        id: seqRef.current++,
        level: msg.level,
        name: msg.name || "",
        msg: msg.msg || "",
        stamp: msg.stamp,
      });
    };

    topic.subscribe(handler);
    return () => topic.unsubscribe(handler);
  }, [ros]);

  // Flush the buffer into state at ~5 Hz, capped to the newest MAX_LOGS.
  useEffect(() => {
    const id = setInterval(() => {
      if (!bufferRef.current.length) return;
      const incoming = bufferRef.current;
      bufferRef.current = [];
      setLogs((prev) => {
        const next = prev.concat(incoming);
        return next.length > MAX_LOGS ? next.slice(next.length - MAX_LOGS) : next;
      });
    }, 200);
    return () => clearInterval(id);
  }, []);

  const filtered = useMemo(() => {
    const node = nodeFilter.trim().toLowerCase();
    const text = textFilter.trim().toLowerCase();
    return logs.filter((l) => {
      if (l.level < minLevel) return false;
      if (node && !l.name.toLowerCase().includes(node)) return false;
      if (text && !l.msg.toLowerCase().includes(text)) return false;
      return true;
    });
  }, [logs, minLevel, nodeFilter, textFilter]);

  // Keep the view pinned to the newest line while autoscroll is on.
  useEffect(() => {
    if (!autoscroll || !listRef.current) return;
    listRef.current.scrollTop = listRef.current.scrollHeight;
  }, [filtered, autoscroll]);

  return (
    <article className="dashboard-card flex h-full min-h-0 w-full flex-col overflow-hidden font-[RobotoMono]">
      <header className="flex flex-wrap items-center gap-2 border-b border-borderSubtle bg-bgSurface px-3 py-2">
        <h2
          className="mr-1 text-sm font-semibold uppercase tracking-wider text-themeBlue"
          title="/rosout"
        >
          System Log
        </h2>

        <select
          value={minLevel}
          onChange={(e) => setMinLevel(Number(e.target.value))}
          className="rounded-lg border border-borderSubtle bg-bgCard px-2 py-1 text-xs text-textWhiteHover"
        >
          {LEVEL_OPTIONS.map(([v, l]) => (
            <option key={v} value={v}>
              {l}
            </option>
          ))}
        </select>

        <input
          value={nodeFilter}
          onChange={(e) => setNodeFilter(e.target.value)}
          placeholder="node…"
          className="w-24 rounded-lg border border-borderSubtle bg-bgCard px-2 py-1 text-xs text-textWhiteHover placeholder:text-themeTextGray"
        />
        <input
          value={textFilter}
          onChange={(e) => setTextFilter(e.target.value)}
          placeholder="search text…"
          className="min-w-[100px] flex-1 rounded-lg border border-borderSubtle bg-bgCard px-2 py-1 text-xs text-textWhiteHover placeholder:text-themeTextGray"
        />

        <label className="flex items-center gap-1 text-[10px] uppercase text-themeTextGray">
          <input
            type="checkbox"
            checked={autoscroll}
            onChange={(e) => setAutoscroll(e.target.checked)}
            className="accent-themeBlue"
          />
          Follow
        </label>

        <button
          onClick={() => setPaused((p) => !p)}
          className={`rounded-lg border px-3 py-1 text-xs transition-colors ${
            paused
              ? "border-statusYellow text-statusYellow"
              : "border-borderSubtle text-themeTextGray hover:border-themeBlue hover:text-themeBlue"
          }`}
        >
          {paused ? "Resume" : "Pause"}
        </button>
        <button
          onClick={() => {
            bufferRef.current = [];
            setLogs([]);
          }}
          className="rounded-lg border border-borderSubtle px-3 py-1 text-xs text-themeTextGray transition-colors hover:border-themeBlue hover:text-themeBlue"
        >
          Clear
        </button>
      </header>

      <div
        ref={listRef}
        className="flex-1 overflow-y-auto px-2 py-1 text-xs leading-relaxed"
      >
        {filtered.length === 0 ? (
          <div className="flex h-full items-center justify-center text-themeTextGray opacity-50">
            {logs.length === 0 ? "Waiting for /rosout…" : "No messages match the filters"}
          </div>
        ) : (
          filtered.map((l) => {
            const info = LEVELS[l.level] || LEVELS[20];
            return (
              <div
                key={l.id}
                className="flex gap-2 border-b border-borderSubtle/20 py-0.5 last:border-0"
              >
                <span className="shrink-0 text-themeTextGray opacity-70">
                  {fmtTime(l.stamp)}
                </span>
                <span className={`w-11 shrink-0 font-semibold ${info.color}`}>
                  {info.label}
                </span>
                <span className="shrink-0 text-themeBlue/80">[{l.name}]</span>
                <span className="min-w-0 break-words text-textWhiteHover">
                  {l.msg}
                </span>
              </div>
            );
          })
        )}
      </div>

      <footer className="border-t border-borderSubtle bg-bgSurface px-3 py-1 text-[10px] uppercase tracking-wider text-themeTextGray">
        {filtered.length} shown · {logs.length}/{MAX_LOGS} buffered
        {paused && " · paused"}
      </footer>
    </article>
  );
};

export default RosoutConsole;
