import React, { useCallback, useEffect, useRef, useState } from "react";
import { useNavigate } from "react-router-dom";

import { createAnnouncementMemory, nextAnnouncements, primeMemory } from "../shared/reporting/announcements";
import { DISPLAY } from "../shared/reporting/reportingAdapter";
import { useReporting } from "../shared/reporting/reportingSource";

/**
 * Reporting Mode: read-only, full-screen view for the detachable panel
 * (Getac ZX80, 8 inch, 1920 x 1200, landscape, Android Chrome). Scope is
 * base and navigation only. It has no command of any kind: it only renders
 * the shared reporting adapter (shared/reporting).
 *
 * Proposed defaults for Parth to confirm on the ZX80: the type sizes in
 * TYPE below and the 2 s long press on the exit control.
 */

// Type sizes in CSS px for 1920 x 1200. Proposed defaults.
const TYPE = { state: 120, reason: 44, action: 32, body: 28, row: 24, small: 20 };

// Long press on the corner control to leave Reporting Mode. Proposed default.
export const EXIT_HOLD_MS = 2000;

const STATE_STYLE = {
  [DISPLAY.READY]: { bg: "#0b3d1e", fg: "#ffffff", accent: "#34d399" },
  [DISPLAY.EXECUTING]: { bg: "#0b2a4a", fg: "#ffffff", accent: "#60a5fa" },
  [DISPLAY.WAITING]: { bg: "#2a2a2a", fg: "#ffffff", accent: "#e5e7eb" },
  [DISPLAY.BLOCKED]: { bg: "#4a3300", fg: "#ffffff", accent: "#fbbf24" },
  [DISPLAY.RECOVERING]: { bg: "#4a3300", fg: "#ffffff", accent: "#fbbf24" },
  [DISPLAY.NEEDS_ASSISTANCE]: { bg: "#5a2a00", fg: "#ffffff", accent: "#fb923c" },
  [DISPLAY.FAULTED]: { bg: "#5c0b0b", fg: "#ffffff", accent: "#f87171" },
  [DISPLAY.STOPPED_SOFTWARE]: { bg: "#3a0a0a", fg: "#ffffff", accent: "#f87171" },
  [DISPLAY.UNKNOWN]: { bg: "#1f1f1f", fg: "#ffffff", accent: "#9ca3af" },
  [DISPLAY.DISCONNECTED]: { bg: "#111111", fg: "#ffffff", accent: "#9ca3af" },
};

const useWakeLock = () => {
  const [status, setStatus] = useState("not supported");
  useEffect(() => {
    if (!navigator.wakeLock?.request) return undefined;
    let lock = null;
    let cancelled = false;
    const acquire = async () => {
      try {
        lock = await navigator.wakeLock.request("screen");
        if (!cancelled) setStatus("on");
        lock.addEventListener?.("release", () => !cancelled && setStatus("released"));
      } catch {
        if (!cancelled) setStatus("refused");
      }
    };
    const onVisible = () => {
      if (document.visibilityState === "visible") acquire();
    };
    acquire();
    document.addEventListener("visibilitychange", onVisible);
    return () => {
      cancelled = true;
      document.removeEventListener("visibilitychange", onVisible);
      lock?.release?.().catch?.(() => {});
    };
  }, []);
  return status;
};

const ExitControl = () => {
  const navigate = useNavigate();
  const timer = useRef(null);
  const [holding, setHolding] = useState(false);
  const start = () => {
    setHolding(true);
    timer.current = setTimeout(() => navigate("/"), EXIT_HOLD_MS);
  };
  const cancel = () => {
    setHolding(false);
    clearTimeout(timer.current);
  };
  useEffect(() => () => clearTimeout(timer.current), []);
  return (
    <button
      type="button"
      aria-label="Hold to exit Reporting Mode"
      onPointerDown={start}
      onPointerUp={cancel}
      onPointerLeave={cancel}
      onPointerCancel={cancel}
      onContextMenu={(e) => e.preventDefault()}
      style={{ fontSize: TYPE.small }}
      className={`absolute right-4 top-4 rounded-lg border px-4 py-3 ${
        holding ? "border-white bg-white/20" : "border-white/30 bg-black/30"
      } select-none text-white/80`}
    >
      {holding ? "Keep holding to exit" : "Hold to exit"}
    </button>
  );
};

const SpeechToggle = ({ view }) => {
  const supported = typeof window !== "undefined" && "speechSynthesis" in window;
  const [enabled, setEnabled] = useState(false);
  const memory = useRef(createAnnouncementMemory());

  useEffect(() => {
    if (!enabled || !view) return;
    nextAnnouncements(memory.current, view).forEach((text) => {
      window.speechSynthesis.speak(new window.SpeechSynthesisUtterance(text));
    });
  }, [enabled, view]);

  const toggle = useCallback(() => {
    setEnabled((on) => {
      if (!on) {
        memory.current = primeMemory(createAnnouncementMemory(), view);
      } else {
        window.speechSynthesis?.cancel?.();
      }
      return !on;
    });
  }, [view]);

  if (!supported) {
    return (
      <span style={{ fontSize: TYPE.small }} className="text-white/60">
        Speech not supported
      </span>
    );
  }
  return (
    <button
      type="button"
      aria-pressed={enabled}
      onClick={toggle}
      style={{ fontSize: TYPE.small }}
      className="rounded-lg border border-white/30 px-4 py-3 text-white"
    >
      {enabled ? "Speech on" : "Speech off"}
    </button>
  );
};

const ReportingPage = () => {
  const view = useReporting();
  const wakeLock = useWakeLock();

  if (!view) return null;
  const style = STATE_STYLE[view.display] || STATE_STYLE[DISPLAY.UNKNOWN];

  return (
    <div
      data-testid="reporting-page"
      data-state={view.display}
      className="relative flex h-screen w-screen flex-col overflow-hidden font-[RobotoMono]"
      style={{ background: style.bg, color: style.fg }}
    >
      <ExitControl />
      <header className="flex items-center gap-6 px-10 pt-6" style={{ fontSize: TYPE.body }}>
        <span className="font-semibold">Base and navigation</span>
        <span className="text-white/60" style={{ fontSize: TYPE.small }}>
          Navigation status covers navigation only. It is not whole-robot readiness or E-stop state.
        </span>
      </header>

      <main className="grid flex-1 grid-cols-2 gap-10 px-10 py-6">
        <section aria-label="Robot state">
          <h1
            data-testid="reporting-state"
            className="font-bold leading-none"
            style={{ fontSize: TYPE.state, color: style.accent }}
          >
            {view.display}
          </h1>
          <p data-testid="reporting-reason" className="mt-6 font-semibold" style={{ fontSize: TYPE.reason }}>
            {view.reason}
          </p>
          <p data-testid="reporting-next" className="mt-4 text-white/90" style={{ fontSize: TYPE.action }}>
            Next: {view.nextAction}
          </p>
          {view.completion && (
            <p data-testid="reporting-completion" className="mt-6 font-semibold" style={{ fontSize: TYPE.reason, color: "#34d399" }}>
              {view.completion}
            </p>
          )}
        </section>

        <section aria-label="Task and result" style={{ fontSize: TYPE.body }} className="flex flex-col gap-5">
          <div data-testid="reporting-task">
            <p className="text-white/60" style={{ fontSize: TYPE.small }}>Current task</p>
            {view.task ? (
              <p>
                {view.task.label}
                {view.task.step ? `, ${view.task.step}` : ""}
              </p>
            ) : (
              <p className="text-white/70">None</p>
            )}
          </div>

          {view.progress && (
            <div data-testid="reporting-progress">
              <p className="text-white/60" style={{ fontSize: TYPE.small }}>Progress</p>
              <div
                role="progressbar"
                aria-valuenow={view.progress.percent}
                aria-valuemin={0}
                aria-valuemax={100}
                className="mt-2 h-6 w-full overflow-hidden rounded-full"
                style={{ background: "rgba(255, 255, 255, 0.2)" }}
              >
                <div className="h-full" style={{ width: `${view.progress.percent}%`, background: "#ffffff" }} />
              </div>
              <p className="mt-2">
                {view.progress.distanceRemaining.toFixed(1)} m remaining
                {Number.isFinite(view.progress.etaSeconds) ? `, about ${Math.round(view.progress.etaSeconds)} s` : ""}
                {view.progress.recoveries ? `, ${view.progress.recoveries} recoveries` : ""}
              </p>
            </div>
          )}

          <div data-testid="reporting-result">
            <p className="text-white/60" style={{ fontSize: TYPE.small }}>Last result</p>
            {view.result ? (
              <p>
                {view.result.name}: {view.result.invalid ? "Result unknown (invalid data)" : view.result.outcome}
                {view.result.reason ? `. ${view.result.reason}` : ""}
              </p>
            ) : (
              <p className="text-white/70">None</p>
            )}
          </div>

          <div data-testid="reporting-warnings">
            <p className="text-white/60" style={{ fontSize: TYPE.small }}>Warnings</p>
            {view.warnings.length ? (
              <ul className="list-disc pl-8">
                {view.warnings.slice(0, 5).map((w) => (
                  <li key={w}>{w}</li>
                ))}
              </ul>
            ) : (
              <p className="text-white/70">None</p>
            )}
          </div>
        </section>
      </main>

      <footer className="grid grid-cols-3 gap-x-10 gap-y-2 border-t border-white/20 px-10 py-4" style={{ fontSize: TYPE.row }}>
        {view.rows.map((row) => (
          <div key={row.label} className="flex justify-between gap-4" data-testid={`row-${row.label}`}>
            <span className="text-white/70">{row.label}</span>
            <span className="font-semibold">
              {row.value}
              {row.freshness && row.freshness !== "Fresh" ? ` (${row.freshness.toLowerCase()})` : ""}
            </span>
          </div>
        ))}
        <div className="col-span-3 mt-2 flex items-center justify-between text-white/60" style={{ fontSize: TYPE.small }}>
          <span>Screen wake lock: {wakeLock}</span>
          <SpeechToggle view={view} />
        </div>
      </footer>
    </div>
  );
};

export default ReportingPage;
