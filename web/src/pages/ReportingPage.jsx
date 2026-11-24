import React, { useEffect, useRef, useState } from "react";
import { useNavigate } from "react-router-dom";

import { useRuntimeConfig } from "../app/App";
import { createAnnouncementMemory, nextAnnouncements, stateText } from "../shared/reporting/announcements";
import { DISPLAY, formatClockTime } from "../shared/reporting/reportingAdapter";
import { useReporting } from "../shared/reporting/reportingSource";
import { useT } from "../shared/i18n/i18n";

/**
 * Reporting Mode: read-only, full-screen view for the operator panel
 * (Getac ZX80, docked, portrait mount -- TABLET-MOUNT decision 2 Nov 2026).
 * Layout is a single column. Scope is base and navigation only. No command
 * of any kind: it only renders the shared reporting adapter (shared/reporting).
 *
 * Type sizes, layout constants, state colors and touch target minimums are
 * defined below. Proposed defaults are noted; accepted defaults are not.
 */

// Type sizes in CSS px for a 1200 x 1920 portrait page. Proposed defaults.
const TYPE_PX = { state: 120, reason: 44, action: 32, body: 28, row: 24, small: 20 };

// Every size is a share of the page width (px / 1200 of 100vw), capped at
// its 1200 px value. Text keeps the same physical size whatever the display
// scale Chrome uses on the panel (the CSS px count is not confirmed).
export const fluid = (px) => `min(${px}px, ${+(px / 12).toFixed(4)}vw)`;
const TYPE = Object.fromEntries(Object.entries(TYPE_PX).map(([k, px]) => [k, fluid(px)]));

// Spacing scales the same way: 40 px side padding at 1200 px wide.
const SIDE = fluid(40);

// The longest unbroken state word, in English or German, in characters.
// German RECOVERING (WIEDERHERSTELLUNG) is 17. A test checks every state
// word against this, so a longer translation fails the test.
export const STATE_MAX_TOKEN_CHARS = 17;

// RobotoMono is 0.6 em per character: 17 characters are 10.2 em, plus the
// icon (0.6 em) and the gap come to about 11.1 em. Dividing the available
// width by 12 leaves room for bold glyphs at every page width.
export const STATE_FONT_SIZE = `min(${TYPE_PX.state}px, calc((100vw - 2 * ${SIDE}) / 12))`;

// Warnings listed on the page. More are counted, never dropped silently.
// Proposed default (fits the portrait page with long, wrapped warnings).
export const MAX_WARNINGS_SHOWN = 8;

// Long diagnostic names and reasons can be single tokens; let them wrap.
const WRAP = { overflowWrap: "anywhere" };

// Long press on the corner control to leave Reporting Mode. Proposed default.
export const EXIT_HOLD_MS = 2000;

// Minimum touch target height for every interactive control (48 CSS px,
// matching the Material Design and WCAG 2.5.5 guidance for touch targets).
const MIN_TOUCH_H = 48;

// State icons are inline SVG drawn in currentColor, so they look the same on
// every device and take the state's accent color. Text glyphs did not: emoji
// keep their own colors, and some symbols are missing from RobotoMono.
const STROKE = { fill: "none", stroke: "currentColor", strokeLinecap: "round", strokeLinejoin: "round" };
const ICON_PATHS = {
  // Filled circle
  [DISPLAY.READY]: <circle cx="12" cy="12" r="9" fill="currentColor" />,
  // Play triangle
  [DISPLAY.EXECUTING]: <path d="M7 4.5v15l12.5-7.5z" fill="currentColor" />,
  // Pause bars
  [DISPLAY.WAITING]: (
    <>
      <rect x="5.5" y="4.5" width="4.5" height="15" rx="1" fill="currentColor" />
      <rect x="14" y="4.5" width="4.5" height="15" rx="1" fill="currentColor" />
    </>
  ),
  // No entry: ring with a bar
  [DISPLAY.BLOCKED]: (
    <>
      <circle cx="12" cy="12" r="8.5" strokeWidth="3" {...STROKE} />
      <path d="M7.5 12h9" strokeWidth="3" {...STROKE} />
    </>
  ),
  // Circular arrow
  [DISPLAY.RECOVERING]: (
    <>
      <path d="M19 12a7 7 0 1 1-2.1-5" strokeWidth="2.6" {...STROKE} />
      <path d="M20 3.5v6h-6z" fill="currentColor" />
    </>
  ),
  // Ring with an exclamation mark
  [DISPLAY.NEEDS_ASSISTANCE]: (
    <>
      <circle cx="12" cy="12" r="9" strokeWidth="2.4" {...STROKE} />
      <path d="M12 7v6" strokeWidth="2.6" {...STROKE} />
      <circle cx="12" cy="16.6" r="1.5" fill="currentColor" />
    </>
  ),
  // Filled octagon
  [DISPLAY.SAFETY_STOPPED]: (
    <path d="M8.3 2.5h7.4l5.8 5.8v7.4l-5.8 5.8H8.3l-5.8-5.8V8.3z" fill="currentColor" />
  ),
  // Warning triangle
  [DISPLAY.FAULTED]: (
    <>
      <path d="M12 3.2 21.8 20.5H2.2z" strokeWidth="2.2" {...STROKE} />
      <path d="M12 9.5v5" strokeWidth="2.4" {...STROKE} />
      <circle cx="12" cy="17.4" r="1.3" fill="currentColor" />
    </>
  ),
  // Filled square
  [DISPLAY.STOPPED_SOFTWARE]: <rect x="5" y="5" width="14" height="14" rx="1.5" fill="currentColor" />,
  // Ring with a question mark
  [DISPLAY.UNKNOWN]: (
    <>
      <circle cx="12" cy="12" r="9" strokeWidth="2.4" {...STROKE} />
      <path d="M9.6 9.6a2.5 2.5 0 1 1 3.4 2.3c-.6.3-1 .8-1 1.5v.4" strokeWidth="2.4" {...STROKE} />
      <circle cx="12" cy="16.9" r="1.3" fill="currentColor" />
    </>
  ),
  // Cross
  [DISPLAY.DISCONNECTED]: <path d="M6 6l12 12M18 6 6 18" strokeWidth="3" {...STROKE} />,
};

const STATE_STYLE = {
  [DISPLAY.READY]:            { bg: "#0b3d1e", fg: "#ffffff", accent: "#34d399" },
  [DISPLAY.EXECUTING]:        { bg: "#0b2a4a", fg: "#ffffff", accent: "#60a5fa" },
  [DISPLAY.WAITING]:          { bg: "#2a2a2a", fg: "#ffffff", accent: "#e5e7eb" },
  [DISPLAY.BLOCKED]:          { bg: "#4a3300", fg: "#ffffff", accent: "#fbbf24" },
  [DISPLAY.RECOVERING]:       { bg: "#063c3b", fg: "#ffffff", accent: "#5eead4" },
  [DISPLAY.NEEDS_ASSISTANCE]: { bg: "#5a2a00", fg: "#ffffff", accent: "#fb923c" },
  [DISPLAY.SAFETY_STOPPED]:   { bg: "#3b174d", fg: "#ffffff", accent: "#d8b4fe" },
  [DISPLAY.FAULTED]:          { bg: "#5c0b0b", fg: "#ffffff", accent: "#f87171" },
  [DISPLAY.STOPPED_SOFTWARE]: { bg: "#3a0a0a", fg: "#ffffff", accent: "#fca5a5" },
  [DISPLAY.UNKNOWN]:          { bg: "#1f1f1f", fg: "#ffffff", accent: "#9ca3af" },
  [DISPLAY.DISCONNECTED]:     { bg: "#111111", fg: "#ffffff", accent: "#d1d5db" },
};

export const StateIcon = ({ display }) => {
  const known = ICON_PATHS[display] ? display : DISPLAY.UNKNOWN;
  return (
    <svg
      data-testid="reporting-state-icon"
      data-icon={known}
      viewBox="0 0 24 24"
      aria-hidden="true"
      focusable="false"
      style={{ width: "0.6em", height: "0.6em", flexShrink: 0 }}
    >
      {ICON_PATHS[known]}
    </svg>
  );
};

export const StateHeading = ({ display, t }) => {
  const style = STATE_STYLE[display] || STATE_STYLE[DISPLAY.UNKNOWN];
  return (
    <h1
      data-testid="reporting-state"
      className="flex items-center gap-3 font-bold leading-none"
      style={{ fontSize: STATE_FONT_SIZE, color: style.accent }}
    >
      <StateIcon display={display} />
      <span>{t(display)}</span>
    </h1>
  );
};

// --- Demo Mode label ---

// The app's Demo Mode banner lives in AppLayout, and this page renders
// outside it, so the page shows its own label whenever Demo Mode is on.
// Demo Mode feeds every value here from browser-side sample telemetry.
const DemoLabel = ({ t }) => {
  const { config } = useRuntimeConfig();
  if (!config?.demoMode) return null;
  return (
    <div
      data-testid="reporting-demo-banner"
      role="status"
      className="border-b-2 border-sky-300 bg-sky-950 font-bold text-sky-100"
      style={{ fontSize: TYPE.body, padding: `${fluid(12)} ${SIDE}`, ...WRAP }}
    >
      <span className="uppercase tracking-wider">{t("Demo mode")}</span>
      {": "}
      {t("Every value on this page is simulated. No robot is connected.")}
    </div>
  );
};

// --- Wake lock ---

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

// --- Announcements ---

// The announcement text is always worked out and shown on screen, so it is
// visible with speech off (the default). The speech engine is only called
// when `speak` is true. Because the memory advances either way, switching
// speech on never reads out what is already on screen, and a disconnect and
// reconnect does not repeat anything.
//
// On screen the text carries the panel time it was made, so it reads as a
// past event, not as the current state. When the state changes without an
// announcement (back to the state spoken before a disconnect), the screen
// text is replaced by the current state, so it never names a state the page
// has left. Nothing is spoken for that replacement.
const useAnnouncements = (view, speak) => {
  const memory = useRef(createAnnouncementMemory());
  const [last, setLast] = useState(null); // { text, display, atWallMs }

  useEffect(() => {
    if (!view) return;
    const announcements = nextAnnouncements(memory.current, view);
    if (!announcements.length) {
      setLast((prev) =>
        prev && prev.display !== view.display
          ? { text: stateText(view), display: view.display, atWallMs: Date.now() }
          : prev,
      );
      return;
    }
    setLast({ text: announcements.join(" "), display: view.display, atWallMs: Date.now() });
    if (!speak) return;
    announcements.forEach((text) => {
      window.speechSynthesis.speak(new window.SpeechSynthesisUtterance(text));
    });
  }, [view, speak]);

  return last;
};

// --- Exit control ---

const ExitControl = ({ t }) => {
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
      aria-label={t("Hold to exit Reporting Mode")}
      onPointerDown={start}
      onPointerUp={cancel}
      onPointerLeave={cancel}
      onPointerCancel={cancel}
      onContextMenu={(e) => e.preventDefault()}
      style={{ fontSize: TYPE.small, minHeight: MIN_TOUCH_H }}
      className={`shrink-0 rounded-lg border px-4 ${
        holding ? "border-white bg-white/20" : "border-white/30 bg-black/30"
      } select-none text-white/80`}
    >
      {holding ? t("Keep holding to exit") : t("Hold to exit")}
    </button>
  );
};

// --- Speech toggle ---

const SpeechToggle = ({ supported, enabled, onToggle, t }) => {
  if (!supported) {
    return (
      <span
        data-testid="speech-not-supported"
        style={{ fontSize: TYPE.small }}
        className="text-white/60"
      >
        {t("Speech not supported")}
      </span>
    );
  }
  return (
    <button
      type="button"
      aria-pressed={enabled}
      data-testid="speech-toggle"
      onClick={onToggle}
      style={{ fontSize: TYPE.small, minHeight: MIN_TOUCH_H }}
      className="rounded-lg border border-white/30 px-4 text-white"
    >
      {enabled ? t("Speech on") : t("Speech off")}
    </button>
  );
};

// --- Source row freshness label ---

const freshnessLabel = (row, t) => {
  const { freshness, ageSeconds } = row;
  if (!freshness || freshness === "Fresh") return "";
  if (freshness === "Stale") return ` ${t("Stale")} (${ageSeconds} ${t("s")})`;
  if (freshness === "No data") return ` ${t("(no data)")}`;
  if (freshness === "Invalid") return ` ${t("(invalid)")}`;
  return ` (${freshness.toLowerCase()})`;
};

// --- Main page ---

const ReportingPage = () => {
  const { t } = useT();
  const view = useReporting();
  const wakeLock = useWakeLock();

  const speechSupported = typeof window !== "undefined" && "speechSynthesis" in window;
  const [speechOn, setSpeechOn] = useState(false);
  const lastAnnouncement = useAnnouncements(view, speechOn && speechSupported);
  const toggleSpeech = () => {
    if (speechOn) window.speechSynthesis?.cancel?.();
    setSpeechOn(!speechOn);
  };

  if (!view) return null;
  const style = STATE_STYLE[view.display] || STATE_STYLE[DISPLAY.UNKNOWN];
  const disconnected = view.display === DISPLAY.DISCONNECTED;
  // While disconnected, the task and progress are what was known before the
  // connection was lost. They are dimmed and labelled, never shown as live.
  const lastKnownLabel = disconnected && (view.task || view.progress) && (
    <p
      data-testid="reporting-task-last-known"
      className="font-bold uppercase tracking-wider text-white/80"
      style={{ fontSize: TYPE.small }}
    >
      {t("Last known, not live")}
    </p>
  );

  return (
    <div
      data-testid="reporting-page"
      data-state={view.display}
      className="relative flex h-screen w-screen flex-col overflow-hidden font-[RobotoMono]"
      style={{ background: style.bg, color: style.fg }}
    >
      <DemoLabel t={t} />

      {/* Header: scope label + exit control */}
      <header
        data-testid="reporting-header"
        className="flex items-start justify-between"
        style={{ fontSize: TYPE.body, gap: fluid(24), padding: `${fluid(24)} ${SIDE} 0` }}
      >
        <div className="flex min-w-0 flex-col gap-1">
          <span className="font-semibold">{t("Base and navigation")}</span>
          <span className="text-white/60" style={{ fontSize: TYPE.small }}>
            {t(
              "Navigation status covers navigation only. It is not whole-robot readiness or E-stop state."
            )}
          </span>
        </div>
        <ExitControl t={t} />
      </header>

      {/* Main: state word, reason, next action, task, progress, result, warnings.
          If a long reason still does not fit, the middle scrolls; the header
          and the source rows stay on screen. */}
      <main
        data-testid="reporting-main"
        className="flex min-h-0 flex-1 flex-col overflow-y-auto"
        style={{ gap: SIDE, padding: `${fluid(24)} ${SIDE}` }}
      >
        <section aria-label="Robot state" style={WRAP}>
          {disconnected && (
            <div
              data-testid="reporting-disconnected-banner"
              role="status"
              className="mb-4 border-2 border-white/60 bg-black px-4 py-3 font-bold"
              style={{ fontSize: TYPE.body }}
            >
              {view.disconnectedAtWallMs !== null && view.disconnectedAtWallMs !== undefined ? (
                <>
                  {t("Disconnected since")}{" "}
                  <span data-testid="reporting-disconnected-since">
                    {formatClockTime(view.disconnectedAtWallMs)}
                  </span>
                </>
              ) : (
                t("Disconnected")
              )}{" "}
              · {t("Last known state")}:{" "}
              <span data-testid="reporting-last-known-state">
                {t(view.lastKnownDisplay || DISPLAY.UNKNOWN)}
              </span>
            </div>
          )}
          {!disconnected && view.staleSources.length > 0 && (
            <div
              data-testid="reporting-stale-banner"
              role="status"
              className="mb-4 border-2 border-amber-300 bg-amber-950 px-4 py-3 font-bold text-amber-100"
              style={{ fontSize: TYPE.body }}
            >
              {t("Stale data")}: {t("Connection is up; source data is old")}
            </div>
          )}
          <StateHeading display={view.display} t={t} />
          <p
            data-testid="reporting-reason"
            className="mt-6 font-semibold"
            style={{ fontSize: TYPE.reason }}
          >
            {view.reason}
          </p>
          <p
            data-testid="reporting-next"
            className="mt-4 text-white/90"
            style={{ fontSize: TYPE.action }}
          >
            {t("Next:")} {view.nextAction}
          </p>
          {view.completion && (
            <p
              data-testid="reporting-completion"
              className="mt-6 font-semibold"
              style={{ fontSize: TYPE.reason, color: "#34d399" }}
            >
              {/* The adapter's completion text is English ("Completed: <name>"),
                  so the prefix is stripped and translated here. Known gap until
                  the shared #21 adapter carries structured text. */}
              {t("Completed:")} {view.completion.replace(/^Completed:\s*/i, "")}
            </p>
          )}
        </section>

        <section
          aria-label="Task and result"
          style={{ fontSize: TYPE.body, ...WRAP }}
          className="flex min-w-0 flex-col gap-5"
        >
          <div
            data-testid="reporting-task-block"
            className={`flex flex-col gap-5 ${lastKnownLabel ? "opacity-50" : ""}`}
          >
            {lastKnownLabel}
            <div data-testid="reporting-task">
              <p className="text-white/60" style={{ fontSize: TYPE.small }}>
                {t("Current task")}
              </p>
              {view.task ? (
                <p>
                  {view.task.label}
                  {view.task.step ? `, ${view.task.step}` : ""}
                </p>
              ) : (
                <p className="text-white/70">{t("None")}</p>
              )}
            </div>

            {view.progress && (
              <div data-testid="reporting-progress">
                <p className="text-white/60" style={{ fontSize: TYPE.small }}>
                  {t("Progress")}
                </p>
                <div
                  role="progressbar"
                  aria-valuenow={view.progress.percent}
                  aria-valuemin={0}
                  aria-valuemax={100}
                  className="mt-2 h-6 w-full overflow-hidden rounded-full"
                  style={{ background: "rgba(255, 255, 255, 0.2)" }}
                >
                  <div
                    className="h-full"
                    style={{ width: `${view.progress.percent}%`, background: "#ffffff" }}
                  />
                </div>
                <p className="mt-2">
                  {view.progress.distanceRemaining.toFixed(1)} {t("m remaining")}
                  {Number.isFinite(view.progress.etaSeconds)
                    ? `, ${t("about")} ${Math.round(view.progress.etaSeconds)} ${t("s")}`
                    : ""}
                  {view.progress.recoveries
                    ? `, ${view.progress.recoveries} ${t("recoveries")}`
                    : ""}
                </p>
              </div>
            )}
          </div>

          <div data-testid="reporting-result">
            <p className="text-white/60" style={{ fontSize: TYPE.small }}>
              {t("Last result")}
            </p>
            {view.result ? (
              <p>
                {view.result.name}:{" "}
                {view.result.invalid
                  ? t("Result unknown (invalid data)")
                  : view.result.outcome}
                {view.result.reason ? `. ${view.result.reason}` : ""}
              </p>
            ) : (
              <p className="text-white/70">{t("No result yet")}</p>
            )}
          </div>

          <div data-testid="reporting-warnings">
            <p className="text-white/60" style={{ fontSize: TYPE.small }}>
              {t("Warnings")}
            </p>
            {view.warnings.length ? (
              <ul className="list-disc pl-8">
                {view.warnings.slice(0, MAX_WARNINGS_SHOWN).map((w) => (
                  <li key={w}>{w}</li>
                ))}
                {view.warnings.length > MAX_WARNINGS_SHOWN && (
                  <li
                    data-testid="reporting-warnings-more"
                    className="text-white/70"
                  >
                    {view.warnings.length - MAX_WARNINGS_SHOWN} {t("more not shown")}
                  </li>
                )}
              </ul>
            ) : (
              <p className="text-white/70">{t("None")}</p>
            )}
          </div>
        </section>
      </main>

      {/* Footer: source rows + last announcement + wake lock + speech toggle */}
      <footer
        data-testid="reporting-footer"
        className="grid grid-cols-2 border-t border-white/20"
        style={{
          fontSize: TYPE.row,
          columnGap: SIDE,
          rowGap: fluid(8),
          padding: `${fluid(16)} ${SIDE}`,
        }}
      >
        {view.rows.map((row) => (
          <div
            key={row.label}
            className={`flex justify-between gap-4 ${row.freshness === "Stale" ? "opacity-50" : ""}`}
            data-testid={`row-${row.label}`}
            data-freshness={row.freshness || undefined}
          >
            <span className="text-white/70">{t(row.label)}</span>
            <span className="font-semibold">
              {t(row.value)}
              {freshnessLabel(row, t)}
            </span>
          </div>
        ))}
        <p
          data-testid="reporting-speech-text"
          role="status"
          aria-live="polite"
          className="col-span-2 mt-2"
          style={{ fontSize: TYPE.small, ...WRAP }}
        >
          {t("Last announcement")}
          {lastAnnouncement ? (
            <>
              {" ("}
              <span data-testid="reporting-speech-time">
                {formatClockTime(lastAnnouncement.atWallMs)}
              </span>
              {"): "}
              {lastAnnouncement.text}
            </>
          ) : (
            `: ${t("No announcement yet")}`
          )}
        </p>
        <div
          className="col-span-2 flex items-center justify-between text-white/60"
          style={{ fontSize: TYPE.small }}
        >
          <span>
            {t("Screen wake lock")}: {t(wakeLock)}
          </span>
          <SpeechToggle
            supported={speechSupported}
            enabled={speechOn}
            onToggle={toggleSpeech}
            t={t}
          />
        </div>
      </footer>
    </div>
  );
};

export default ReportingPage;
