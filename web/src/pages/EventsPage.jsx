import React, { useMemo, useState } from "react";

import useEventLog from "../shared/hooks/useEventLog";
import { EVENT_SEVERITY, clearEvents } from "../shared/events/eventLog";
import { DashboardCard, EmptyState, SectionHeader } from "../shared/ui/Dashboard";

const TYPE_FILTERS = ["all", "navigation", "docking", "battery", "safety", "system"];
const SEVERITY_FILTERS = ["all", "info", "success", "warning", "error"];

const fmt = (iso) => {
  const d = new Date(iso);
  return Number.isNaN(d.getTime()) ? "" : d.toLocaleString(undefined, { hour12: false });
};

/**
 * Reviewable timeline of robot events (nav outcomes, docking, low battery,
 * E-stops…) recorded by EventRecorder into the shared, persisted event log.
 * Filterable by type and severity; survives reloads via localStorage.
 */
const EventsPage = () => {
  const events = useEventLog();
  const [type, setType] = useState("all");
  const [severity, setSeverity] = useState("all");

  const filtered = useMemo(
    () =>
      events.filter(
        (e) =>
          (type === "all" || e.type === type) &&
          (severity === "all" || e.severity === severity),
      ),
    [events, type, severity],
  );

  const exportJson = () => {
    const blob = new Blob([JSON.stringify(events, null, 2)], {
      type: "application/json",
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `openamr-events-${Date.now()}.json`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const pill = (value, current, setter) => (
    <button
      key={value}
      onClick={() => setter(value)}
      className={`rounded-lg border px-2.5 py-1 text-xs capitalize transition-colors ${
        current === value
          ? "border-themeBlue bg-themeBlue/10 text-themeBlue"
          : "border-borderSubtle text-themeTextGray hover:border-themeBlue"
      }`}
    >
      {value}
    </button>
  );

  return (
    <div className="sectionHeight space-y-5 py-4 sm:py-6">
      <SectionHeader
        eyebrow="Audit"
        title="Events"
        description="A persisted timeline of navigation, docking, battery and safety events — reviewable after the fact. Kept locally in the browser."
        action={
          <div className="flex gap-2">
            <button
              onClick={exportJson}
              disabled={events.length === 0}
              className="rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeTextGray transition-colors hover:border-themeBlue hover:text-themeBlue disabled:opacity-40"
            >
              Export
            </button>
            <button
              onClick={clearEvents}
              disabled={events.length === 0}
              className="rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeTextGray transition-colors hover:border-statusRed/50 hover:text-statusRed disabled:opacity-40"
            >
              Clear
            </button>
          </div>
        }
      />

      <DashboardCard className="p-3 font-[RobotoMono]">
        <div className="flex flex-wrap items-center gap-3">
          <span className="text-[10px] uppercase tracking-wider text-themeTextGray">Type</span>
          {TYPE_FILTERS.map((v) => pill(v, type, setType))}
          <span className="ml-3 text-[10px] uppercase tracking-wider text-themeTextGray">Severity</span>
          {SEVERITY_FILTERS.map((v) => pill(v, severity, setSeverity))}
        </div>
      </DashboardCard>

      <DashboardCard className="p-0">
        {filtered.length === 0 ? (
          <EmptyState
            title={events.length === 0 ? "No events recorded yet" : "No events match the filters"}
            description={
              events.length === 0
                ? "Navigation, docking and battery events will appear here as they happen."
                : "Try widening the type or severity filter."
            }
          />
        ) : (
          <ul className="divide-y divide-borderSubtle/30 font-[RobotoMono]">
            {filtered.map((e) => {
              const sev = EVENT_SEVERITY[e.severity] || EVENT_SEVERITY.info;
              return (
                <li key={e.id} className="flex items-center gap-3 px-4 py-2 text-sm">
                  <span className={`h-2 w-2 shrink-0 rounded-full ${sev.dot}`} />
                  <span className="w-40 shrink-0 text-xs text-themeTextGray">{fmt(e.ts)}</span>
                  <span className="w-20 shrink-0 text-[11px] uppercase tracking-wider text-themeTextGray/70">
                    {e.type}
                  </span>
                  <span className={`min-w-0 flex-1 ${sev.color}`}>{e.message}</span>
                </li>
              );
            })}
          </ul>
        )}
      </DashboardCard>
    </div>
  );
};

export default EventsPage;
