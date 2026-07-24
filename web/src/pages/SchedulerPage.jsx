import React, { useEffect, useState } from "react";

import {
  getSchedules,
  subscribeSchedules,
  addSchedule,
  updateSchedule,
  removeSchedule,
} from "../shared/schedules/schedules";
import { getMissions, subscribeMissions } from "../shared/missions/missions";
import useSavedWaypoints from "../shared/hooks/useSavedWaypoints";
import { DashboardCard, EmptyState, SectionHeader } from "../shared/ui/Dashboard";

const EMPTY = { name: "", time: "08:00", repeat: "daily", target: "home" };

const inputClass =
  "rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover outline-none focus:border-themeBlue";

/**
 * Manage time-triggered navigation missions. Fires while a browser tab is
 * open (see SchedulerRunner) — a convenience scheduler, not robot-side cron.
 */
const SchedulerPage = () => {
  const { waypoints } = useSavedWaypoints();
  const [schedules, setSchedules] = useState(getSchedules);
  const [missions, setMissions] = useState(getMissions);
  const [form, setForm] = useState(EMPTY);

  useEffect(() => subscribeSchedules(setSchedules), []);
  useEffect(() => subscribeMissions(setMissions), []);

  const set = (k) => (e) => setForm((p) => ({ ...p, [k]: e.target.value }));

  const add = () => {
    if (!form.name.trim() || !form.time) return;
    let action;
    if (form.target === "home") action = { type: "home" };
    else if (form.target.startsWith("mission:")) {
      action = { type: "mission", missionId: form.target.slice("mission:".length) };
    } else {
      action = { type: "waypoint", waypointId: Number(form.target.slice("wp:".length)) };
    }
    addSchedule({ name: form.name.trim(), time: form.time, repeat: form.repeat, action });
    setForm(EMPTY);
  };

  const describe = (s) => {
    if (s.action?.type === "home") return "Go home (0, 0)";
    if (s.action?.type === "mission") {
      const mission = missions.find((m) => m.id === s.action?.missionId);
      return mission ? `Run mission "${mission.name}"` : "Run mission (missing)";
    }
    const wp = waypoints.find((w) => w.id === s.action?.waypointId);
    return wp ? `Navigate to "${wp.name}"` : "Navigate to (missing waypoint)";
  };

  return (
    <div className="sectionHeight space-y-5 py-4 sm:py-6">
      <SectionHeader
        eyebrow="Autonomy"
        title="Scheduler"
        description="Trigger navigation at set times. Runs while a browser tab is open — it is not robot-side cron, so keep a tab running for schedules to fire."
      />

      <DashboardCard className="p-0">
        {schedules.length === 0 ? (
          <EmptyState
            title="No schedules"
            description="Add one below — e.g. send the robot to its dock every evening."
          />
        ) : (
          <ul className="divide-y divide-borderSubtle/30 font-[RobotoMono]">
            {schedules.map((s) => (
              <li key={s.id} className="flex flex-wrap items-center gap-3 px-4 py-3">
                <span className="w-14 shrink-0 text-lg font-semibold text-themeBlue">
                  {s.time}
                </span>
                <div className="min-w-0 flex-1">
                  <p className="text-sm font-semibold text-textWhiteHover">{s.name}</p>
                  <p className="text-xs text-themeTextGray">
                    {describe(s)} · {s.repeat}
                  </p>
                </div>
                <button
                  onClick={() => updateSchedule(s.id, { enabled: !s.enabled })}
                  className={`rounded-lg border px-3 py-1 text-xs transition-colors ${
                    s.enabled
                      ? "border-statusGreen/50 text-statusGreen"
                      : "border-borderSubtle text-themeTextGray"
                  }`}
                >
                  {s.enabled ? "Enabled" : "Disabled"}
                </button>
                <button
                  onClick={() => removeSchedule(s.id)}
                  aria-label={`Delete ${s.name}`}
                  className="px-1 text-themeTextGray hover:text-statusRed"
                >
                  ×
                </button>
              </li>
            ))}
          </ul>
        )}
      </DashboardCard>

      <DashboardCard className="p-4 font-[RobotoMono]">
        <p className="mb-3 text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Add schedule
        </p>
        <div className="grid grid-cols-1 gap-2 sm:grid-cols-[1.4fr_0.7fr_0.9fr_1.2fr_auto]">
          <input
            className={inputClass}
            value={form.name}
            onChange={set("name")}
            placeholder="Name (e.g. Nightly dock)"
          />
          <input className={inputClass} type="time" value={form.time} onChange={set("time")} />
          <select className={inputClass} value={form.repeat} onChange={set("repeat")}>
            <option value="daily">Daily</option>
            <option value="once">Once</option>
          </select>
          <select className={inputClass} value={form.target} onChange={set("target")}>
            <option value="home">Go home</option>
            {waypoints.map((w) => (
              <option key={`wp:${w.id}`} value={`wp:${w.id}`}>
                {w.name}
              </option>
            ))}
            {missions.map((m) => (
              <option key={`mission:${m.id}`} value={`mission:${m.id}`}>
                Mission: {m.name}
              </option>
            ))}
          </select>
          <button
            onClick={add}
            className="rounded-lg border border-themeBlue bg-themeBlue/10 px-4 py-2 text-sm font-semibold text-themeBlue transition-colors hover:bg-themeBlue hover:text-white"
          >
            Add
          </button>
        </div>
        <p className="mt-2 text-[11px] text-themeTextGray/70">
          Targets are saved waypoints (add them on the Map page), home, or a
          whole mission (build one on the Missions page).
        </p>
      </DashboardCard>
    </div>
  );
};

export default SchedulerPage;
