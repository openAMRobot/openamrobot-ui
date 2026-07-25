import React, { useEffect, useState } from "react";

import useSavedWaypoints from "../shared/hooks/useSavedWaypoints";
import useMissionRun from "../shared/hooks/useMissionRun";
import {
  getMissions,
  subscribeMissions,
  addMission,
  removeMission,
  addStep,
  removeStep,
  moveStep,
} from "../shared/missions/missions";
import { requestStart, requestStop } from "../shared/missions/missionRunner";
import { DashboardCard, EmptyState, SectionHeader } from "../shared/ui/Dashboard";

const inputClass =
  "rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover outline-none focus:border-themeBlue";

const STEP_TYPES = [
  { value: "waypoint", label: "Go to waypoint" },
  { value: "home", label: "Go home" },
  { value: "wait", label: "Wait" },
  { value: "dock", label: "Dock" },
  { value: "undock", label: "Undock" },
];

const STATUS_STYLE = {
  running: "text-statusBlue",
  succeeded: "text-statusGreen",
  failed: "text-statusRed",
  stopped: "text-statusYellow",
};

const describeStep = (step, waypoints) => {
  switch (step.type) {
    case "waypoint": {
      const wp = waypoints.find((w) => w.id === step.waypointId);
      return `Go to "${wp?.name || "(missing waypoint)"}"`;
    }
    case "home":
      return "Go home";
    case "wait":
      return `Wait ${step.seconds}s`;
    case "dock":
      return "Dock";
    case "undock":
      return "Undock";
    default:
      return step.type;
  }
};

/**
 * Author and run multi-step missions — ordered sequences of waypoint/home
 * navigation, timed waits, and dock/undock actions. Runs live via the
 * headless MissionRunner (mounted in AppLayout) while a browser tab stays
 * open, same "not robot-side autonomy" honesty as the Scheduler page — and
 * can itself be time-triggered from there (a schedule's target can be a
 * mission, not just a single waypoint).
 */
const MissionsPage = () => {
  const { waypoints } = useSavedWaypoints();
  const run = useMissionRun();
  const [missions, setMissions] = useState(getMissions);
  const [selectedId, setSelectedId] = useState(null);
  const [newName, setNewName] = useState("");
  const [stepForm, setStepForm] = useState({ type: "waypoint", waypointId: "", seconds: "5" });

  useEffect(() => subscribeMissions(setMissions), []);

  const selected = missions.find((m) => m.id === selectedId) || null;

  const createMission = () => {
    const trimmed = newName.trim();
    if (!trimmed) return;
    const mission = addMission(trimmed);
    setNewName("");
    setSelectedId(mission.id);
  };

  const addStepToSelected = () => {
    if (!selected) return;
    if (stepForm.type === "waypoint") {
      if (!stepForm.waypointId) return;
      addStep(selected.id, { type: "waypoint", waypointId: Number(stepForm.waypointId) });
    } else if (stepForm.type === "wait") {
      const seconds = Math.max(1, parseInt(stepForm.seconds, 10) || 5);
      addStep(selected.id, { type: "wait", seconds });
    } else {
      addStep(selected.id, { type: stepForm.type });
    }
  };

  return (
    <div className="sectionHeight space-y-5 py-4 sm:py-6">
      <SectionHeader
        eyebrow="Autonomy"
        title="Missions"
        description="Chain waypoints, waits, and dock/undock into one sequence. Runs while a browser tab is open — not robot-side autonomy — and can be triggered from the Scheduler page too."
      />

      <DashboardCard className="p-0">
        {missions.length === 0 ? (
          <EmptyState
            title="No missions yet"
            description="Create one below, then add steps — e.g. go to the loading bay, wait 10s, then dock."
          />
        ) : (
          <ul className="divide-y divide-borderSubtle/30 font-[RobotoMono]">
            {missions.map((m) => {
              const isSelected = selectedId === m.id;
              const isRunning = run?.missionId === m.id && run.status === "running";
              const activeRun = run?.missionId === m.id ? run : null;
              return (
                <li key={m.id} className="px-4 py-3">
                  <div className="flex flex-wrap items-center gap-3">
                    <button
                      onClick={() => setSelectedId(isSelected ? null : m.id)}
                      className="min-w-0 flex-1 text-left"
                    >
                      <p className="text-sm font-semibold text-textWhiteHover">
                        {m.name}
                        {isSelected && (
                          <span className="ml-2 text-themeBlue">▾</span>
                        )}
                      </p>
                      <p className="text-xs text-themeTextGray">
                        {m.steps.length} step{m.steps.length === 1 ? "" : "s"}
                        {activeRun && (
                          <span className={`ml-2 ${STATUS_STYLE[activeRun.status] || ""}`}>
                            · {activeRun.status}
                            {activeRun.status === "running" &&
                              ` (step ${activeRun.stepIndex + 1}/${m.steps.length})`}
                          </span>
                        )}
                      </p>
                    </button>
                    {isRunning ? (
                      <button
                        onClick={requestStop}
                        className="rounded-lg border border-statusRed px-3 py-1 text-xs text-statusRed transition-colors hover:bg-statusRed hover:text-white"
                      >
                        Stop
                      </button>
                    ) : (
                      <button
                        onClick={() => {
                          if (!window.confirm("This will start moving the robot through this mission's steps. Continue?")) return;
                          requestStart(m.id);
                        }}
                        disabled={!m.steps.length || run?.status === "running"}
                        className="rounded-lg border border-themeBlue px-3 py-1 text-xs text-themeBlue transition-colors hover:bg-themeBlue hover:text-white disabled:opacity-40"
                      >
                        Run (moves robot)
                      </button>
                    )}
                    <button
                      onClick={() => {
                        removeMission(m.id);
                        if (selectedId === m.id) setSelectedId(null);
                      }}
                      aria-label={`Delete ${m.name}`}
                      className="px-1 text-themeTextGray hover:text-statusRed"
                    >
                      ×
                    </button>
                  </div>

                  {isSelected && (
                    <div className="mt-3 space-y-2 rounded-lg bg-bgSurface p-3">
                      {m.steps.length === 0 ? (
                        <p className="text-xs text-themeTextGray opacity-70">
                          No steps yet — add one below.
                        </p>
                      ) : (
                        <ol className="space-y-1.5">
                          {m.steps.map((step, index) => (
                            <li
                              key={step.id}
                              className="flex items-center gap-2 rounded-lg border border-borderSubtle bg-bgCard px-2.5 py-1.5 text-xs"
                            >
                              <span className="w-5 shrink-0 text-themeTextGray">
                                {index + 1}.
                              </span>
                              <span className="flex-1 text-textWhiteHover">
                                {describeStep(step, waypoints)}
                              </span>
                              {activeRun?.log?.[index] && (
                                <span
                                  className={
                                    activeRun.log[index].ok
                                      ? "text-statusGreen"
                                      : "text-statusRed"
                                  }
                                >
                                  {activeRun.log[index].ok ? "✓" : "✗"}
                                </span>
                              )}
                              <button
                                onClick={() => moveStep(m.id, step.id, -1)}
                                disabled={index === 0}
                                className="text-themeTextGray hover:text-themeBlue disabled:opacity-30"
                                aria-label="Move up"
                              >
                                ↑
                              </button>
                              <button
                                onClick={() => moveStep(m.id, step.id, 1)}
                                disabled={index === m.steps.length - 1}
                                className="text-themeTextGray hover:text-themeBlue disabled:opacity-30"
                                aria-label="Move down"
                              >
                                ↓
                              </button>
                              <button
                                onClick={() => removeStep(m.id, step.id)}
                                className="text-themeTextGray hover:text-statusRed"
                                aria-label="Remove step"
                              >
                                ×
                              </button>
                            </li>
                          ))}
                        </ol>
                      )}

                      <div className="flex flex-wrap items-center gap-2 border-t border-borderSubtle pt-2">
                        <select
                          className={inputClass}
                          value={stepForm.type}
                          onChange={(e) => setStepForm((p) => ({ ...p, type: e.target.value }))}
                        >
                          {STEP_TYPES.map((t) => (
                            <option key={t.value} value={t.value}>
                              {t.label}
                            </option>
                          ))}
                        </select>
                        {stepForm.type === "waypoint" && (
                          <select
                            className={inputClass}
                            value={stepForm.waypointId}
                            onChange={(e) =>
                              setStepForm((p) => ({ ...p, waypointId: e.target.value }))
                            }
                          >
                            <option value="">Choose a waypoint…</option>
                            {waypoints.map((w) => (
                              <option key={w.id} value={w.id}>
                                {w.name}
                              </option>
                            ))}
                          </select>
                        )}
                        {stepForm.type === "wait" && (
                          <input
                            type="number"
                            min="1"
                            className={`${inputClass} w-24`}
                            value={stepForm.seconds}
                            onChange={(e) =>
                              setStepForm((p) => ({ ...p, seconds: e.target.value }))
                            }
                            placeholder="seconds"
                          />
                        )}
                        <button
                          onClick={addStepToSelected}
                          className="rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeTextGray transition-colors hover:border-themeBlue hover:text-themeBlue"
                        >
                          + Add step
                        </button>
                      </div>
                      {stepForm.type === "waypoint" && waypoints.length === 0 && (
                        <p className="text-[11px] text-themeTextGray/70">
                          No saved waypoints yet — add one from the Map page first.
                        </p>
                      )}
                    </div>
                  )}
                </li>
              );
            })}
          </ul>
        )}
      </DashboardCard>

      <DashboardCard className="p-4 font-[RobotoMono]">
        <p className="mb-3 text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          New mission
        </p>
        <div className="flex gap-2">
          <input
            className={`${inputClass} flex-1`}
            value={newName}
            onChange={(e) => setNewName(e.target.value)}
            onKeyDown={(e) => e.key === "Enter" && createMission()}
            placeholder="e.g. Evening patrol"
          />
          <button
            onClick={createMission}
            className="shrink-0 rounded-lg border border-themeBlue bg-themeBlue/10 px-4 py-2 text-sm font-semibold text-themeBlue transition-colors hover:bg-themeBlue hover:text-white"
          >
            Create
          </button>
        </div>
      </DashboardCard>
    </div>
  );
};

export default MissionsPage;
