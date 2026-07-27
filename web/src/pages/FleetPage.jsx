import React, { useCallback, useEffect, useState } from "react";
import { Link } from "react-router-dom";

import { useRuntimeConfig, useRosStatus } from "../app/App";
import { resolveRosbridgeHost } from "../shared/constants/runtimeConfig";
import { DashboardCard, EmptyState, SectionHeader } from "../shared/ui/Dashboard";
import SystemHealth from "../components/SystemHealth";
import LifecycleStatus from "../components/LifecycleStatus";
import useSystemDiagnostics, {
  OVERALL_LABELS,
} from "../shared/hooks/useSystemDiagnostics";

const OVERALL_DOT = {
  0: "bg-statusGreen",
  1: "bg-statusYellow",
  2: "bg-statusRed",
  3: "bg-statusRed",
};

const STORAGE_KEY = "openamrFleet";

const loadFleet = () => {
  try {
    return JSON.parse(localStorage.getItem(STORAGE_KEY) || "[]");
  } catch {
    return [];
  }
};

// Lightweight reachability probe: rosbridge speaks WebSocket, so if we can open
// a socket to ws://host:port the robot's bridge is up. We close immediately —
// this is a liveness ping, not the app's real connection (that's the shared
// ROSLIB.Ros managed in App for the *active* robot only).
const probe = (host, port) =>
  new Promise((resolve) => {
    let settled = false;
    const finish = (v, ws) => {
      if (settled) return;
      settled = true;
      try {
        ws && ws.close();
      } catch {
        // ignore
      }
      resolve(v);
    };
    try {
      const ws = new WebSocket(`ws://${host}:${port}`);
      const timer = setTimeout(() => finish(false, ws), 3000);
      ws.onopen = () => {
        clearTimeout(timer);
        finish(true, ws);
      };
      ws.onerror = () => {
        clearTimeout(timer);
        finish(false, ws);
      };
    } catch {
      resolve(false);
    }
  });

const EMPTY = { name: "", host: "", port: "9090" };

/**
 * Fleet view: a roster of robots (name + rosbridge host/port) with a live
 * reachability dot for each, and one-click switching of the *active* robot —
 * which just repoints the app-wide rosbridge connection (runtime config) at
 * that host, so the rest of the single-robot UI follows.
 */
const FleetPage = () => {
  const { config, updateConfig } = useRuntimeConfig();
  const rosStatus = useRosStatus();
  const { reportHealth, reportLifecycle, issues, overall, overallLabel } =
    useSystemDiagnostics();

  const [fleet, setFleet] = useState(loadFleet);
  const [reach, setReach] = useState({}); // id -> bool | "checking"
  const [form, setForm] = useState(EMPTY);

  const persist = (next) => {
    setFleet(next);
    localStorage.setItem(STORAGE_KEY, JSON.stringify(next));
  };

  const activeHost = resolveRosbridgeHost(config);
  const isActive = (r) =>
    r.host === activeHost && String(r.port) === String(config.rosbridgePort);

  const checkAll = useCallback(async () => {
    const entries = loadFleet();
    setReach((prev) => {
      const next = { ...prev };
      entries.forEach((r) => (next[r.id] = "checking"));
      return next;
    });
    await Promise.all(
      entries.map(async (r) => {
        const ok = await probe(r.host, r.port);
        setReach((prev) => ({ ...prev, [r.id]: ok }));
      }),
    );
  }, []);

  useEffect(() => {
    checkAll();
  }, [checkAll]);

  const addRobot = () => {
    const name = form.name.trim();
    const host = form.host.trim();
    if (!name || !host) return;
    const next = [...fleet, { id: `${Date.now()}`, name, host, port: form.port.trim() || "9090" }];
    persist(next);
    setForm(EMPTY);
    checkAll();
  };

  const removeRobot = (id) => persist(fleet.filter((r) => r.id !== id));

  const connect = (r) =>
    updateConfig({ rosbridgeHost: r.host, rosbridgePort: String(r.port) });

  const dotFor = (r) => {
    if (isActive(r)) {
      return rosStatus === "connected" ? OVERALL_DOT[overall] : "bg-statusYellow";
    }
    const state = reach[r.id];
    if (state === "checking") return "bg-themeTextGray animate-pulse";
    if (state === true) return "bg-statusBlue";
    if (state === false) return "bg-statusRed";
    return "bg-themeTextGray";
  };

  const dotTitleFor = (r) => {
    if (isActive(r)) {
      return rosStatus === "connected"
        ? `Connected — full status: ${overallLabel || OVERALL_LABELS[0]}`
        : "Connecting…";
    }
    const state = reach[r.id];
    if (state === "checking") return "Checking reachability…";
    if (state === true) return "Reachable — can connect";
    if (state === false) return "Unreachable — can't reach this robot";
    return "Not checked yet";
  };

  return (
    <div className="sectionHeight space-y-5 py-4 sm:py-6">
      <SectionHeader
        eyebrow="Fleet"
        title="Robots"
        description="Manage multiple robots and switch which one this UI controls. Reachability is probed directly; the active robot uses the live connection."
        action={
          <button
            onClick={checkAll}
            className="rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeTextGray transition-colors hover:border-themeBlue hover:text-themeBlue"
          >
            Re-check
          </button>
        }
      />

      <DashboardCard className="p-4">
        <div className="flex flex-wrap items-center justify-between gap-2">
          <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
            Active robot health
          </p>
          <div className="flex items-center gap-1.5">
            <span className={`h-2 w-2 rounded-full ${OVERALL_DOT[overall]}`} />
            <span className="text-xs font-semibold text-themeTextGray">
              {rosStatus === "connected" ? overallLabel || OVERALL_LABELS[0] : "Not connected"}
            </span>
          </div>
        </div>
        <p className="mt-1 text-sm text-themeTextGray">
          Detailed health info is only available for the robot you're
          currently connected to. Other robots below just show whether they
          can be reached — click Connect to see their full status.
        </p>

        {issues.length > 0 && (
          <div className="mt-3 flex flex-wrap gap-1.5">
            {issues.map((issue) =>
              issue.linkTo ? (
                <Link
                  key={issue.id}
                  to={issue.linkTo}
                  className="rounded-lg border border-borderSubtle bg-bgSurface px-2.5 py-1.5 text-xs text-textWhiteHover hover:border-themeBlue hover:text-themeBlue"
                >
                  {issue.message}
                </Link>
              ) : (
                <span
                  key={issue.id}
                  className="rounded-lg border border-borderSubtle bg-bgSurface px-2.5 py-1.5 text-xs text-themeTextGray"
                >
                  {issue.message}
                </span>
              ),
            )}
          </div>
        )}

        <div className="mt-4 grid gap-3 sm:grid-cols-2">
          <div className="dashboard-card p-3">
            <SystemHealth compact onHealthChange={reportHealth} />
          </div>
          <div className="dashboard-card p-3">
            <LifecycleStatus compact onStatesChange={reportLifecycle} />
          </div>
        </div>

        <Link
          to="/health"
          className="mt-3 inline-block text-xs text-themeBlue hover:underline"
        >
          Open full Health Centre →
        </Link>
      </DashboardCard>

      <DashboardCard className="p-0">
        {fleet.length === 0 ? (
          <EmptyState
            title="No robots added"
            description="Add a robot below with its address and port."
          />
        ) : (
          <ul className="divide-y divide-borderSubtle/30 font-[RobotoMono]">
            {fleet.map((r) => {
              const active = isActive(r);
              return (
                <li key={r.id} className="flex flex-wrap items-center gap-3 px-4 py-3">
                  <span
                    className={`h-2.5 w-2.5 shrink-0 rounded-full ${dotFor(r)}`}
                    title={dotTitleFor(r)}
                  />
                  <div className="min-w-0 flex-1">
                    <p className="text-sm font-semibold text-textWhiteHover">
                      {r.name}
                      {active && (
                        <span className="ml-2 rounded border border-themeBlue/40 px-1.5 py-0.5 text-[10px] text-themeBlue">
                          active
                        </span>
                      )}
                    </p>
                    <p className="text-xs text-themeTextGray">
                      <span title={`ws://${r.host}:${r.port}`}>
                        {r.host}:{r.port}
                      </span>
                      {active && rosStatus === "connected" && (
                        <>
                          {" · "}
                          <Link to="/health" className="hover:text-themeBlue hover:underline">
                            {overallLabel || OVERALL_LABELS[0]}
                            {issues.length > 0 && ` (${issues.length})`}
                          </Link>
                        </>
                      )}
                      {reach[r.id] === false && !active && " · unreachable"}
                      {reach[r.id] === true && !active && (
                        <span title="A quick network check, not a full status check.">
                          {" "}
                          · reachable, but not fully checked
                        </span>
                      )}
                    </p>
                  </div>
                  <button
                    onClick={() => connect(r)}
                    disabled={active}
                    className="rounded-lg border border-themeBlue px-3 py-1 text-xs text-themeBlue transition-colors hover:bg-themeBlue hover:text-white disabled:opacity-40"
                  >
                    {active ? "Connected" : "Connect"}
                  </button>
                  <button
                    onClick={() => removeRobot(r.id)}
                    aria-label={`Remove ${r.name}`}
                    className="px-1 text-themeTextGray hover:text-statusRed"
                  >
                    ×
                  </button>
                </li>
              );
            })}
          </ul>
        )}
      </DashboardCard>

      <DashboardCard className="p-4 font-[RobotoMono]">
        <p className="mb-3 text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Add robot
        </p>
        <div className="grid grid-cols-1 gap-2 sm:grid-cols-[1.5fr_2fr_0.8fr_auto]">
          <input
            value={form.name}
            onChange={(e) => setForm((p) => ({ ...p, name: e.target.value }))}
            placeholder="Name"
            className="rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover"
          />
          <input
            value={form.host}
            onChange={(e) => setForm((p) => ({ ...p, host: e.target.value }))}
            placeholder="host / IP (e.g. 192.168.0.100)"
            className="rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover"
          />
          <input
            value={form.port}
            onChange={(e) => setForm((p) => ({ ...p, port: e.target.value }))}
            placeholder="9090"
            className="rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover"
          />
          <button
            onClick={addRobot}
            className="rounded-lg border border-themeBlue bg-themeBlue/10 px-4 py-2 text-sm font-semibold text-themeBlue transition-colors hover:bg-themeBlue hover:text-white"
          >
            Add
          </button>
        </div>
      </DashboardCard>
    </div>
  );
};

export default FleetPage;
