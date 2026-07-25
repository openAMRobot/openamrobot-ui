import React, { useEffect, useState } from "react";

import { useRosStatus } from "../app/App";
import useRobotMetrics from "../shared/hooks/useRobotMetrics";
import {
  DashboardCard,
  MetricCard,
  SectionHeader,
  StatusBadge,
} from "../shared/ui/Dashboard";

const fmtDistance = (m) => (m >= 1000 ? `${(m / 1000).toFixed(2)}` : `${m.toFixed(1)}`);
const distUnit = (m) => (m >= 1000 ? "km" : "m");

const fmtDuration = (ms) => {
  const s = Math.max(0, Math.floor(ms / 1000));
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const sec = s % 60;
  if (h > 0) return `${h}h ${m}m`;
  if (m > 0) return `${m}m ${sec}s`;
  return `${sec}s`;
};

/**
 * The robot's "track record" — distance, uptime, goal + docking outcomes and
 * speed, all derived client-side from telemetry the stack already publishes
 * (see useRobotMetrics). Cumulative counters persist across reloads; Reset
 * zeroes them.
 */
const MetricsPage = () => {
  const rosStatus = useRosStatus();
  const connected = rosStatus === "connected";
  const { metrics, speed, reset } = useRobotMetrics();
  const [now, setNow] = useState(() => Date.now());

  const handleReset = () => {
    if (!window.confirm("Reset all totals below back to zero? This can't be undone.")) return;
    reset();
  };

  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);

  const totalGoals =
    metrics.goalsSucceeded + metrics.goalsFailed + metrics.goalsCanceled;
  const successRate = totalGoals
    ? Math.round((metrics.goalsSucceeded / totalGoals) * 100)
    : null;
  const totalDock = metrics.dockSuccess + metrics.dockFail;
  const dockRate = totalDock
    ? Math.round((metrics.dockSuccess / totalDock) * 100)
    : null;

  return (
    <div className="sectionHeight space-y-5 py-4 sm:py-6">
      <SectionHeader
        eyebrow="Track record"
        title="Metrics"
        description="Derived from live telemetry and kept locally. Counters accumulate across sessions until reset."
        action={
          <div className="flex items-center gap-3">
            <StatusBadge
              status={connected ? "connected" : "disconnected"}
              pulse={connected}
              label={connected ? "Online" : "Offline"}
            />
            <button
              onClick={handleReset}
              className="rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeTextGray transition-colors hover:border-statusRed/50 hover:text-statusRed"
            >
              Reset counters
            </button>
          </div>
        }
      />

      <div className="grid grid-cols-2 gap-3 lg:grid-cols-4">
        <MetricCard
          label="Distance travelled"
          value={fmtDistance(metrics.distance)}
          unit={distUnit(metrics.distance)}
        />
        <MetricCard
          label="Current speed"
          value={speed.toFixed(2)}
          unit="m/s"
          meta={`peak ${metrics.maxSpeed.toFixed(2)} m/s`}
        />
        <MetricCard
          label="Session uptime"
          value={fmtDuration(now - metrics.since)}
        />
        <MetricCard
          label="Trips completed"
          value={totalGoals}
          meta={successRate === null ? "no goals yet" : `${successRate}% success`}
        />
      </div>

      <div className="grid grid-cols-1 gap-3 lg:grid-cols-2">
        <DashboardCard className="p-4 font-[RobotoMono]">
          <p className="mb-3 text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
            Trips
          </p>
          <div className="grid grid-cols-3 gap-3 text-center">
            <div>
              <p className="text-2xl font-semibold text-statusGreen">{metrics.goalsSucceeded}</p>
              <p className="text-xs text-themeTextGray">Completed</p>
            </div>
            <div>
              <p className="text-2xl font-semibold text-statusRed">{metrics.goalsFailed}</p>
              <p className="text-xs text-themeTextGray">Failed</p>
            </div>
            <div>
              <p className="text-2xl font-semibold text-themeTextGray">{metrics.goalsCanceled}</p>
              <p className="text-xs text-themeTextGray">Stopped early</p>
            </div>
          </div>
        </DashboardCard>

        <DashboardCard className="p-4 font-[RobotoMono]">
          <p className="mb-3 text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
            Docking
          </p>
          <div className="grid grid-cols-3 gap-3 text-center">
            <div>
              <p className="text-2xl font-semibold text-statusGreen">{metrics.dockSuccess}</p>
              <p className="text-xs text-themeTextGray">Success</p>
            </div>
            <div>
              <p className="text-2xl font-semibold text-statusRed">{metrics.dockFail}</p>
              <p className="text-xs text-themeTextGray">Failed</p>
            </div>
            <div>
              <p className="text-2xl font-semibold text-themeBlue">
                {dockRate === null ? "—" : `${dockRate}%`}
              </p>
              <p className="text-xs text-themeTextGray">Rate</p>
            </div>
          </div>
        </DashboardCard>
      </div>
    </div>
  );
};

export default MetricsPage;
