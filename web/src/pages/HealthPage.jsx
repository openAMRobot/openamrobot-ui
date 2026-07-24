import React from "react";
import { Link } from "react-router-dom";

import { useRosStatus } from "../app/App";
import SystemHealth from "../components/SystemHealth";
import LifecycleStatus from "../components/LifecycleStatus";
import useSystemDiagnostics from "../shared/hooks/useSystemDiagnostics";
import {
  DashboardCard,
  EmptyState,
  SectionHeader,
  StatusBadge,
} from "../shared/ui/Dashboard";

const OVERALL_STYLE = {
  0: {
    border: "border-statusGreen/30",
    bg: "bg-statusGreen/10",
    text: "text-statusGreen",
    dot: "bg-statusGreen",
    pulse: true,
  },
  1: {
    border: "border-statusYellow/30",
    bg: "bg-statusYellow/10",
    text: "text-statusYellow",
    dot: "bg-statusYellow",
    pulse: false,
  },
  2: {
    border: "border-statusRed/30",
    bg: "bg-statusRed/5",
    text: "text-statusRed",
    dot: "bg-statusRed",
    pulse: false,
  },
  3: {
    border: "border-statusRed/50",
    bg: "bg-statusRed/15",
    text: "text-statusRed",
    dot: "bg-statusRed",
    pulse: true,
  },
};

const formatTime = (ms) =>
  new Date(ms).toLocaleTimeString(undefined, {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  });

const HealthPage = () => {
  const rosbridgeStatus = useRosStatus();
  const {
    reportHealth,
    reportLifecycle,
    battery,
    urdf,
    diagnosticsMsgs,
    missingTopics,
    devices,
    deviceStatuses,
    issues,
    overall,
    overallLabel,
    faultLog,
  } = useSystemDiagnostics();

  const style = OVERALL_STYLE[overall];

  return (
    <div className="sectionHeight space-y-5 py-4 sm:space-y-6 sm:py-6">
      <SectionHeader
        eyebrow="System overview"
        title="Health Centre"
        description="One place to answer: is the whole robot ready? Aggregated from the ROS bridge, live topics, TF, Nav2 lifecycle, registered devices, battery, and the robot description."
      />

      <DashboardCard className={`border p-5 ${style.border} ${style.bg}`}>
        <div className="flex flex-wrap items-center gap-3">
          <div className="relative flex h-3 w-3 shrink-0">
            {style.pulse && (
              <span
                className={`absolute inline-flex h-full w-full animate-ping rounded-full ${style.dot} opacity-50`}
              />
            )}
            <span
              className={`relative inline-flex h-3 w-3 rounded-full ${style.dot}`}
            />
          </div>
          <p className={`font-[RobotoMono] text-lg font-bold ${style.text}`}>
            {overallLabel}
          </p>
        </div>

        {issues.length === 0 ? (
          <p className="mt-2 text-sm text-themeTextGray">
            Every checked signal is nominal.
          </p>
        ) : (
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
      </DashboardCard>

      <div className="grid gap-4 lg:grid-cols-2 xl:grid-cols-3">
        <SystemHealth onHealthChange={reportHealth} />
        <LifecycleStatus onStatesChange={reportLifecycle} />

        <DashboardCard className="p-4">
          <p className="mb-2 font-[RobotoMono] text-xs uppercase tracking-wider text-themeTextGray">
            Devices
          </p>
          {devices.length === 0 ? (
            <EmptyState
              className="px-0 pb-0"
              title="No devices registered"
              description="Register hardware on the Devices page to see it here."
            />
          ) : (
            <div className="space-y-1.5">
              {devices.map((device) => {
                const state = deviceStatuses[device.id] || "unmonitored";
                return (
                  <div
                    key={device.id}
                    className="flex items-center justify-between gap-2 rounded-lg bg-bgSurface px-2.5 py-1.5 text-xs"
                  >
                    <span className="truncate text-textWhiteHover">
                      {device.name}
                    </span>
                    <StatusBadge
                      status={
                        state === "online"
                          ? "connected"
                          : state === "offline"
                          ? "disconnected"
                          : "unknown"
                      }
                      label={
                        state === "online"
                          ? "Online"
                          : state === "offline"
                          ? "Offline"
                          : "No status topic"
                      }
                      pulse={state === "online"}
                    />
                  </div>
                );
              })}
            </div>
          )}
          <Link
            to="/devices"
            className="mt-3 inline-block text-xs text-themeBlue hover:underline"
          >
            Manage devices →
          </Link>
        </DashboardCard>

        <DashboardCard className="p-4">
          <p className="mb-2 font-[RobotoMono] text-xs uppercase tracking-wider text-themeTextGray">
            Battery
          </p>
          {battery.pct === null ? (
            <p className="text-xs text-themeTextGray opacity-70">
              No battery telemetry.
            </p>
          ) : (
            <div className="flex items-center justify-between">
              <p className="font-[RobotoMono] text-2xl font-bold text-textWhiteHover">
                {battery.pct}%
              </p>
              <StatusBadge
                status={battery.charging ? "connected" : "idle"}
                label={battery.charging ? "Charging" : "On battery"}
                pulse={battery.charging}
              />
            </div>
          )}
          <Link
            to="/info"
            className="mt-3 inline-block text-xs text-themeBlue hover:underline"
          >
            Full telemetry →
          </Link>
        </DashboardCard>

        <DashboardCard className="p-4">
          <p className="mb-2 font-[RobotoMono] text-xs uppercase tracking-wider text-themeTextGray">
            Robot description
          </p>
          <div>
            <StatusBadge
              status={
                !urdf.checked
                  ? "unknown"
                  : urdf.available
                  ? "connected"
                  : "disconnected"
              }
              label={
                !urdf.checked
                  ? "Checking…"
                  : urdf.available
                  ? "URDF available"
                  : "URDF unavailable"
              }
            />
          </div>
          <Link
            to="/robot"
            className="mt-3 inline-block text-xs text-themeBlue hover:underline"
          >
            Open Robot page →
          </Link>
        </DashboardCard>

        <DashboardCard className="p-4">
          <p className="mb-2 font-[RobotoMono] text-xs uppercase tracking-wider text-themeTextGray">
            Diagnostics (/diagnostics)
          </p>
          {diagnosticsMsgs.length === 0 ? (
            <p className="text-xs text-themeTextGray opacity-70">
              No warning/error-level diagnostics reported.
            </p>
          ) : (
            <div className="space-y-1.5">
              {diagnosticsMsgs.map((entry, index) => (
                <div
                  key={index}
                  className={`rounded-lg px-2.5 py-1.5 text-xs ${
                    entry.level >= 2
                      ? "bg-statusRed/10 text-statusRed"
                      : "bg-statusYellow/10 text-statusYellow"
                  }`}
                >
                  <span className="font-semibold">{entry.name}</span>:{" "}
                  {entry.message}
                </div>
              ))}
            </div>
          )}
        </DashboardCard>

        <DashboardCard className="p-4">
          <p className="mb-2 font-[RobotoMono] text-xs uppercase tracking-wider text-themeTextGray">
            Expected topics
          </p>
          {missingTopics.length === 0 ? (
            <p className="text-xs text-themeTextGray opacity-70">
              {rosbridgeStatus === "connected"
                ? "All expected topics are present in the ROS graph."
                : "Checked once rosbridge is connected."}
            </p>
          ) : (
            <div className="space-y-1.5">
              {missingTopics.map(({ topic, label }) => (
                <div
                  key={topic}
                  className="rounded-lg bg-statusYellow/10 px-2.5 py-1.5 text-xs text-statusYellow"
                >
                  {label} <span className="text-themeTextGray">({topic})</span>
                </div>
              ))}
            </div>
          )}
        </DashboardCard>

        <DashboardCard className="p-4 lg:col-span-2 xl:col-span-3">
          <p className="mb-2 font-[RobotoMono] text-xs uppercase tracking-wider text-themeTextGray">
            Recent faults (this session)
          </p>
          {faultLog.length === 0 ? (
            <p className="text-xs text-themeTextGray opacity-70">
              Nothing new has gone wrong since this page loaded.
            </p>
          ) : (
            <div className="space-y-1.5">
              {faultLog.map((fault, index) => (
                <div key={index} className="flex items-center gap-2 text-xs">
                  <span className="shrink-0 font-[RobotoMono] text-themeTextGray">
                    {formatTime(fault.time)}
                  </span>
                  <span
                    className={
                      fault.severity >= 2 ? "text-statusRed" : "text-statusYellow"
                    }
                  >
                    {fault.message}
                  </span>
                </div>
              ))}
            </div>
          )}
        </DashboardCard>
      </div>
    </div>
  );
};

export default HealthPage;
