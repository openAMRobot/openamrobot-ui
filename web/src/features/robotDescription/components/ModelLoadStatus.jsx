import React from "react";

import { DashboardCard, StatusBadge } from "../../../shared/ui/Dashboard";

const STAGE_LABEL = {
  idle: "Idle",
  "loading-urdf": "Fetching URDF…",
  "loading-meshes": "Loading meshes…",
  ready: "Ready",
  error: "Failed",
};

const STAGE_STATUS = {
  idle: "idle",
  "loading-urdf": "info",
  "loading-meshes": "info",
  ready: "success",
  error: "error",
};

const ModelLoadStatus = ({ manifest, status }) => {
  if (!manifest) {
    return null;
  }

  if (!manifest.available) {
    return (
      <DashboardCard className="p-4">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Model load status
        </p>
        <p className="mt-2 text-sm text-statusRed">
          This robot's 3D model isn't available right now. Contact your
          integrator or IT support.
        </p>
        <p className="mt-1 text-[11px] text-themeTextGray/60">
          No <code>robot_description/openamrobot</code> share directory
          found. Rebuild <code>openamr_ui_package</code> with the vendored
          URDF in place.
        </p>
      </DashboardCard>
    );
  }

  return (
    <DashboardCard className="p-4">
      <div className="flex items-center justify-between gap-3">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Model load status
        </p>
        <StatusBadge
          status={STAGE_STATUS[status.stage] || "idle"}
          label={STAGE_LABEL[status.stage] || status.stage}
          pulse={status.stage === "loading-meshes"}
        />
      </div>

      {status.stage === "loading-meshes" && status.total > 0 ? (
        <div className="mt-3 space-y-1">
          <div className="premium-progress" role="progressbar" aria-valuemin="0" aria-valuemax={status.total} aria-valuenow={status.loaded}>
            <div
              className="premium-progress__value"
              style={{ width: `${Math.round((status.loaded / status.total) * 100)}%` }}
            />
          </div>
          <p className="text-[11px] text-themeTextGray">
            {status.loaded}/{status.total} mesh requests complete
          </p>
        </div>
      ) : null}

      {status.stage === "error" ? (
        <p className="mt-2 text-sm text-statusRed">{status.error}</p>
      ) : null}

      {status.meshErrors.length > 0 ? (
        <div className="mt-3 space-y-1">
          <p className="text-[11px] font-semibold text-statusYellow">
            {status.meshErrors.length} mesh{status.meshErrors.length === 1 ? "" : "es"} failed
            to load — shown as a red wireframe placeholder in the viewer,
            not substituted with unrelated geometry:
          </p>
          <ul className="max-h-24 space-y-0.5 overflow-y-auto text-[11px] text-themeTextGray">
            {status.meshErrors.map((e) => (
              <li key={e.path} className="truncate" title={e.message}>
                {e.path}
              </li>
            ))}
          </ul>
        </div>
      ) : null}
    </DashboardCard>
  );
};

export default ModelLoadStatus;
