import React from "react";

import { DashboardCard, IconButton, LoadingSkeleton } from "../../../shared/ui/Dashboard";

const RobotModelViewer = ({ containerRef, status, resetView, fitToScreen, toggleFullscreen, isFullscreen }) => {
  const isLoading = status.stage === "loading-urdf" || status.stage === "loading-meshes";

  return (
    <DashboardCard className="dashboard-card--recessed relative flex h-full min-h-[420px] flex-col overflow-hidden p-0">
      <div className="absolute right-3 top-3 z-10 flex gap-2">
        <IconButton label="Reset view" onClick={resetView} className="bg-bgCard/85 backdrop-blur">
          ⟲
        </IconButton>
        <IconButton label="Fit to screen" onClick={fitToScreen} className="bg-bgCard/85 backdrop-blur">
          ⛶
        </IconButton>
        <IconButton
          label={isFullscreen ? "Exit fullscreen" : "Fullscreen"}
          onClick={toggleFullscreen}
          className="bg-bgCard/85 backdrop-blur"
        >
          {isFullscreen ? "✕" : "⤢"}
        </IconButton>
      </div>

      <div ref={containerRef} className="h-full min-h-[420px] w-full flex-1" />

      {isLoading ? (
        <div className="pointer-events-none absolute inset-0 flex items-center justify-center bg-bgBase/50">
          <LoadingSkeleton className="w-56" lines={3} />
        </div>
      ) : null}

      {status.stage === "error" ? (
        <div className="absolute inset-0 flex items-center justify-center bg-bgBase/85 p-6 text-center">
          <p className="max-w-md text-sm text-statusRed">{status.error}</p>
        </div>
      ) : null}
    </DashboardCard>
  );
};

export default RobotModelViewer;
