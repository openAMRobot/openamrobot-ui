import React from "react";

import Switcher from "../../../shared/ui/Switcher";
import { DashboardCard, StatusBadge } from "../../../shared/ui/Dashboard";

const DescriptionModeBanner = ({
  mode,
  onModeChange,
  rosStatus,
  liveConnected,
  poseSource,
  pathPointCount,
}) => {
  const isLive = mode === "live";

  return (
    <DashboardCard
      className={`flex flex-col gap-3 p-4 sm:flex-row sm:items-center sm:justify-between ${
        isLive ? "ring-1 ring-statusYellow/40" : ""
      }`}
    >
      <div className="flex items-center gap-3">
        <Switcher switcherValue={isLive} onChange={(next) => onModeChange(next ? "live" : "description")} />
        <div>
          <p className="text-sm font-semibold text-textWhiteHover">
            {isLive ? "Live Mode" : "Description Mode"}
          </p>
          <p className="mt-0.5 max-w-2xl text-[12px] leading-snug text-themeTextGray">
            {isLive ? (
              <>
                Joint state comes from ROS 2 (<code>/joint_states</code>) when
                connected. This robot has no per-joint position command — only{" "}
                <code>/cmd_vel</code>, which drives both wheels together — so
                sliders here are <strong>read-only telemetry</strong>, never a
                command to the physical robot. The model is also positioned at
                its real map-frame pose and Nav2&apos;s current planned path is
                drawn as a line — the ground grid stays a fixed reference, it
                is not the actual occupancy map.
              </>
            ) : (
              <>
                The robot is loaded locally from its URDF/Xacro description.
                Joint sliders only change this visualization — no ROS
                connection is required and nothing is published anywhere.
              </>
            )}
          </p>
        </div>
      </div>

      {isLive ? (
        <div className="flex flex-none items-center gap-2">
          <StatusBadge
            status={rosStatus === "connected" ? "success" : "danger"}
            label={rosStatus === "connected" ? "ROS connected" : "ROS disconnected"}
          />
          <StatusBadge
            status={liveConnected ? "success" : "idle"}
            label={liveConnected ? "Joint states live" : "No joint states yet"}
          />
          <StatusBadge
            status={poseSource ? "success" : "idle"}
            label={poseSource ? `Pose: ${poseSource.toUpperCase()}` : "No pose yet"}
          />
          <StatusBadge
            status={pathPointCount ? "success" : "idle"}
            label={pathPointCount ? `Path: ${pathPointCount} pts` : "No path"}
          />
        </div>
      ) : null}
    </DashboardCard>
  );
};

export default DescriptionModeBanner;
