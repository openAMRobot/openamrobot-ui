import React from "react";

import { useRosStatus } from "../app/App";
import RosoutConsole from "../components/RosoutConsole";
import TopicEcho from "../components/TopicEcho";
import { SectionHeader, StatusBadge } from "../shared/ui/Dashboard";

/**
 * Observability page: a live /rosout log console plus an "echo any topic"
 * panel, so operators can debug the running stack from the browser without a
 * sourced terminal. Both panels share the app-wide ROS connection.
 */
const ConsolePage = () => {
  const rosStatus = useRosStatus();
  const connected = rosStatus === "connected";

  return (
    <div className="flex min-h-[calc(100vh-145px)] flex-col gap-3 py-3">
      <SectionHeader
        title="Console"
        description="Live ROS logs and raw topic inspection"
        action={
          <StatusBadge
            status={connected ? "connected" : "disconnected"}
            pulse={connected}
            label={connected ? "Robot connected" : "Robot offline"}
          />
        }
      />

      {!connected && (
        <p className="dashboard-card px-4 py-2 font-[RobotoMono] text-xs text-themeTextGray">
          Not connected to the robot — logs and topic echo will start streaming
          once the connection is live. Check the host/port on the Config page.
        </p>
      )}

      <div className="grid min-h-0 flex-1 grid-cols-1 gap-3 xl:grid-cols-2">
        <div className="h-[440px] min-h-0 xl:h-auto">
          <RosoutConsole />
        </div>
        <div className="h-[440px] min-h-0 xl:h-auto">
          <TopicEcho />
        </div>
      </div>
    </div>
  );
};

export default ConsolePage;
