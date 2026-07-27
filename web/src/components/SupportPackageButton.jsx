import React, { useState } from "react";
import { toast } from "react-toastify";

import { useRos, useRosStatus, useRuntimeConfig } from "../app/App";
import { resolveRosbridgeHost } from "../shared/constants/runtimeConfig";
import buildSupportPackage from "../shared/support/buildSupportPackage";

/**
 * Downloads a single JSON file bundling connection info, the Health Centre
 * rollup, recent events, track-record metrics, runtime config, and a
 * best-effort Nav2 param snapshot — meant to be attached to a support
 * ticket instead of asking an operator to screenshot five different pages.
 * `health` ({overall, overallLabel, issues}) is passed in from a parent that
 * already calls useSystemDiagnostics, so this button never subscribes to ROS
 * topics on its own.
 */
const SupportPackageButton = ({ health }) => {
  const ros = useRos();
  const rosStatus = useRosStatus();
  const { config } = useRuntimeConfig();
  const [busy, setBusy] = useState(false);

  const download = async () => {
    setBusy(true);
    try {
      const pkg = await buildSupportPackage({
        ros,
        rosStatus,
        resolvedHost: resolveRosbridgeHost(config),
        config,
        health,
      });
      const blob = new Blob([JSON.stringify(pkg, null, 2)], {
        type: "application/json",
      });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = `openamr-support-${Date.now()}.json`;
      a.click();
      URL.revokeObjectURL(url);
      toast.success("Support package downloaded");
    } catch (err) {
      toast.error(`Couldn't build support package: ${err?.message || err}`);
    } finally {
      setBusy(false);
    }
  };

  return (
    <button
      onClick={download}
      disabled={busy}
      className="rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeTextGray transition-colors hover:border-themeBlue hover:text-themeBlue disabled:opacity-40"
    >
      {busy ? "Building…" : "Export support package"}
    </button>
  );
};

export default SupportPackageButton;
