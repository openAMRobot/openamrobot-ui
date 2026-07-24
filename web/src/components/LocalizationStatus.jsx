import React, { useEffect, useRef, useState } from "react";
import { useRos } from "../app/App";
import { AppConfig } from "../shared/constants";

// AMCL publishes a 6x6 pose covariance (row-major, 36 floats). The three
// diagonal entries we care about for a planar robot are x, y and yaw:
const COV_X = 0; // [0][0]
const COV_Y = 7; // [1][1]
const COV_YAW = 35; // [5][5]

// How long without a fresh /amcl_pose before we treat localization as "no
// data" rather than trusting the last (possibly stale) confidence reading.
const STALE_AFTER_MS = 6000;

// Confidence bands from the position standard deviation (metres) and yaw
// standard deviation (degrees). Tuned for a well-behaved indoor AMCL: a
// converged filter sits well under 0.25 m / 8°; a freshly kidnapped or
// global-init filter has particles spread across metres.
const classify = (posStd, yawStdDeg) => {
  if (posStd <= 0.25 && yawStdDeg <= 8) {
    return {
      key: "good",
      label: "Localized",
      color: "text-statusGreen",
      dot: "bg-statusGreen",
      explain:
        "AMCL's particle cloud has converged — the robot knows where it is with high confidence.",
    };
  }
  if (posStd <= 0.6 && yawStdDeg <= 20) {
    return {
      key: "fair",
      label: "Uncertain",
      color: "text-statusYellow",
      dot: "bg-statusYellow",
      explain:
        "Localization is usable but the estimate is loose. Drive slowly past distinctive features, or set the pose manually to tighten it.",
    };
  }
  return {
    key: "lost",
    label: "Lost",
    color: "text-statusRed",
    dot: "bg-statusRed",
    explain:
      "The particle cloud is spread wide — the robot does not reliably know where it is. Re-localize globally, then drive to help it converge, or set the initial pose on the map.",
  };
};

/**
 * Localization confidence readout for AMCL. Subscribes to the relayed
 * /ui/amcl_pose (PoseWithCovarianceStamped), derives an "am I lost?" band
 * from the pose covariance, and offers two recovery actions:
 *   - Re-localize: calls /reinitialize_global_localization so AMCL scatters
 *     particles across the whole map and re-converges as the robot drives.
 *   - Set pose: hands control back to the Map's existing Set-Pose mode
 *     (via onSetPoseMode) so the operator can click the true pose.
 */
const LocalizationStatus = ({ onSetPoseMode }) => {
  const ros = useRos();
  const [cov, setCov] = useState(null); // { posStd, yawStdDeg }
  const [lastUpdate, setLastUpdate] = useState(null);
  const [now, setNow] = useState(() => Date.now());
  const [showExplain, setShowExplain] = useState(false);
  const [relocalizing, setRelocalizing] = useState(false);
  const globalLocSrvRef = useRef(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return undefined;

    const amclTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.AMCL_POSE_TOPIC,
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });

    const handler = (msg) => {
      const c = msg?.pose?.covariance;
      if (!c || c.length < 36) return;
      const posStd = Math.sqrt(Math.max(c[COV_X], c[COV_Y], 0));
      const yawStdDeg = (Math.sqrt(Math.max(c[COV_YAW], 0)) * 180) / Math.PI;
      setCov({ posStd, yawStdDeg });
      setLastUpdate(Date.now());
    };

    amclTopic.subscribe(handler);

    globalLocSrvRef.current = new window.ROSLIB.Service({
      ros,
      name: "/reinitialize_global_localization",
      serviceType: "std_srvs/Empty",
    });

    return () => amclTopic.unsubscribe(handler);
  }, [ros]);

  // Cheap 1 Hz tick so the "stale / no data" state and the age readout stay
  // live even when AMCL stops publishing (the very case we want to surface).
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);

  const ageMs = lastUpdate == null ? Infinity : now - lastUpdate;
  const stale = ageMs > STALE_AFTER_MS;
  const hasData = cov != null && !stale;
  const band = hasData ? classify(cov.posStd, cov.yawStdDeg) : null;

  const reLocalize = () => {
    const srv = globalLocSrvRef.current;
    if (!srv) return;
    setRelocalizing(true);
    srv.callService(
      new window.ROSLIB.ServiceRequest({}),
      () => setTimeout(() => setRelocalizing(false), 1500),
      () => setRelocalizing(false),
    );
  };

  const dot = band ? band.dot : "bg-themeTextGray";
  const color = band ? band.color : "text-themeTextGray";
  const label = band ? band.label : lastUpdate == null ? "No data" : "Stale";
  const isLost = band?.key === "lost";

  return (
    <div className="dashboard-card px-4 py-1.5 font-[RobotoMono]" data-tour="localization">
      <div className="flex flex-wrap items-center justify-between gap-2">
        <div className="flex items-center gap-3">
          <div className="relative flex h-2.5 w-2.5">
            {isLost && (
              <span
                className={`absolute inline-flex h-full w-full animate-ping rounded-full ${dot} opacity-60`}
              />
            )}
            <span className={`relative inline-flex h-2.5 w-2.5 rounded-full ${dot}`} />
          </div>
          <div>
            <span className="text-xs uppercase tracking-wider text-themeTextGray">
              Localization{" "}
            </span>
            <span className={`text-sm font-semibold ${color}`}>{label}</span>
            <button
              onClick={() => setShowExplain((v) => !v)}
              aria-label="What does this mean?"
              title="What does this mean?"
              className="ml-1.5 inline-flex h-4 w-4 items-center justify-center rounded-full border border-borderSubtle text-[10px] text-themeTextGray hover:border-themeBlue hover:text-themeBlue"
            >
              ?
            </button>
            {hasData && (
              <span className="ml-3 text-xs text-themeTextGray">
                ±{cov.posStd.toFixed(2)}m · ±{cov.yawStdDeg.toFixed(0)}°
              </span>
            )}
          </div>
        </div>

        <div className="flex items-center gap-2">
          {onSetPoseMode && (
            <button
              onClick={onSetPoseMode}
              className="rounded-lg border border-borderSubtle px-3 py-1 text-xs text-themeTextGray transition-colors hover:border-themeBlue hover:text-themeBlue"
            >
              Set pose
            </button>
          )}
          <button
            onClick={reLocalize}
            disabled={relocalizing}
            title="Scatter AMCL particles across the map and re-converge (drive to help)"
            className={`rounded-lg border px-3 py-1 text-xs transition-colors ${
              isLost
                ? "border-statusRed text-statusRed hover:bg-statusRed hover:text-white"
                : "border-borderSubtle text-themeTextGray hover:border-themeBlue hover:text-themeBlue"
            } disabled:opacity-50`}
          >
            {relocalizing ? "Re-localizing…" : "Re-localize"}
          </button>
        </div>
      </div>

      {showExplain && (
        <p className="mt-1 border-t border-borderSubtle pt-1 text-[11px] leading-snug text-themeTextGray">
          {band
            ? band.explain
            : lastUpdate == null
              ? "No /amcl_pose received yet. AMCL may not be running, or the robot hasn't been given an initial pose."
              : "AMCL has stopped publishing a pose estimate — the last reading is stale. Check that the amcl node is alive on the Health page."}
        </p>
      )}
    </div>
  );
};

export default LocalizationStatus;
