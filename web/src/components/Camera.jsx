import React, { useEffect, useCallback, useState } from "react";
import {
  CAMERA_TOPIC_OPTIONS,
  DEFAULT_CAMERA_TOPIC,
} from "../shared/constants/index";
import { resolveRosbridgeHost } from "../shared/constants/runtimeConfig";
import { useRuntimeConfig } from "../app/App";
import { EmptyState, LoadingSkeleton } from "../shared/ui/Dashboard";

const Camera = () => {
  const { config } = useRuntimeConfig();
  const [videoSrc, setVideoSrc] = useState("");
  const [topic, setTopic] = useState(DEFAULT_CAMERA_TOPIC);
  const [quality, setQuality] = useState("balanced");
  const [status, setStatus] = useState("idle");

  const streamProfiles = {
    low: { quality: 25, width: 320, height: 240 },
    balanced: { quality: 45, width: 480, height: 360 },
    high: { quality: 70, width: 640, height: 480 },
  };

  const tryToConnectToCamera = useCallback(async () => {
    const ip = resolveRosbridgeHost(config);
    const profile = streamProfiles[quality] || streamProfiles.balanced;
    const videoSrcString = `http://${ip}:${config.cameraPort}/stream?topic=${topic}&type=mjpeg&quality=${profile.quality}&width=${profile.width}&height=${profile.height}`;
    setStatus("loading");
    setVideoSrc(videoSrcString);
    // streamProfiles is intentionally omitted — it's a fresh object literal
    // every render, but its values are static, so it doesn't need to be a dep.
  }, [quality, topic, config]);

  useEffect(() => {
    setVideoSrc("");
    setStatus("idle");
  }, [quality, topic]);

  const stopCamera = () => {
    setVideoSrc("");
    setStatus("idle");
  };

  const openFullscreen = () => {
    if (videoSrc) window.open(videoSrc, "_blank", "noopener,noreferrer");
  };

  return (
    <div className="dashboard-card dashboard-card--recessed flex h-full w-full flex-col overflow-hidden font-[RobotoMono]">
      <div className="flex flex-wrap items-center justify-between gap-2 border-b border-borderSubtle bg-bgSurface/70 px-3 py-2">
        <div className="flex items-center gap-2">
          <span
            className={`h-2 w-2 rounded-full ${
              status === "online"
                ? "bg-statusGreen"
                : status === "error"
                ? "bg-statusRed"
                : "bg-statusYellow"
            }`}
          />
          <span className="text-xs uppercase tracking-wider text-themeTextGray">
            Camera
          </span>
        </div>
        <div className="flex flex-wrap items-center justify-end gap-1.5">
          <select
            value={topic}
            onChange={(e) => setTopic(e.target.value)}
            className="rounded border border-borderSubtle bg-bgCard px-2 py-0.5 text-xs text-textWhiteHover"
          >
            {CAMERA_TOPIC_OPTIONS.map(({ value, label }) => (
              <option key={value} value={value}>
                {label}
              </option>
            ))}
          </select>
          <select
            value={quality}
            onChange={(e) => setQuality(e.target.value)}
            className="rounded border border-borderSubtle bg-bgCard px-2 py-0.5 text-xs text-textWhiteHover"
          >
            <option value="low">Low</option>
            <option value="balanced">Balanced</option>
            <option value="high">High</option>
          </select>
          <button
            type="button"
            onClick={status === "idle" ? tryToConnectToCamera : stopCamera}
            className="min-h-[32px] rounded-lg border border-borderSubtle px-2.5 text-xs text-themeBlue hover:border-themeBlue"
          >
            {status === "idle" ? "Start" : "Stop"}
          </button>
          <button
            type="button"
            onClick={openFullscreen}
            disabled={!videoSrc}
            className="min-h-[32px] rounded-lg border border-borderSubtle px-2.5 text-xs text-themeBlue hover:border-themeBlue disabled:opacity-40"
          >
            Full
          </button>
        </div>
      </div>
      <div className="relative flex min-h-0 flex-1 items-center justify-center bg-bgSurface/30">
        {videoSrc ? (
          <img
            key={videoSrc}
            src={videoSrc}
            className="h-full w-full object-cover"
            alt="Camera feed"
            onLoad={() => setStatus("online")}
            onError={(e) => {
              setStatus("error");
              e.currentTarget.style.display = "none";
            }}
          />
        ) : (
          <EmptyState
            title="Camera is paused"
            description="Start the configured MJPEG stream when visual monitoring is needed."
            icon={
              <svg
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="1.7"
                className="h-5 w-5"
                aria-hidden="true"
              >
                <rect x="3" y="6" width="14" height="12" rx="3" />
                <path d="m17 10 4-2v8l-4-2" strokeLinejoin="round" />
              </svg>
            }
            action={
              <button
                type="button"
                onClick={tryToConnectToCamera}
                className="rounded-lg border border-themeBlue/40 bg-themeBlue/10 px-4 py-2 text-xs font-semibold text-themeBlue hover:bg-themeBlue hover:text-white"
              >
                Start camera
              </button>
            }
          />
        )}
        {status === "loading" ? (
          <div className="absolute inset-x-8 bottom-8 rounded-xl border border-borderSubtle bg-bgCard/90 p-4 backdrop-blur">
            <LoadingSkeleton lines={2} />
          </div>
        ) : null}
        {status === "error" && (
          <button
            type="button"
            onClick={tryToConnectToCamera}
            className="absolute rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-xs text-statusRed"
          >
            Camera unavailable
          </button>
        )}
      </div>
    </div>
  );
};

export default Camera;
