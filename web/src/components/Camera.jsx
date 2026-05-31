import React, { useEffect, useCallback, useState } from "react";
import { AppConfig } from "../shared/constants/index";

const Camera = () => {
  const [videoSrc, setVideoSrc] = useState("");
  const [topic, setTopic] = useState("/rgb_image");
  const [quality, setQuality] = useState("balanced");
  const [status, setStatus] = useState("idle");

  const streamProfiles = {
    low: { quality: 25, width: 320, height: 240 },
    balanced: { quality: 45, width: 480, height: 360 },
    high: { quality: 70, width: 640, height: 480 },
  };

  const tryToConnectToCamera = useCallback(async () => {
    const currentIP = window.location.hostname;
    const currentUrl = window.location.href;
    const ip = !currentUrl.includes("http://localhost:3000/")
      ? currentIP
      : AppConfig.ROSBRIDGE_SERVER_IP;
    const profile = streamProfiles[quality] || streamProfiles.balanced;
    const videoSrcString = `http://${ip}:${AppConfig.CAMERA_PORT}/stream?topic=${topic}&type=mjpeg&quality=${profile.quality}&width=${profile.width}&height=${profile.height}`;
    setStatus("loading");
    setVideoSrc(videoSrcString);
  }, [quality, topic]);

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
    <div className="flex h-full w-full flex-col overflow-hidden rounded-xl border border-borderSubtle bg-bgCard font-[RobotoMono]">
      <div className="flex items-center justify-between gap-2 border-b border-borderSubtle bg-bgSurface px-3 py-1.5">
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
            <option value="/camera/color/image_raw">Color</option>
            <option value="/rgb_image">RGB Sim</option>
            <option value="/camera/image_raw">Raw</option>
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
            onClick={status === "idle" ? tryToConnectToCamera : stopCamera}
            className="rounded border border-borderSubtle px-2 py-0.5 text-xs text-themeBlue hover:border-themeBlue"
          >
            {status === "idle" ? "Start" : "Stop"}
          </button>
          <button
            onClick={openFullscreen}
            className="rounded border border-borderSubtle px-2 py-0.5 text-xs text-themeBlue hover:border-themeBlue"
          >
            Full
          </button>
        </div>
      </div>
      <div className="relative flex min-h-0 flex-1 items-center justify-center">
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
          <button
            onClick={tryToConnectToCamera}
            className="rounded-lg border border-borderSubtle bg-bgCard px-4 py-2 text-xs text-themeBlue"
          >
            Start camera
          </button>
        )}
        {status === "error" && (
          <button
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
