import React, { useEffect, useCallback, useState } from "react";
import { AppConfig } from "../shared/constants/index";

const Camera = () => {
  const [videoSrc, setVideoSrc] = useState("");

  const tryToConnectToCamera = useCallback(async () => {
    const currentIP = window.location.hostname;
    const currentUrl = window.location.href;
    const ip = !currentUrl.includes("http://localhost:3000/")
      ? currentIP
      : AppConfig.ROSBRIDGE_SERVER_IP;
    // const videoSrcString = `http://${ip}:${AppConfig.CAMERA_PORT}/stream?topic=/usb_cam/image_raw`;
    const videoSrcString = `http://${ip}:${AppConfig.CAMERA_PORT}/stream?topic=/camera/color/image_raw`;
    /*
    /camera/color/image_raw - топик для камеры?
    */
    setVideoSrc(videoSrcString);
  }, []);

  useEffect(() => {
    tryToConnectToCamera();
  }, [tryToConnectToCamera]);

  return (
    <div className="border-themeBlue flex h-full w-full items-center justify-center border rounded-[16px]">
      <img src={videoSrc} className="h-full w-full" />
    </div>
  );
};

export default Camera;
