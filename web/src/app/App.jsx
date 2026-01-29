import React, { useCallback, useEffect, createContext, useState } from "react";

import "./configs/normalize.css";
import "../shared/styles/fonts.css";
import "../shared/styles/global.css";
import "../shared/styles/icons.css";

import { AppConfig } from "../shared/constants";

import withProviders from "./providers";
import Routes from "../pages";

export const RosContext = createContext(null);

const App = () => {
  const [ros] = useState(new window.ROSLIB.Ros());

  const tryToConnect = useCallback(async () => {
    const currentIP = window.location.hostname;
    const currentUrl = window.location.href;
    const ip = !currentUrl.includes("http://localhost:3000/")
      ? currentIP
      : AppConfig.ROSBRIDGE_SERVER_IP;
    try {
      ros.connect("ws://" + ip + ":" + AppConfig.ROSBRIDGE_SERVER_PORT);
    } catch (err) {
      console.log("Connecting error", err);
    }
  }, [ros]);

  const checkConnectionStatus = useCallback(() => {
    ros.on("connection", () => {
      console.log("Connection Successful");
    });

    ros.on("close", () => {
      console.log("Connection failed, reloading...");
      tryToConnect();
    });
  }, [ros, tryToConnect]);

  useEffect(() => {
    tryToConnect();
    checkConnectionStatus();
  }, [checkConnectionStatus, tryToConnect]);

  return (
    <RosContext.Provider value={ros}>
      <Routes />
    </RosContext.Provider>
  );
};

export default withProviders(App);
