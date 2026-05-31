import React, { useCallback, useEffect, createContext, useState } from "react";

import "./configs/normalize.css";
import "../shared/styles/fonts.css";
import "../shared/styles/global.css";
import "../shared/styles/icons.css";

import { AppConfig } from "../shared/constants";

import withProviders from "./providers";
import Routes from "../pages";

export const RosContext = createContext(null);
export const RosStatusContext = createContext("disconnected");

const App = () => {
  const [ros] = useState(new window.ROSLIB.Ros());
  const [status, setStatus] = useState("disconnected");

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

  useEffect(() => {
    let reconnectTimeout = null;

    const handleConnect = () => {
      setStatus("connected");
    };

    const handleClose = () => {
      setStatus("disconnected");
      if (reconnectTimeout) clearTimeout(reconnectTimeout);
      reconnectTimeout = setTimeout(tryToConnect, AppConfig.RECONNECTION_TIME);
    };

    const handleError = () => {
      setStatus("error");
      if (reconnectTimeout) clearTimeout(reconnectTimeout);
      reconnectTimeout = setTimeout(tryToConnect, AppConfig.RECONNECTION_TIME);
    };

    ros.on("connection", handleConnect);
    ros.on("close", handleClose);
    ros.on("error", handleError);

    tryToConnect();

    return () => {
      ros.off("connection", handleConnect);
      ros.off("close", handleClose);
      ros.off("error", handleError);
      if (reconnectTimeout) {
        clearTimeout(reconnectTimeout);
      }
    };
  }, [ros, tryToConnect]);

  return (
    <RosContext.Provider value={ros}>
      <RosStatusContext.Provider value={status}>
        <Routes />
      </RosStatusContext.Provider>
    </RosContext.Provider>
  );
};

export default withProviders(App);
