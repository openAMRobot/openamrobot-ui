import React, {
  useCallback,
  useContext,
  useEffect,
  createContext,
  useState,
  useMemo,
} from "react";

import "./configs/normalize.css";
import "../shared/styles/fonts.css";
import "../shared/styles/global.css";
import "../shared/styles/icons.css";

import { AppConfig } from "../shared/constants";
import {
  loadRuntimeConfig,
  saveRuntimeConfig,
  resolveRosbridgeHost,
} from "../shared/constants/runtimeConfig";

import withProviders from "./providers";
import Routes from "../pages";
import {
  installDemoTransport,
  uninstallDemoTransport,
  startDemoTicking,
} from "../shared/demo/demoData";

export const RosContext = createContext(null);
export const RosStatusContext = createContext("disconnected");
export const ThemeContext = createContext({
  theme: "dark",
  toggleTheme: () => {},
});
export const RuntimeConfigContext = createContext({
  config: loadRuntimeConfig(),
  updateConfig: () => {},
});

// Standard way for a panel/page to reach the shared ROS connection — see
// docs/extending/add-a-ui-panel.md. One ROSLIB.Ros instance is created here
// and shared app-wide; panels never create their own.
export const useRos = () => useContext(RosContext);
export const useRosStatus = () => useContext(RosStatusContext);
// Connection host/port, camera port, and speed-limit overrides editable on
// the Config page (/config) and persisted to localStorage — see
// shared/constants/runtimeConfig.js for the shape and defaults.
export const useRuntimeConfig = () => useContext(RuntimeConfigContext);

const App = () => {
  const [ros] = useState(new window.ROSLIB.Ros());
  const [status, setStatus] = useState("disconnected");

  const [runtimeConfig, setRuntimeConfig] = useState(() => loadRuntimeConfig());

  const updateRuntimeConfig = useCallback((partial) => {
    setRuntimeConfig((prev) => {
      const next = { ...prev, ...partial };
      saveRuntimeConfig(next);
      return next;
    });
  }, []);

  const runtimeConfigValue = useMemo(
    () => ({ config: runtimeConfig, updateConfig: updateRuntimeConfig }),
    [runtimeConfig, updateRuntimeConfig],
  );

  const [theme, setTheme] = useState(
    () => localStorage.getItem("theme") || "dark",
  );

  const toggleTheme = useCallback(() => {
    setTheme((t) => {
      const next = t === "light" ? "dark" : "light";
      localStorage.setItem("theme", next);
      return next;
    });
  }, []);

  useEffect(() => {
    document.documentElement.classList.toggle("dark", theme === "dark");
    document.documentElement.style.colorScheme = theme;
  }, [theme]);

  const themeValue = useMemo(
    () => ({ theme, toggleTheme }),
    [theme, toggleTheme],
  );

  // Depend only on the two connection-relevant fields (not the whole
  // runtimeConfig object) so unrelated settings changes — e.g. a speed
  // limit — don't tear down and reopen the rosbridge connection.
  const { rosbridgeHost, rosbridgePort, demoMode } = runtimeConfig;

  // Installed synchronously during render, not in an effect: React fires
  // child effects before this component's own effects on mount, so a child
  // like LifecycleStatus would otherwise make its first service call
  // before the override below existed. See demoData.js for why this needs
  // to be synchronous and idempotent.
  if (demoMode) installDemoTransport(ros);

  const tryToConnect = useCallback(async () => {
    const host = resolveRosbridgeHost({ rosbridgeHost });
    try {
      ros.connect("ws://" + host + ":" + rosbridgePort);
    } catch (err) {
      console.log("Connecting error", err);
    }
  }, [ros, rosbridgeHost, rosbridgePort]);

  // Demo mode never touches the real connection at all — no ros.connect(),
  // no reconnect loop. It feeds synthetic telemetry through the same
  // shared `ros` event emitter every component already subscribes through
  // (see shared/demo/demoData.js), so switching this off hands control
  // straight back to the normal connect/reconnect logic below with no
  // extra bookkeeping here.
  useEffect(() => {
    if (!demoMode) return undefined;
    const stopTicking = startDemoTicking(ros);
    setStatus("connected");
    return () => {
      stopTicking();
      uninstallDemoTransport(ros);
      setStatus("disconnected");
    };
  }, [ros, demoMode]);

  useEffect(() => {
    if (demoMode) return undefined;
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
      // Detach listeners before closing so this deliberate close (settings
      // changed, or the app is unmounting) doesn't trigger handleClose's
      // own reconnect-with-stale-settings scheduling.
      ros.off("connection", handleConnect);
      ros.off("close", handleClose);
      ros.off("error", handleError);
      if (reconnectTimeout) {
        clearTimeout(reconnectTimeout);
      }
      ros.close();
    };
  }, [ros, tryToConnect, demoMode]);

  return (
    <RuntimeConfigContext.Provider value={runtimeConfigValue}>
      <ThemeContext.Provider value={themeValue}>
        <RosContext.Provider value={ros}>
          <RosStatusContext.Provider value={status}>
            <Routes />
          </RosStatusContext.Provider>
        </RosContext.Provider>
      </ThemeContext.Provider>
    </RuntimeConfigContext.Provider>
  );
};

export default withProviders(App);
