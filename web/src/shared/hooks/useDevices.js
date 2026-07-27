import { useEffect, useState } from "react";

const STORAGE_KEY = "openamrRegisteredDevices";

const loadDevices = () => {
  try {
    return JSON.parse(localStorage.getItem(STORAGE_KEY) || "[]");
  } catch {
    return [];
  }
};

/**
 * Manually registered external devices (USB/CAN/network/Raspberry-Pi
 * attached hardware). This app has no OS-level access to auto-detect
 * plug-and-play hardware, so an operator registers what's connected and
 * the Devices page watches whatever ROS topic that device's driver
 * publishes for live status — the same pattern SystemHealth.jsx already
 * uses for built-in topics.
 */
export default function useDevices() {
  const [devices, setDevices] = useState(loadDevices);

  useEffect(() => {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(devices));
  }, [devices]);

  const addDevice = (device) => {
    setDevices((prev) => [...prev, { id: Date.now(), ...device }]);
  };

  const removeDevice = (id) => {
    setDevices((prev) => prev.filter((d) => d.id !== id));
  };

  return { devices, addDevice, removeDevice };
}
