import { useEffect, useState } from "react";

const STORAGE_KEY = "openamrRobotProfiles";

const loadProfiles = () => {
  try {
    return JSON.parse(localStorage.getItem(STORAGE_KEY) || "[]");
  } catch {
    return [];
  }
};

/**
 * Named, persisted connection presets (host/port/cameraPort) so switching
 * which robot the browser points at is a one-click pick from a saved list
 * instead of retyping an IP each time. Purely a frontend convenience layer
 * on top of the existing runtime config — selecting a profile just writes
 * its fields into RuntimeConfigContext the same way the Config page's
 * "Save settings" button already does.
 */
export default function useRobotProfiles() {
  const [profiles, setProfiles] = useState(loadProfiles);

  useEffect(() => {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(profiles));
  }, [profiles]);

  const addProfile = (name, connection) => {
    setProfiles((prev) => [...prev, { id: Date.now(), name, ...connection }]);
  };

  const removeProfile = (id) => {
    setProfiles((prev) => prev.filter((p) => p.id !== id));
  };

  return { profiles, addProfile, removeProfile };
}
