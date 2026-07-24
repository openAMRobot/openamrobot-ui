import { useEffect, useState } from "react";

export const SAVED_WAYPOINTS_KEY = "openamrSavedWaypoints";

export const loadWaypoints = () => {
  try {
    return JSON.parse(localStorage.getItem(SAVED_WAYPOINTS_KEY) || "[]");
  } catch {
    return [];
  }
};

/**
 * Named, persisted goal poses ("Dock", "Loading bay") — single source of
 * truth shared between WaypointLibrary.jsx's list UI and the map's saved-
 * waypoint pins/context menu, so both can add/remove/click-to-go against
 * the same list. Also keeps window.NAV2D.savedWaypointItems (the map pins)
 * in sync any time the list changes, from whichever page owns this hook.
 */
export default function useSavedWaypoints() {
  const [waypoints, setWaypoints] = useState(loadWaypoints);

  useEffect(() => {
    localStorage.setItem(SAVED_WAYPOINTS_KEY, JSON.stringify(waypoints));
    window.NAV2D?.setSavedWaypoints?.(waypoints);
  }, [waypoints]);

  const addWaypoint = (name, pose) => {
    setWaypoints((prev) => [...prev, { id: Date.now(), name, ...pose }]);
  };

  const removeWaypoint = (id) => {
    setWaypoints((prev) => prev.filter((wp) => wp.id !== id));
  };

  return { waypoints, addWaypoint, removeWaypoint };
}
