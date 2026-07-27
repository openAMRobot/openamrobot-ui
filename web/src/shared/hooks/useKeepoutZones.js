import { useEffect, useState } from "react";

const STORAGE_KEY = "openamrKeepoutZones";

const load = () => {
  try {
    return JSON.parse(localStorage.getItem(STORAGE_KEY) || "[]");
  } catch {
    return [];
  }
};

/**
 * Persisted rectangular keep-out zones, in ROS map coordinates
 * ({ id, name, cx, cy, w, h } — centre + size in metres). Kept in sync with
 * the map overlay (window.NAV2D.setKeepoutZones) whenever the list changes,
 * mirroring useSavedWaypoints. NOTE: this is a visual/planning aid only;
 * enforcement needs a Nav2 keepout_filter on the robot.
 */
export default function useKeepoutZones() {
  const [zones, setZones] = useState(load);

  useEffect(() => {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(zones));
    window.NAV2D?.setKeepoutZones?.(zones);
  }, [zones]);

  const addZone = ({ name, cx, cy, w, h }) =>
    setZones((prev) => [
      ...prev,
      {
        id: `${Date.now()}`,
        name: name || `Zone ${prev.length + 1}`,
        cx: Number(cx),
        cy: Number(cy),
        w: Number(w),
        h: Number(h),
      },
    ]);

  const removeZone = (id) => setZones((prev) => prev.filter((z) => z.id !== id));

  return { zones, addZone, removeZone };
}
