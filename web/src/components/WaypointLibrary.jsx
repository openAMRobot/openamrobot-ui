import React, { useEffect, useRef, useState } from "react";
import { toast } from "react-toastify";

import { useRos } from "../app/App";
import { AppConfig } from "../shared/constants";

/**
 * Named, persisted goal poses ("Dock", "Loading bay") the operator can save
 * once and send to with one click — distinct from MapPage's waypoint queue,
 * which is an ephemeral, unnamed sequence built by clicking the map and
 * cleared after it runs. The list itself lives in the parent (via
 * useSavedWaypoints) so the map's saved-waypoint pins and context menu can
 * share and mutate the exact same data; this component is presentational.
 */
const WaypointLibrary = ({ waypoints, onAdd, onGo, onRemove }) => {
  const ros = useRos();
  const [name, setName] = useState("");
  const [hasPose, setHasPose] = useState(false);

  const currentPoseRef = useRef(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    const amclTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.AMCL_POSE_TOPIC,
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });
    amclTopic.subscribe((msg) => {
      const pos = msg?.pose?.pose?.position;
      const ori = msg?.pose?.pose?.orientation;
      if (!pos || !ori) return;
      // Nested {position, orientation} shape — matches what the map's
      // right-click "Save waypoint here" passes, so onAdd (saveWaypointAt
      // in MapPage.jsx) has one consistent input shape regardless of
      // whether the pose came from here or a map click.
      currentPoseRef.current = {
        position: { x: pos.x, y: pos.y, z: 0 },
        orientation: { x: 0, y: 0, z: ori.z, w: ori.w },
      };
      setHasPose(true);
    });

    return () => amclTopic.unsubscribe();
  }, [ros]);

  const handleSave = () => {
    const trimmed = name.trim();
    if (!trimmed) {
      toast.warn("Enter a name for this waypoint");
      return;
    }
    if (!currentPoseRef.current) {
      toast.warn("No localized position yet — waiting for AMCL");
      return;
    }
    onAdd(trimmed, currentPoseRef.current);
    setName("");
  };

  return (
    <div className="dashboard-card p-3 font-[RobotoMono]">
      <p className="mb-2 text-xs uppercase tracking-wider text-themeTextGray">
        Saved Waypoints
      </p>

      <div className="mb-2 flex gap-2">
        <input
          type="text"
          value={name}
          onChange={(e) => setName(e.target.value)}
          onKeyDown={(e) => e.key === "Enter" && handleSave()}
          placeholder={hasPose ? "e.g. Dock, Loading bay" : "Waiting for pose…"}
          disabled={!hasPose}
          className="min-w-0 flex-1 rounded-lg border border-borderSubtle bg-bgSurface px-3 py-1.5 text-xs text-textWhiteHover placeholder:text-themeTextGray disabled:opacity-50"
        />
        <button
          onClick={handleSave}
          disabled={!hasPose}
          className="shrink-0 rounded-lg border border-themeBlue px-3 py-1.5 text-xs font-semibold text-themeBlue transition-colors hover:bg-themeBlue hover:text-white disabled:cursor-not-allowed disabled:opacity-40"
        >
          Save here
        </button>
      </div>

      {waypoints.length === 0 ? (
        <p className="text-xs text-themeTextGray opacity-70">
          No saved waypoints yet — drive somewhere and save it, or right-click
          the map.
        </p>
      ) : (
        <div className="flex flex-wrap gap-1.5">
          {waypoints.map((wp) => (
            <div
              key={wp.id}
              className="flex items-center gap-1.5 rounded-lg border border-borderSubtle bg-bgSurface px-2 py-1 text-xs"
            >
              <button
                onClick={() => onGo(wp)}
                className="text-themeBlue hover:underline"
                title={`(${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})`}
              >
                ▸ {wp.name}
              </button>
              <button
                onClick={() => onRemove(wp.id)}
                className="text-themeTextGray hover:text-statusRed"
                aria-label={`Delete ${wp.name}`}
              >
                ×
              </button>
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default WaypointLibrary;
