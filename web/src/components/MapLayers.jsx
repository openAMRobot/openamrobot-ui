import React, { useEffect, useState } from "react";

const DEFAULT_VISIBLE = {
  map: true,
  costmap: false,
  scan: false,
  path: true,
  goal: true,
  waypoints: true,
  robotTrail: false,
};

const DEFAULT_OPACITY = {
  costmap: 0.35,
  scan: 0.32,
  path: 0.95,
  robotTrail: 0.65,
};

const TOGGLES = [
  ["map", "Map"],
  ["costmap", "Costmap"],
  ["scan", "Laser"],
  ["path", "Path"],
  ["goal", "Goal"],
  ["waypoints", "Waypoints"],
  ["robotTrail", "Trail"],
];

const OPACITY = [
  ["costmap", "Costmap"],
  ["scan", "Laser"],
  ["path", "Path"],
  ["robotTrail", "Trail"],
];

const MapLayers = () => {
  const [visible, setVisible] = useState(DEFAULT_VISIBLE);
  const [opacity, setOpacity] = useState(DEFAULT_OPACITY);

  useEffect(() => {
    if (!window.NAV2D) return;
    Object.entries(visible).forEach(([layer, enabled]) => {
      window.NAV2D.setLayerVisible?.(layer, enabled);
    });
    Object.entries(opacity).forEach(([layer, value]) => {
      window.NAV2D.setLayerOpacity?.(layer, value);
    });
  }, [visible, opacity]);

  return (
    <div className="rounded-xl border border-borderSubtle bg-bgCard px-3 py-2 font-[RobotoMono]">
      <div className="mb-1.5 flex items-center gap-3">
        <p className="shrink-0 text-xs uppercase tracking-wider text-themeTextGray">
          Layers
        </p>
        <div className="grid min-w-0 flex-1 grid-cols-4 gap-1.5 xl:grid-cols-7">
          {TOGGLES.map(([key, label]) => (
            <label
              key={key}
              className={`flex min-h-[24px] items-center gap-1.5 rounded-md px-2 py-0.5 text-xs ${
                visible[key]
                  ? "bg-themeBlue/10 text-themeBlue"
                  : "bg-bgSurface text-themeTextGray"
              }`}
            >
              <input
                type="checkbox"
                checked={visible[key]}
                onChange={(e) =>
                  setVisible((prev) => ({ ...prev, [key]: e.target.checked }))
                }
                className="accent-themeBlue"
              />
              {label}
            </label>
          ))}
        </div>
        <button
          onClick={() => {
            setVisible(DEFAULT_VISIBLE);
            setOpacity(DEFAULT_OPACITY);
          }}
          className="shrink-0 text-[10px] font-semibold text-themeBlue hover:underline"
        >
          Reset
        </button>
      </div>

      <div className="grid grid-cols-2 gap-x-3 gap-y-1 xl:grid-cols-4">
        {OPACITY.map(([key, label]) => (
          <label key={key} className="text-[10px] uppercase text-themeTextGray">
            <div className="flex justify-between">
              <span>{label}</span>
              <span>{Math.round(opacity[key] * 100)}%</span>
            </div>
            <input
              type="range"
              min="0.05"
              max="1"
              step="0.05"
              value={opacity[key]}
              onChange={(e) =>
                setOpacity((prev) => ({
                  ...prev,
                  [key]: parseFloat(e.target.value),
                }))
              }
              className="w-full accent-themeBlue"
            />
          </label>
        ))}
      </div>
    </div>
  );
};

export default MapLayers;
