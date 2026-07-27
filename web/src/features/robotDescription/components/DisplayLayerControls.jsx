import React from "react";

import { DashboardCard } from "../../../shared/ui/Dashboard";

const LAYER_DEFS = [
  { key: "visual", label: "Visual model" },
  { key: "collision", label: "Collision geometry", requires: "collision" },
  { key: "tfAxes", label: "TF / frame axes" },
  { key: "jointAxes", label: "Joint axes", requires: "movableJoints" },
  { key: "linkNames", label: "Link names" },
  { key: "jointNames", label: "Joint names" },
  { key: "centerOfMass", label: "Centre-of-mass markers", requires: "mass" },
  { key: "footprint", label: "Workspace / footprint boundary", requires: "collision" },
];

const DisplayLayerControls = ({ robot, layers, onChange }) => {
  const availability = {
    // robot.colliders is keyed by the optional <collision name="..."> attribute,
    // which this (and most) URDFs never set, so check the actual link tree instead.
    collision: Boolean(
      robot && Object.values(robot.links || {}).some((l) => l.children.some((c) => c.isURDFCollider)),
    ),
    mass: Boolean(robot && Object.values(robot.links || {}).some((l) => l.inertial?.mass > 0)),
    movableJoints: Boolean(robot && Object.values(robot.joints || {}).some((j) => j.jointType !== "fixed")),
  };

  return (
    <DashboardCard className="p-3">
      <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
        Display layers
      </p>
      <div className="mt-2 grid grid-cols-1 gap-1.5 sm:grid-cols-2">
        {LAYER_DEFS.map(({ key, label, requires }) => {
          const supported = !requires || availability[requires];
          return (
            <label
              key={key}
              className={`flex items-center gap-2 rounded-md px-1.5 py-1 text-[12px] ${
                supported ? "text-themeTextGray hover:bg-bgCard/60" : "text-themeTextGray/30"
              }`}
              title={supported ? undefined : "Not present in this model"}
            >
              <input
                type="checkbox"
                className="accent-themeBlue"
                checked={supported && Boolean(layers[key])}
                disabled={!supported}
                onChange={(e) => onChange(key, e.target.checked)}
              />
              {label}
            </label>
          );
        })}
      </div>
    </DashboardCard>
  );
};

export default DisplayLayerControls;
