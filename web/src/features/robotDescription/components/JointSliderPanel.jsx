import React from "react";

import { DashboardCard, EmptyState } from "../../../shared/ui/Dashboard";
import Button from "../../../shared/ui/Button";

const toDeg = (rad) => (rad * 180) / Math.PI;

const JointSliderPanel = ({ robot, jointValues, onSetJointValue, onReset, mode, selected, onSelect }) => {
  const readOnly = mode === "live";
  const movableJoints = robot ? Object.values(robot.joints).filter((j) => j.jointType !== "fixed") : [];

  return (
    <DashboardCard className="flex h-full min-h-0 flex-col p-3">
      <div className="flex flex-none items-center justify-between gap-2">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Joint controls
        </p>
      </div>

      {!robot ? (
        <EmptyState className="min-h-[100px] px-0" title="No model loaded" description="Waiting for the robot description." />
      ) : movableJoints.length === 0 ? (
        <EmptyState className="min-h-[100px] px-0" title="No movable joints" description="Every joint in this model is fixed." />
      ) : (
        <>
          <div className="mt-2 grid flex-none grid-cols-2 gap-2">
            <Button type={readOnly ? "disabled" : undefined} onBtnClick={onReset}>
              Reset joints
            </Button>
            <Button type={readOnly ? "disabled" : undefined} onBtnClick={onReset}>
              Home position
            </Button>
          </div>
          {readOnly ? (
            <p className="mt-1.5 flex-none text-[11px] text-statusYellow">
              Live Mode — sliders show telemetry only, see the mode banner above.
            </p>
          ) : null}

          <div className="mt-3 min-h-0 flex-1 space-y-3 overflow-y-auto">
            {movableJoints.map((joint) => {
              const name = joint.urdfName || joint.name;
              const value = jointValues[name] ?? 0;
              const isContinuous = joint.jointType === "continuous";
              const min = isContinuous ? -Math.PI : joint.limit.lower;
              const max = isContinuous ? Math.PI : joint.limit.upper;
              const isSelected = selected?.kind === "joint" && selected.name === name;

              return (
                <div
                  key={name}
                  className={`cursor-pointer rounded-lg p-2 transition-colors ${
                    isSelected ? "bg-themeBlue/10 ring-1 ring-themeBlue/30" : "hover:bg-bgCard/60"
                  }`}
                  onClick={() => onSelect?.(name, "joint")}
                >
                  <div className="flex items-center justify-between gap-2 text-[12px]">
                    <span className="truncate font-medium text-textWhiteHover">{name}</span>
                    <span className="flex-none font-[RobotoMono] text-[11px] text-themeTextGray">
                      {value.toFixed(3)} rad ({toDeg(value).toFixed(1)}°)
                    </span>
                  </div>
                  <input
                    type="range"
                    min={min}
                    max={max}
                    step={0.001}
                    value={value}
                    disabled={readOnly}
                    onClick={(e) => e.stopPropagation()}
                    onChange={(e) => onSetJointValue(name, parseFloat(e.target.value))}
                    className="mt-1 w-full accent-themeBlue disabled:opacity-50"
                  />
                  <div className="flex justify-between text-[10px] text-themeTextGray/60">
                    <span>{isContinuous ? "−180°" : `${min.toFixed(2)} rad`}</span>
                    <span>{isContinuous ? "continuous (no limit)" : "limit"}</span>
                    <span>{isContinuous ? "180°" : `${max.toFixed(2)} rad`}</span>
                  </div>
                </div>
              );
            })}
          </div>
        </>
      )}
    </DashboardCard>
  );
};

export default JointSliderPanel;
