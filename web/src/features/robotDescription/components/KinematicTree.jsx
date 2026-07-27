import React, { useState } from "react";

import { DashboardCard, EmptyState } from "../../../shared/ui/Dashboard";

const rowClasses = (isSelected) =>
  `flex min-w-0 cursor-pointer items-center gap-1.5 rounded-md px-1.5 py-1 text-[12px] transition-colors ${
    isSelected
      ? "bg-themeBlue/15 text-themeBlue"
      : "text-themeTextGray hover:bg-bgCard/70 hover:text-textWhiteHover"
  }`;

const Toggle = ({ expanded, onToggle }) => (
  <button
    type="button"
    onClick={(e) => {
      e.stopPropagation();
      onToggle();
    }}
    className="flex h-4 w-4 flex-none items-center justify-center text-[10px] text-themeTextGray/70 hover:text-themeBlue"
    aria-label={expanded ? "Collapse" : "Expand"}
  >
    {expanded ? "▾" : "▸"}
  </button>
);

const JointNode = ({ joint, selected, onSelect }) => {
  const [expanded, setExpanded] = useState(true);
  const childLink = joint.children.find((c) => c.isURDFLink);
  const name = joint.urdfName || joint.name;
  const isSelected = selected?.kind === "joint" && selected.name === name;
  const isFixed = joint.jointType === "fixed";

  return (
    <li>
      <div className={rowClasses(isSelected)} onClick={() => onSelect(name, "joint")}>
        {childLink ? <Toggle expanded={expanded} onToggle={() => setExpanded((x) => !x)} /> : <span className="w-4" />}
        <span className={`flex-none ${isFixed ? "text-themeTextGray/50" : "text-statusYellow"}`} aria-hidden="true">
          {isFixed ? "⏚" : "⟳"}
        </span>
        <span className="truncate">{name}</span>
        <span className="ml-auto flex-none font-[RobotoMono] text-[9px] uppercase tracking-wide text-themeTextGray/50">
          {joint.jointType}
        </span>
      </div>
      {expanded && childLink ? (
        <ul className="ml-4 border-l border-borderSubtle/40 pl-2">
          <LinkNode link={childLink} selected={selected} onSelect={onSelect} />
        </ul>
      ) : null}
    </li>
  );
};

const LinkNode = ({ link, selected, onSelect }) => {
  const [expanded, setExpanded] = useState(true);
  const childJoints = link.children.filter((c) => c.isURDFJoint);
  const name = link.urdfName || link.name;
  const isSelected = selected?.kind === "link" && selected.name === name;

  return (
    <li>
      <div className={rowClasses(isSelected)} onClick={() => onSelect(name, "link")}>
        {childJoints.length > 0 ? (
          <Toggle expanded={expanded} onToggle={() => setExpanded((x) => !x)} />
        ) : (
          <span className="w-4" />
        )}
        <span className="flex-none text-themeBlue" aria-hidden="true">
          ◆
        </span>
        <span className="truncate">{name}</span>
      </div>
      {expanded && childJoints.length > 0 ? (
        <ul className="ml-4 border-l border-borderSubtle/40 pl-2">
          {childJoints.map((joint) => (
            <JointNode key={joint.name} joint={joint} selected={selected} onSelect={onSelect} />
          ))}
        </ul>
      ) : null}
    </li>
  );
};

const KinematicTree = ({ robot, selected, onSelect }) => (
  <DashboardCard className="flex h-full min-h-0 flex-col p-3">
    <p className="flex-none font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
      Kinematic tree
    </p>
    <div className="mt-2 min-h-0 flex-1 overflow-y-auto">
      {robot ? (
        <ul>
          <LinkNode link={robot} selected={selected} onSelect={onSelect} />
        </ul>
      ) : (
        <EmptyState className="min-h-[120px] px-0" title="No model loaded" description="Waiting for the robot description." />
      )}
    </div>
  </DashboardCard>
);

export default KinematicTree;
