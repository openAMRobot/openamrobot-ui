import React from "react";
import * as THREE from "three";

import { DashboardCard, EmptyState } from "../../../shared/ui/Dashboard";

const fmt = (n, digits = 4) => (typeof n === "number" ? n.toFixed(digits).replace(/\.?0+$/, (m) => (m === "." ? "" : m)) : "—");
const fmtVec = (v, digits = 4) => (v ? `${fmt(v.x ?? v[0], digits)}, ${fmt(v.y ?? v[1], digits)}, ${fmt(v.z ?? v[2], digits)}` : "—");

const Row = ({ label, children }) => (
  <>
    <dt className="text-[11px] font-semibold uppercase tracking-wide text-themeTextGray/70">{label}</dt>
    <dd className="min-w-0 break-words text-[12px] text-textWhiteHover">{children}</dd>
  </>
);

function describeGeometryGroup(group, meshesReady) {
  if (!group) return "none";
  const mesh = group.children.find((c) => c.isMesh);
  if (!mesh) return meshesReady ? "none" : "loading…";
  if (mesh.userData.isPlaceholder) {
    return `⚠ missing mesh (${mesh.userData.sourcePath || "unknown path"})`;
  }
  const geomType = mesh.geometry?.type;
  if (geomType === "BoxGeometry") {
    return `box ${fmt(mesh.scale.x, 3)} × ${fmt(mesh.scale.y, 3)} × ${fmt(mesh.scale.z, 3)} m`;
  }
  if (geomType === "SphereGeometry") {
    return `sphere, radius ${fmt(mesh.scale.x, 3)} m`;
  }
  if (geomType === "CylinderGeometry") {
    return `cylinder, radius ${fmt(mesh.scale.x, 3)} m, length ${fmt(mesh.scale.y, 3)} m`;
  }
  const filename = mesh.userData.sourcePath ? mesh.userData.sourcePath.split("/").pop() : "mesh";
  return `mesh (${filename})`;
}

const tempEuler = new THREE.Euler();

const JointInfo = ({ joint }) => {
  const name = joint.urdfName || joint.name;
  const parentLink = joint.parent?.isURDFLink ? joint.parent.urdfName || joint.parent.name : "—";
  const childLink = joint.children.find((c) => c.isURDFLink);
  const childLinkName = childLink ? childLink.urdfName || childLink.name : "—";
  const isFixed = joint.jointType === "fixed";
  const isContinuous = joint.jointType === "continuous";
  const angle = joint.angle ?? 0;
  const rpy = tempEuler.setFromQuaternion(joint.quaternion, "ZYX");
  const isMimic = Boolean(joint.mimicJoint);

  return (
    <dl className="grid grid-cols-[auto_1fr] gap-x-3 gap-y-2">
      <Row label="Joint name">{name}</Row>
      <Row label="Joint type">{joint.jointType}</Row>
      <Row label="Parent link">{parentLink}</Row>
      <Row label="Child link">{childLinkName}</Row>
      <Row label="Joint axis">{isFixed ? "n/a (fixed)" : fmtVec(joint.axis, 2)}</Row>
      <Row label="Current position">
        {isFixed ? "n/a (fixed)" : `${fmt(angle, 4)} rad (${fmt((angle * 180) / Math.PI, 2)}°)`}
      </Row>
      <Row label="Lower / upper limit">
        {isFixed
          ? "n/a (fixed)"
          : isContinuous
            ? "unlimited (continuous joint)"
            : `${fmt(joint.limit.lower, 4)} … ${fmt(joint.limit.upper, 4)} rad`}
      </Row>
      <Row label="Velocity limit">
        {!isFixed && joint.limit.velocity ? `${fmt(joint.limit.velocity, 4)} rad/s` : "not specified in URDF"}
      </Row>
      <Row label="Effort limit">
        {!isFixed && joint.limit.effort ? `${fmt(joint.limit.effort, 4)} N·m` : "not specified in URDF"}
      </Row>
      <Row label="Origin position">{fmtVec(joint.position, 4)} m</Row>
      <Row label="Origin orientation (rpy)">
        {fmt(rpy.x, 4)}, {fmt(rpy.y, 4)}, {fmt(rpy.z, 4)} rad
      </Row>
      <Row label="Mimic relationship">
        {isMimic
          ? `mimics "${joint.mimicJoint}" (value = ${fmt(joint.multiplier, 3)} × source + ${fmt(joint.offset, 3)})`
          : joint.mimicJoints?.length
            ? `mimicked by: ${joint.mimicJoints.map((j) => j.urdfName || j.name).join(", ")}`
            : "none"}
      </Row>
    </dl>
  );
};

const ComponentInspector = ({ robot, selected, meshesReady }) => {
  if (!robot || !selected) {
    return (
      <DashboardCard className="p-3">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Component inspector
        </p>
        <EmptyState
          className="min-h-[140px] px-0"
          title="Nothing selected"
          description="Select a link or joint in the tree or the 3D viewer."
        />
      </DashboardCard>
    );
  }

  const node = selected.kind === "link" ? robot.links?.[selected.name] : robot.joints?.[selected.name];
  if (!node) {
    return (
      <DashboardCard className="p-3">
        <EmptyState className="min-h-[140px] px-0" title="Not found" description={`"${selected.name}" is not in the current model.`} />
      </DashboardCard>
    );
  }

  return (
    <DashboardCard className="max-h-[420px] overflow-y-auto p-3">
      <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
        {selected.kind === "link" ? "Link information" : "Joint information"}
      </p>
      <div className="mt-2">
        {selected.kind === "link" ? (
          <LinkFields link={node} meshesReady={meshesReady} />
        ) : (
          <JointInfo joint={node} />
        )}
      </div>
    </DashboardCard>
  );
};

const LinkFields = ({ link, meshesReady }) => {
  const name = link.urdfName || link.name;
  const parentJoint = link.parent?.isURDFJoint ? link.parent.urdfName || link.parent.name : null;
  const childJoints = link.children.filter((c) => c.isURDFJoint).map((j) => j.urdfName || j.name);
  const { mass, origin, inertia } = link.inertial || { mass: 0, origin: { xyz: [0, 0, 0] }, inertia: {} };
  // robot.visual / robot.colliders are keyed by the *optional* name attribute
  // on <visual>/<collision> elements (rarely set — not set anywhere in this
  // robot's URDF), so look these up as direct children of the link instead.
  const visualGroup = link.children.find((c) => c.isURDFVisual);
  const collisionGroup = link.children.find((c) => c.isURDFCollider);

  return (
    <dl className="grid grid-cols-[auto_1fr] gap-x-3 gap-y-2">
      <Row label="Link name">{name}</Row>
      <Row label="Associated frame">
        <code>{name}</code>
      </Row>
      <Row label="Parent joint">{parentJoint || "— (root link)"}</Row>
      <Row label="Child joints">{childJoints.length ? childJoints.join(", ") : "none"}</Row>
      <Row label="Visual geometry">{describeGeometryGroup(visualGroup, meshesReady)}</Row>
      <Row label="Collision geometry">{describeGeometryGroup(collisionGroup, meshesReady)}</Row>
      <Row label="Mass">{mass ? `${fmt(mass, 4)} kg` : "not specified"}</Row>
      <Row label="Centre of mass">{mass ? `${fmtVec(origin.xyz, 4)} m` : "not specified"}</Row>
      <Row label="Inertia (ixx, iyy, izz)">
        {mass
          ? `${fmt(inertia.ixx, 6)}, ${fmt(inertia.iyy, 6)}, ${fmt(inertia.izz, 6)} kg·m²`
          : "not specified"}
      </Row>
      <Row label="Inertia (ixy, ixz, iyz)">
        {mass
          ? `${fmt(inertia.ixy, 6)}, ${fmt(inertia.ixz, 6)}, ${fmt(inertia.iyz, 6)} kg·m²`
          : "not specified"}
      </Row>
    </dl>
  );
};

export default ComponentInspector;
