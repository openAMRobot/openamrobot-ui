// Pure helpers for the read-only upper-body view on the Robot Description
// page. Nothing here talks to ROS; the hooks next to this file subscribe and
// hand plain data in. Kept free of React and three.js so Jest can test it
// directly.

// The combined OpenAMRobot 2.0 model is a candidate, not accepted (P-03
// item 25). This is the source this view was checked against. The robot does
// not report which commit it runs, so the UI shows this as "checked against",
// never as the robot's own version.
export const CANDIDATE_MODEL_SOURCE = {
  repository: "openAMRobot/openamr-upperbody-sw",
  pullRequest: 14,
  commit: "462907556cbb9c770eb814dda436cc3abca29741",
};

// Calibrated lift stops: shoulder height in mm (decisions register; the
// lift height itself is joint state of lift_joint).
export const LIFT_STOPS = [
  { id: "L1000", shoulderMm: 1000 },
  { id: "L1175", shoulderMm: 1175 },
  { id: "L1350", shoulderMm: 1350 },
];

// Head-camera pitch fixed by P-03 rev20.1. The PR #14 xacro still defaults to
// hp25; a robot reporting hp25 is flagged, not corrected.
export const DECIDED_HEAD_PITCH = "hp30";
export const STALE_HEAD_PITCH = "hp25";

export const LIFT_JOINT = "lift_joint";
export const FLOOR_FRAME = "base_footprint";
export const SHOULDER_LINKS = {
  left: "openarm_left_base_link",
  right: "openarm_right_base_link",
};

const armJoints = (side) =>
  [1, 2, 3, 4, 5, 6, 7].map((i) => `openarm_${side}_joint${i}`);
const fingerJoints = (side) => [
  `openarm_${side}_finger_joint1`,
  `openarm_${side}_finger_joint2`,
];

export const JOINT_GROUPS = [
  { id: "lift", label: "Lift", joints: [LIFT_JOINT] },
  { id: "leftArm", label: "Left arm", joints: armJoints("left") },
  { id: "rightArm", label: "Right arm", joints: armJoints("right") },
  { id: "leftGripper", label: "Left gripper", joints: fingerJoints("left") },
  { id: "rightGripper", label: "Right gripper", joints: fingerJoints("right") },
];

export const CONFIGURATION_FIELDS = [
  { key: "column", label: "Column" },
  { key: "base_plate", label: "Base plate" },
  { key: "head_pitch", label: "Head pitch" },
  { key: "base_tilt", label: "Base tilt" },
  { key: "datums_revision", label: "Datums revision" },
];

export const CONFIGURATION_UNKNOWN = "model or configuration unknown";

/**
 * Configuration identity exactly as the robot reports it. No running
 * producer exists yet (openamr-upperbody-sw PR #14, known gaps), so callers
 * pass null today. Any missing field makes the whole identity unknown: the
 * UI never fills a gap from xacro defaults or from what the model looks like.
 */
export function describeConfiguration(reported) {
  const complete =
    reported &&
    CONFIGURATION_FIELDS.every(
      ({ key }) => typeof reported[key] === "string" && reported[key].trim() !== "",
    );
  if (!complete) {
    return { known: false, message: CONFIGURATION_UNKNOWN, fields: [], flags: [] };
  }
  const fields = CONFIGURATION_FIELDS.map(({ key, label }) => ({
    key,
    label,
    value: reported[key],
  }));
  const flags = [];
  if (reported.head_pitch === STALE_HEAD_PITCH) {
    flags.push(
      `The robot reports head_pitch ${STALE_HEAD_PITCH}. P-03 rev20.1 sets the head camera pitch to ${DECIDED_HEAD_PITCH}; ${STALE_HEAD_PITCH} is the stale PR #14 xacro default.`,
    );
  }
  return { known: true, message: null, fields, flags };
}

function parseUrdf(urdfText) {
  if (typeof urdfText !== "string" || urdfText.trim() === "") return null;
  const doc = new DOMParser().parseFromString(urdfText, "application/xml");
  if (doc.getElementsByTagName("parsererror").length > 0) return null;
  const robot = doc.getElementsByTagName("robot")[0];
  return robot || null;
}

function childElements(node, tag) {
  return Array.from(node.childNodes).filter((n) => n.nodeType === 1 && n.tagName === tag);
}

function readJoints(robot) {
  return childElements(robot, "joint").map((j) => {
    const origin = childElements(j, "origin")[0];
    const axis = childElements(j, "axis")[0];
    const nums = (value, fallback) =>
      value ? value.trim().split(/\s+/).map(Number) : fallback;
    return {
      name: j.getAttribute("name"),
      type: j.getAttribute("type"),
      parent: childElements(j, "parent")[0]?.getAttribute("link"),
      child: childElements(j, "child")[0]?.getAttribute("link"),
      xyz: nums(origin?.getAttribute("xyz"), [0, 0, 0]),
      rpy: nums(origin?.getAttribute("rpy"), [0, 0, 0]),
      axis: nums(axis?.getAttribute("xyz"), [1, 0, 0]),
    };
  });
}

/** True when the URDF is the combined model: a lift_joint and OpenArm joints. */
export function isCombinedDescription(urdfText) {
  const robot = parseUrdf(urdfText);
  if (!robot) return false;
  const names = readJoints(robot).map((j) => j.name);
  return names.includes(LIFT_JOINT) && names.some((n) => /^openarm_(left|right)_joint\d$/.test(n));
}

/**
 * Links that carry placeholder geometry in the reported model: a visual or
 * collision named placeholder_* or using the placeholder_geometry material.
 */
export function findPlaceholderLinks(urdfText) {
  const robot = parseUrdf(urdfText);
  if (!robot) return [];
  return childElements(robot, "link")
    .filter((link) =>
      ["visual", "collision"].some((tag) =>
        childElements(link, tag).some(
          (el) =>
            (el.getAttribute("name") || "").startsWith("placeholder_") ||
            childElements(el, "material").some((m) => m.getAttribute("name") === "placeholder_geometry"),
        ),
      ),
    )
    .map((link) => link.getAttribute("name"));
}

// ── Minimal forward kinematics (4x4 row-major) ───────────────────────────
const identity = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

function multiply(a, b) {
  const out = new Array(16).fill(0);
  for (let r = 0; r < 4; r += 1) {
    for (let c = 0; c < 4; c += 1) {
      for (let k = 0; k < 4; k += 1) out[r * 4 + c] += a[r * 4 + k] * b[k * 4 + c];
    }
  }
  return out;
}

function fromRotation(R, t = [0, 0, 0]) {
  return [R[0], R[1], R[2], t[0], R[3], R[4], R[5], t[1], R[6], R[7], R[8], t[2], 0, 0, 0, 1];
}

function rpyMatrix([roll, pitch, yaw]) {
  const [cr, sr, cp, sp, cy, sy] = [
    Math.cos(roll), Math.sin(roll), Math.cos(pitch), Math.sin(pitch), Math.cos(yaw), Math.sin(yaw),
  ];
  // URDF: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr,
  ];
}

function axisAngle([x, y, z], angle) {
  const n = Math.hypot(x, y, z) || 1;
  const [ux, uy, uz] = [x / n, y / n, z / n];
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const t = 1 - c;
  return [
    t * ux * ux + c, t * ux * uy - s * uz, t * ux * uz + s * uy,
    t * ux * uy + s * uz, t * uy * uy + c, t * uy * uz - s * ux,
    t * ux * uz - s * uy, t * uy * uz + s * ux, t * uz * uz + c,
  ];
}

/**
 * Height (m) of `link` above `rootLink`, from the reported model and reported
 * joint positions. Returns null if the chain is missing or any movable joint
 * on it has no reported position: nothing is assumed in its place.
 */
export function linkHeightAbove(urdfText, rootLink, link, positions) {
  const robot = parseUrdf(urdfText);
  if (!robot) return null;
  const byChild = new Map(readJoints(robot).map((j) => [j.child, j]));
  const chain = [];
  let current = link;
  while (current !== rootLink) {
    const joint = byChild.get(current);
    if (!joint) return null;
    chain.unshift(joint);
    current = joint.parent;
  }
  let T = identity();
  for (const joint of chain) {
    T = multiply(T, fromRotation(rpyMatrix(joint.rpy), joint.xyz));
    if (joint.type === "fixed") continue;
    const q = positions?.[joint.name];
    if (typeof q !== "number" || !Number.isFinite(q)) return null;
    if (joint.type === "prismatic") {
      T = multiply(T, fromRotation(identity3(), joint.axis.map((a) => a * q)));
    } else if (joint.type === "revolute" || joint.type === "continuous") {
      T = multiply(T, fromRotation(axisAngle(joint.axis, q)));
    } else {
      return null;
    }
  }
  return T[11];
}

function identity3() {
  return [1, 0, 0, 0, 1, 0, 0, 0, 1];
}

/**
 * Lift readout against the calibrated stops. shoulderM is the computed
 * shoulder height in metres or null. This is a position readout only: being
 * near a stop says nothing about calibration, readiness or safety.
 */
export function liftReadout({ liftPositionM, shoulderM }) {
  const hasLift = typeof liftPositionM === "number" && Number.isFinite(liftPositionM);
  if (typeof shoulderM !== "number" || !Number.isFinite(shoulderM)) {
    return { liftPositionM: hasLift ? liftPositionM : null, shoulderMm: null, nearest: null, fraction: null };
  }
  const shoulderMm = shoulderM * 1000;
  const nearest = LIFT_STOPS.reduce((best, stop) => {
    const deltaMm = shoulderMm - stop.shoulderMm;
    return !best || Math.abs(deltaMm) < Math.abs(best.deltaMm) ? { id: stop.id, deltaMm } : best;
  }, null);
  const lo = LIFT_STOPS[0].shoulderMm;
  const hi = LIFT_STOPS[LIFT_STOPS.length - 1].shoulderMm;
  return {
    liftPositionM: hasLift ? liftPositionM : null,
    shoulderMm,
    nearest,
    fraction: (shoulderMm - lo) / (hi - lo),
  };
}
