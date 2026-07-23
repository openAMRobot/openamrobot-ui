import * as THREE from "three";

// Flags every object in a helper's subtree (e.g. ArrowHelper's internal
// line + cone meshes) as non-geometry so bounding-box and raycast/highlight
// code elsewhere can cheaply skip all of it with a single userData check,
// regardless of how many internal children a given helper type has.
function markAsLayerHelper(object) {
  object.traverse((node) => {
    node.userData.isLayerHelper = true;
  });
  return object;
}

// Display-layer helpers for a loaded URDFRobot. Every helper is attached as
// a child of the link/joint Object3D it describes so it inherits that
// node's transform automatically — nothing here tracks positions by hand,
// and nothing hardcodes a link/joint name, so this works for any URDF the
// robot-description backend serves in the future, not just OpenAMRobot.

function makeLabelSprite(text) {
  const canvas = document.createElement("canvas");
  const ctx = canvas.getContext("2d");
  const fontSize = 44;
  ctx.font = `${fontSize}px monospace`;
  const textWidth = ctx.measureText(text).width;
  const width = Math.ceil(textWidth) + 20;
  const height = fontSize + 16;
  canvas.width = width;
  canvas.height = height;

  ctx.font = `${fontSize}px monospace`;
  ctx.fillStyle = "rgba(8, 8, 10, 0.82)"; // surface-1
  ctx.fillRect(0, 0, width, height);
  ctx.fillStyle = "#f5f5f7"; // text-primary
  ctx.textBaseline = "middle";
  ctx.fillText(text, 10, height / 2);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const material = new THREE.SpriteMaterial({
    map: texture,
    depthTest: false,
    depthWrite: false,
    transparent: true,
    sizeAttenuation: true,
  });
  const sprite = new THREE.Sprite(material);
  // Canvas is rasterized at `fontSize` px for crisp text, then scaled down
  // into small world-space units — this robot is well under 1m, so labels
  // need to land around a few centimeters tall, not meters.
  const scale = 0.0005;
  sprite.scale.set(width * scale, height * scale, 1);
  sprite.renderOrder = 999;
  sprite.visible = false;
  return sprite;
}

// robot.visual / robot.colliders (from urdf-loader) are keyed by the
// *optional* name attribute on <visual>/<collision> elements, which most
// URDFs (including this one) never set — so those dicts end up empty here.
// Collecting the real groups by walking the tree and checking the
// isURDFVisual/isURDFCollider flag works regardless of whether that
// optional name was ever specified.
function collectByFlag(robot, flagName) {
  const found = [];
  robot.traverse((obj) => {
    if (obj[flagName]) found.push(obj);
  });
  return found;
}

/**
 * Builds every optional display-layer object for a loaded robot and parents
 * each one where it belongs in the URDF tree. Returns flat arrays of the
 * created Object3Ds so the caller can toggle `.visible` per layer.
 */
export function buildDisplayLayers(robot) {
  const tfAxes = [];
  const jointAxes = [];
  const linkLabels = [];
  const jointLabels = [];
  const comMarkers = [];
  const visualGroups = collectByFlag(robot, "isURDFVisual");
  const colliderGroups = collectByFlag(robot, "isURDFCollider");

  Object.values(robot.links || {}).forEach((link) => {
    const axes = markAsLayerHelper(new THREE.AxesHelper(0.09));
    axes.visible = false;
    link.add(axes);
    tfAxes.push(axes);

    const label = markAsLayerHelper(makeLabelSprite(link.urdfName || link.name));
    label.position.set(0, 0, 0.04);
    link.add(label);
    linkLabels.push(label);

    const { mass, origin } = link.inertial || {};
    if (mass && mass > 0) {
      const marker = markAsLayerHelper(
        new THREE.Mesh(
          new THREE.SphereGeometry(Math.max(0.012, Math.cbrt(mass) * 0.01), 12, 12),
          // technical-cyan — centre of mass is spatial/technical data, not a status indicator
          new THREE.MeshBasicMaterial({ color: 0x38bdf8 }),
        ),
      );
      marker.visible = false;
      marker.position.set(origin.xyz[0], origin.xyz[1], origin.xyz[2]);
      link.add(marker);
      comMarkers.push(marker);
    }
  });

  Object.values(robot.joints || {}).forEach((joint) => {
    const label = markAsLayerHelper(makeLabelSprite(joint.urdfName || joint.name));
    label.position.set(0, 0, 0.06);
    joint.add(label);
    jointLabels.push(label);

    if (joint.jointType !== "fixed") {
      const dir = joint.axis.clone().normalize();
      // technical-cyan — a persistent spatial indicator, distinct from the
      // violet/pink used for the moment of selecting a link or joint.
      const arrow = markAsLayerHelper(
        new THREE.ArrowHelper(dir, new THREE.Vector3(0, 0, 0), 0.14, 0x38bdf8, 0.035, 0.02),
      );
      arrow.visible = false;
      joint.add(arrow);
      jointAxes.push(arrow);
    }
  });

  return { tfAxes, jointAxes, linkLabels, jointLabels, comMarkers, visualGroups, colliderGroups };
}

/**
 * Computes an axis-aligned footprint outline (ground-plane bounding
 * rectangle) from every collision object's current world-space bounds. Not
 * a URDF concept — no `<footprint>` element exists — so this is explicitly
 * a derived/computed layer, built from real collision geometry rather than
 * fabricated. Takes the collider group list already collected by
 * buildDisplayLayers rather than re-traversing the robot.
 */
export function buildFootprintOutline(robot, colliderGroups) {
  const colliders = colliderGroups || collectByFlag(robot, "isURDFCollider");
  if (colliders.length === 0) return null;

  const box = new THREE.Box3();
  const objectBox = new THREE.Box3();
  let hasBounds = false;

  colliders.forEach((collider) => {
    objectBox.setFromObject(collider);
    if (!objectBox.isEmpty()) {
      box.union(objectBox);
      hasBounds = true;
    }
  });

  if (!hasBounds) return null;

  const z = box.min.z;
  const points = [
    new THREE.Vector3(box.min.x, box.min.y, z),
    new THREE.Vector3(box.max.x, box.min.y, z),
    new THREE.Vector3(box.max.x, box.max.y, z),
    new THREE.Vector3(box.min.x, box.max.y, z),
    new THREE.Vector3(box.min.x, box.min.y, z),
  ];
  const geometry = new THREE.BufferGeometry().setFromPoints(points);
  // technical-cyan — computed spatial boundary, not a status/health signal
  const material = new THREE.LineBasicMaterial({ color: 0x38bdf8 });
  const line = markAsLayerHelper(new THREE.Line(geometry, material));
  line.visible = false;
  return line;
}

export function setLayerVisible(objects, visible) {
  (objects || []).forEach((obj) => {
    obj.visible = visible;
  });
}
