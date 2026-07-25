import { useCallback, useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import URDFLoader from "urdf-loader";

import { resolveRobotDescriptionUrl } from "../api/robotDescriptionApi";
import { buildDisplayLayers, buildFootprintOutline, setLayerVisible } from "./layers";

// Selection highlight — violet for a selected link, pink for the child link
// of a selected joint. Reserved for the moment of interaction; persistent
// technical layers (axes/frames/COM/footprint, see layers.js) use cyan instead.
const HIGHLIGHT_COLOR = new THREE.Color(0x8b5cf6);
const JOINT_HIGHLIGHT_COLOR = new THREE.Color(0xec6f91);

function computeRealGeometryBounds(robot) {
  const box = new THREE.Box3();
  let hasAny = false;
  robot.traverse((obj) => {
    if (obj.isMesh && !obj.userData.isLayerHelper) {
      box.expandByObject(obj);
      hasAny = true;
    }
  });
  return hasAny ? box : null;
}

function collectMeshes(node) {
  const meshes = [];
  if (node) {
    node.traverse((obj) => {
      if (obj.isMesh && !obj.userData.isLayerHelper) meshes.push(obj);
    });
  }
  return meshes;
}

/**
 * Owns the imperative three.js side of the Robot Description viewer: scene,
 * camera, renderer, controls, the loaded URDFRobot, display-layer helpers,
 * selection highlighting, and raycast picking. Everything here is generic
 * over whatever URDF the manifest points at — no OpenAMRobot-specific link
 * or joint names appear anywhere in this file.
 */
export default function useRobotScene({ manifest, layers, selected, onSelect, pose, path }) {
  const containerRef = useRef(null);
  const rendererRef = useRef(null);
  const sceneRef = useRef(null);
  const cameraRef = useRef(null);
  const controlsRef = useRef(null);
  const robotRef = useRef(null);
  const robotGroupRef = useRef(null);
  const layerObjectsRef = useRef(null);
  const footprintRef = useRef(null);
  const pathLineRef = useRef(null);
  const highlightedRef = useRef([]);
  const rafRef = useRef(null);

  const [status, setStatus] = useState({
    stage: "idle",
    error: null,
    meshErrors: [],
    loaded: 0,
    total: 0,
  });
  const [isFullscreen, setIsFullscreen] = useState(false);
  // Structure (links/joints/limits) is available the instant the URDF is
  // parsed, well before mesh geometry finishes loading — expose it as state
  // as soon as we have it so the tree/inspector can render immediately.
  const [robot, setRobot] = useState(null);

  const frameRobot = useCallback((preserveAngle) => {
    const robot = robotRef.current;
    const camera = cameraRef.current;
    const controls = controlsRef.current;
    if (!robot || !camera || !controls) return;

    const box = computeRealGeometryBounds(robot);
    if (!box) return;

    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const radius = Math.max(size.length() / 2, 0.05);
    const fovRad = (camera.fov * Math.PI) / 180;
    const distance = (radius / Math.sin(fovRad / 2)) * 1.35;

    let direction;
    if (preserveAngle) {
      direction = camera.position.clone().sub(controls.target);
      if (direction.lengthSq() < 1e-6) direction = new THREE.Vector3(1, -1, 0.8);
      direction.normalize();
    } else {
      direction = new THREE.Vector3(1, -1, 0.8).normalize();
    }

    controls.target.copy(center);
    camera.position.copy(center).addScaledVector(direction, distance);
    camera.near = Math.max(distance / 100, 0.01);
    camera.far = distance * 100;
    camera.updateProjectionMatrix();
    controls.update();
  }, []);

  const resetView = useCallback(() => frameRobot(false), [frameRobot]);
  const fitToScreen = useCallback(() => frameRobot(true), [frameRobot]);

  const toggleFullscreen = useCallback(() => {
    const container = containerRef.current;
    if (!container) return;
    if (document.fullscreenElement) {
      document.exitFullscreen();
    } else {
      container.requestFullscreen?.();
    }
  }, []);

  // ── One-time scene/camera/renderer/controls setup ──────────────────────
  useEffect(() => {
    const container = containerRef.current;
    if (!container) return undefined;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x030304); // --background
    sceneRef.current = scene;

    const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
    camera.up.set(0, 0, 1); // URDF/ROS convention: Z up
    camera.position.set(0.9, -0.9, 0.7);
    cameraRef.current = camera;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    rendererRef.current = renderer;
    container.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.target.set(0, 0, 0.15);
    controlsRef.current = controls;

    // Neutral graphite lighting, not a blue sci-fi cast — a faint violet
    // tint on the fill light nods at the accent palette without tinting
    // the whole model.
    scene.add(new THREE.AmbientLight(0x4a4a52, 0.7));
    const keyLight = new THREE.DirectionalLight(0xffffff, 1.1);
    keyLight.position.set(1.2, -1.4, 2);
    scene.add(keyLight);
    const fillLight = new THREE.DirectionalLight(0x6d6a8a, 0.35);
    fillLight.position.set(-1.5, 1, 1);
    scene.add(fillLight);

    const grid = new THREE.GridHelper(4, 40, 0x24242c, 0x18181d);
    grid.rotation.x = Math.PI / 2; // GridHelper defaults to the XZ plane; rotate into ground-plane XY
    scene.add(grid);

    const worldAxes = new THREE.AxesHelper(0.5);
    scene.add(worldAxes);

    const robotGroup = new THREE.Group();
    scene.add(robotGroup);
    robotGroupRef.current = robotGroup;

    const resize = () => {
      const width = container.clientWidth || 1;
      const height = container.clientHeight || 1;
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderer.setSize(width, height);
    };
    resize();
    const resizeObserver = new ResizeObserver(resize);
    resizeObserver.observe(container);

    let raycastStart = null;
    const raycaster = new THREE.Raycaster();
    const pointerNdc = new THREE.Vector2();

    const onPointerDown = (event) => {
      raycastStart = { x: event.clientX, y: event.clientY };
    };
    const onPointerUp = (event) => {
      if (!raycastStart) return;
      const dx = event.clientX - raycastStart.x;
      const dy = event.clientY - raycastStart.y;
      raycastStart = null;
      if (Math.hypot(dx, dy) > 4) return; // treat as an orbit drag, not a click

      const robot = robotRef.current;
      if (!robot) return;
      const rect = renderer.domElement.getBoundingClientRect();
      pointerNdc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
      pointerNdc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointerNdc, camera);
      const hits = raycaster.intersectObject(robot, true);
      const hit = hits.find((h) => h.object.isMesh && !h.object.userData.isLayerHelper);
      if (!hit) return;

      let node = hit.object;
      while (node && !node.isURDFLink && !node.isURDFJoint) node = node.parent;
      if (node) onSelect?.(node.urdfName || node.name, node.isURDFJoint ? "joint" : "link");
    };
    renderer.domElement.addEventListener("pointerdown", onPointerDown);
    renderer.domElement.addEventListener("pointerup", onPointerUp);

    const animate = () => {
      controls.update();
      renderer.render(scene, camera);
      rafRef.current = requestAnimationFrame(animate);
    };
    animate();

    const onFullscreenChange = () => {
      setIsFullscreen(Boolean(document.fullscreenElement));
      resize();
    };
    document.addEventListener("fullscreenchange", onFullscreenChange);

    return () => {
      cancelAnimationFrame(rafRef.current);
      document.removeEventListener("fullscreenchange", onFullscreenChange);
      resizeObserver.disconnect();
      renderer.domElement.removeEventListener("pointerdown", onPointerDown);
      renderer.domElement.removeEventListener("pointerup", onPointerUp);
      controls.dispose();
      renderer.dispose();
      container.removeChild(renderer.domElement);
      if (pathLineRef.current) {
        scene.remove(pathLineRef.current);
        pathLineRef.current.geometry.dispose();
        pathLineRef.current.material.dispose();
        pathLineRef.current = null;
      }
    };
    // Intentional one-time setup (empty deps): onSelect is read via a
    // ref-stable closure and only invoked from event handlers torn down
    // together with this same effect, so it doesn't need to be a dep here.
  }, []);

  // ── Load the robot whenever the manifest becomes available ─────────────
  useEffect(() => {
    if (!manifest || !manifest.available) return undefined;
    let cancelled = false;

    setStatus({ stage: "loading-urdf", error: null, meshErrors: [], loaded: 0, total: 0 });

    const manager = new THREE.LoadingManager();
    manager.onProgress = (_url, loaded, total) => {
      if (cancelled) return;
      setStatus((prev) => ({ ...prev, stage: "loading-meshes", loaded, total }));
    };

    const loader = new URDFLoader(manager);
    loader.parseCollision = true;
    const packages = {};
    Object.entries(manifest.packages || {}).forEach(([pkg, url]) => {
      packages[pkg] = resolveRobotDescriptionUrl(url);
    });
    loader.packages = packages;

    loader.loadMeshCb = (path, mgr, material, done) => {
      loader.defaultMeshLoader(path, mgr, material, (obj, err) => {
        if (cancelled) return;
        if (err || !obj) {
          const placeholder = new THREE.Mesh(
            new THREE.BoxGeometry(0.05, 0.05, 0.05),
            new THREE.MeshBasicMaterial({ color: 0xff2d55, wireframe: true }),
          );
          placeholder.userData.isPlaceholder = true;
          placeholder.userData.sourcePath = path;
          setStatus((prev) => ({
            ...prev,
            meshErrors: [
              ...prev.meshErrors,
              { path, message: err ? String(err.message || err) : "Mesh failed to load" },
            ],
          }));
          done(placeholder);
          return;
        }
        obj.traverse((c) => {
          if (c.isMesh) c.userData.sourcePath = path;
        });
        done(obj);
      });
    };

    fetch(resolveRobotDescriptionUrl(manifest.urdfUrl))
      .then(async (res) => {
        if (!res.ok) {
          const body = await res.text().catch(() => "");
          console.error(`Failed to fetch URDF (${res.status}): ${body.slice(0, 300)}`);
          throw new Error(
            "Couldn't load the 3D model. Try refreshing, or check with your integrator if this keeps happening.",
          );
        }
        return res.text();
      })
      .then((urdfText) => {
        if (cancelled) return;
        const loadedRobot = loader.parse(urdfText, "");
        robotRef.current = loadedRobot;
        robotGroupRef.current?.add(loadedRobot);

        const layerObjects = buildDisplayLayers(loadedRobot);
        layerObjectsRef.current = layerObjects;
        const footprint = buildFootprintOutline(loadedRobot, layerObjects.colliderGroups);
        footprintRef.current = footprint;
        if (footprint) sceneRef.current?.add(footprint);

        setRobot(loadedRobot);

        manager.onLoad = () => {
          if (cancelled) return;
          setStatus((prev) => ({ ...prev, stage: "ready" }));
          frameRobot(false);
        };
        // No meshes queued at all (shouldn't happen for this robot, but stay correct)
        if (manager.itemsLoaded === manager.itemsTotal) {
          manager.onLoad();
        }
      })
      .catch((error) => {
        if (cancelled) return;
        setStatus((prev) => ({ ...prev, stage: "error", error: error.message }));
      });

    return () => {
      cancelled = true;
      if (robotRef.current) {
        robotRef.current.parent?.remove(robotRef.current);
      }
      if (footprintRef.current) {
        sceneRef.current?.remove(footprintRef.current);
      }
      robotRef.current = null;
      layerObjectsRef.current = null;
      footprintRef.current = null;
      setRobot(null);
    };
    // frameRobot is intentionally omitted (it's stable, declared with an
    // empty dep array above) — this effect should only re-run when the
    // manifest changes, not be retriggered by it.
  }, [manifest]);

  // Imperative setter (not a reactive prop) so the joint-values source
  // (useJointStates, which itself needs this hook's `robot` to know which
  // joints exist) can push updates in without a circular hook dependency.
  const setJointValues = useCallback((values) => {
    robotRef.current?.setJointValues(values);
  }, []);

  // ── Apply display-layer visibility toggles ──────────────────────────────
  useEffect(() => {
    const layerObjects = layerObjectsRef.current;
    const robot = robotRef.current;
    if (!layerObjects || !robot) return;

    setLayerVisible(layerObjects.visualGroups, layers.visual);
    setLayerVisible(layerObjects.colliderGroups, layers.collision);
    setLayerVisible(layerObjects.tfAxes, layers.tfAxes);
    setLayerVisible(layerObjects.jointAxes, layers.jointAxes);
    setLayerVisible(layerObjects.linkLabels, layers.linkNames);
    setLayerVisible(layerObjects.jointLabels, layers.jointNames);
    setLayerVisible(layerObjects.comMarkers, layers.centerOfMass);
    if (footprintRef.current) footprintRef.current.visible = layers.footprint;
  }, [layers, status.stage]);

  // ── Apply selection highlight (from tree OR viewer click) ──────────────
  useEffect(() => {
    highlightedRef.current.forEach(({ material, color }) => {
      material.emissive.copy(color);
    });
    highlightedRef.current = [];

    const robot = robotRef.current;
    if (!robot || !selected?.name) return;

    if (selected.kind === "link") {
      const link = robot.links?.[selected.name];
      collectMeshes(link).forEach((mesh) => {
        if (!mesh.material?.emissive) return;
        highlightedRef.current.push({ material: mesh.material, color: mesh.material.emissive.clone() });
        mesh.material.emissive.copy(HIGHLIGHT_COLOR);
      });
    } else if (selected.kind === "joint") {
      const joint = robot.joints?.[selected.name];
      const childLink = joint?.children.find((c) => c.isURDFLink);
      collectMeshes(childLink).forEach((mesh) => {
        if (!mesh.material?.emissive) return;
        highlightedRef.current.push({ material: mesh.material, color: mesh.material.emissive.clone() });
        mesh.material.emissive.copy(JOINT_HIGHLIGHT_COLOR);
      });
    }
  }, [selected]);

  // ── Live Mode: position the whole robot at its real map-frame pose ─────
  // Description Mode always passes pose=null, which snaps the robot back to
  // the origin so it behaves exactly as before this was added.
  useEffect(() => {
    const group = robotGroupRef.current;
    if (!group) return;
    if (pose) {
      group.position.set(pose.x, pose.y, 0);
      group.rotation.set(0, 0, pose.yaw);
    } else {
      group.position.set(0, 0, 0);
      group.rotation.set(0, 0, 0);
    }
  }, [pose]);

  // ── Live Mode: draw Nav2's current planned path as a line in the map frame ─
  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) return;

    if (!path || path.length < 2) {
      if (pathLineRef.current) pathLineRef.current.visible = false;
      return;
    }

    const positions = new Float32Array(path.length * 3);
    path.forEach((p, i) => {
      positions[i * 3] = p.x;
      positions[i * 3 + 1] = p.y;
      positions[i * 3 + 2] = 0.005; // just above the ground grid, avoids z-fighting
    });

    if (!pathLineRef.current) {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
      // technical-cyan — a planned path is spatial/technical data, same
      // convention as the axes/frame/footprint helpers in layers.js, not a
      // status color.
      const material = new THREE.LineBasicMaterial({ color: 0x38bdf8 });
      const line = new THREE.Line(geometry, material);
      line.userData.isLayerHelper = true; // excluded from bounds/raycast/highlight passes
      scene.add(line);
      pathLineRef.current = line;
    } else {
      const line = pathLineRef.current;
      const attr = line.geometry.getAttribute("position");
      if (attr.count !== path.length) {
        line.geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
      } else {
        attr.set(positions);
        attr.needsUpdate = true;
      }
      line.geometry.computeBoundingSphere();
      line.visible = true;
    }
  }, [path]);

  return { containerRef, status, robot, resetView, fitToScreen, toggleFullscreen, isFullscreen, setJointValues };
}
