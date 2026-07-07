/* eslint-disable no-undef */
/* eslint-disable no-restricted-globals */

// NAV2D global namespace
window.NAV2D = window.NAV2D || {};

// State
window.NAV2D.pointsArray = [];
window.NAV2D.pointsFromTopic = [];
window.NAV2D.arePointsSettable = false;
window.NAV2D.canvas = null;
window.NAV2D.pointType = null;
window.NAV2D.finishedPointItem = null;
window.NAV2D.orientatedPointItem = null;
window.NAV2D.goalMarkerItem = null;
window.NAV2D.latestGoalPose = null;
window.NAV2D.pathShape = null;
window.NAV2D.robotMarker = null;
window.NAV2D.scanShape = null;
window.NAV2D.robotTrailShape = null;
window.NAV2D.robotTrail = [];
window.NAV2D.costmapItem = null;
window.NAV2D.costmapTopic = null;
window.NAV2D.queuedWaypointItems = [];
window.NAV2D.layerState = {
  map: true,
  costmap: false,
  scan: false,
  path: true,
  goal: true,
  waypoints: true,
  robotTrail: false,
};
window.NAV2D.layerOpacity = {
  costmap: 0.35,
  scan: 0.32,
  path: 0.95,
  robotTrail: 0.65,
};

window.NAV2D.mapInited = false;
window.NAV2D.scale = { x: 0, y: 0 };
window.NAV2D.mapClient = null;
window.NAV2D.mapClientScene = null;
window.NAV2D.mapClientChangeBound = false;

const TOPICS = {
  map: "/ui/map",
  globalCostmap: "/global_costmap/costmap",
  waypoints: "/WayPoints_topic",
  uiMessage: "/ui_message",
  uiOperation: "/ui_operation",
  newWaypoint: "/new_way_point",
  plan: "/plan",
  tf: "/tf",
  tfStatic: "/tf_static",
  amclPose: "/ui/amcl_pose",
  scan: "/scan_filtered",
};

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------
const getScene = () => {
  // This file is loaded before Map sometimes, so NAV2D.canvas may be null.
  const nav2d = window.NAV2D;
  if (!nav2d || !nav2d.canvas || !nav2d.canvas.scene) return null;
  return nav2d.canvas.scene;
};

const normalizeFrame = (frame) => (frame || "").replace(/^\/+/, "");

const quatToYaw = (q) =>
  Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

const toTransform2D = (translation, rotation) => ({
  x: Number(translation?.x) || 0,
  y: Number(translation?.y) || 0,
  yaw: quatToYaw(rotation || { x: 0, y: 0, z: 0, w: 1 }),
  orientation: rotation || { x: 0, y: 0, z: 0, w: 1 },
});

const invertTransform2D = (tf) => {
  const c = Math.cos(tf.yaw);
  const s = Math.sin(tf.yaw);
  return {
    x: -(c * tf.x + s * tf.y),
    y: s * tf.x - c * tf.y,
    yaw: -tf.yaw,
    orientation: {
      x: 0,
      y: 0,
      z: Math.sin(-tf.yaw / 2),
      w: Math.cos(-tf.yaw / 2),
    },
  };
};

const composeTransforms2D = (first, second) => {
  const c = Math.cos(second.yaw);
  const s = Math.sin(second.yaw);
  const yaw = first.yaw + second.yaw;
  return {
    x: second.x + c * first.x - s * first.y,
    y: second.y + s * first.x + c * first.y,
    yaw,
    orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
  };
};

// ------------------------------------------------------------
// Scale watcher (prevents crash on startup)
// ------------------------------------------------------------
window.NAV2D.checkScale = () => {
  const scene = getScene();
  if (!scene) {
    // Early init: scene not ready yet, do nothing.
    return;
  }

  // If scene exists, we can stop the polling interval (it was only for bootstrap)
  if (window.NAV2D._checkScaleIntervalId) {
    clearInterval(window.NAV2D._checkScaleIntervalId);
    window.NAV2D._checkScaleIntervalId = null;
  }

  // Detect scale changes (zoom)
  if (
    window.NAV2D.scale.x !== scene.scaleX ||
    window.NAV2D.scale.y !== scene.scaleY
  ) {
    window.NAV2D.scale.x = scene.scaleX;
    window.NAV2D.scale.y = scene.scaleY;

    window.NAV2D.pointsArray = drawPoints(window.NAV2D.pointsFromTopic, scene);
    updateNavigationOverlayScale(scene);
    drawRobotTrail(scene);
    bringNavigationOverlaysToFront(scene);
    applyLayerState();
  }
};

// Poll quickly until scene exists, then it self-stops inside checkScale()
window.NAV2D._checkScaleIntervalId = setInterval(window.NAV2D.checkScale, 100);

// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------
window.NAV2D.InitMap = (ros) => {
  console.log("InitMap");
  window.NAV2D.ros = ros;

  const scene = getScene();
  if (!scene) {
    // Map.jsx should set NAV2D.canvas before calling InitMap,
    // but guard anyway to avoid a hard crash.
    console.warn("NAV2D.InitMap called before NAV2D.canvas.scene is ready");
    return;
  }

  const topic = TOPICS.map;

  // Setup a client to get the map
  const shouldCreateClient =
    !window.NAV2D.mapClient || window.NAV2D.mapClientScene !== scene;

  const client = shouldCreateClient
    ? new window.ROS2D.OccupancyGridClient({
        ros,
        rootObject: scene,
        continuous: true,
        topic,
      })
    : window.NAV2D.mapClient;

  window.NAV2D.mapClient = client;
  window.NAV2D.mapClientScene = scene;
  if (shouldCreateClient) window.NAV2D.mapClientChangeBound = false;

  // Updating map after topic event
  if (!window.NAV2D.mapClientChangeBound) {
    window.NAV2D.mapClientChangeBound = true;
    client.on("change", () => {
      // Canvas must exist here
      if (!window.NAV2D.canvas) return;

      // Scale the canvas to fit the map
      window.NAV2D.canvas.scaleToDimensions(
        client.currentGrid.width,
        client.currentGrid.height,
      );
      window.NAV2D.canvas.shift(
        client.currentGrid.pose.position.x,
        client.currentGrid.pose.position.y,
      );

      // Re-draw points on map update
      const currentScene = getScene();
      if (!currentScene) return;

      window.NAV2D.pointsArray = drawPoints(
        window.NAV2D.pointsFromTopic,
        currentScene,
      );
      updateNavigationOverlayScale(currentScene);
      drawRobotTrail(currentScene);
      bringNavigationOverlaysToFront(currentScene);
      applyLayerState();
    });
  }

  // Costmap grids are large and expensive through rosbridge, so do not stream
  // them by default. RViz remains the better live costmap inspection tool.

  // Define republishWaypoints function
  window.NAV2D.republishWaypoints = (rosObject) => {
    const uiOperation = new window.ROSLIB.Topic({
      ros: rosObject,
      name: TOPICS.uiOperation,
      messageType: "std_msgs/String",
    });

    uiOperation.publish(new window.ROSLIB.Message({ data: "clear_route" }));

    setTimeout(() => {
      const wayPointTopic = new window.ROSLIB.Topic({
        ros: rosObject,
        name: TOPICS.newWaypoint,
        messageType: "geometry_msgs/PoseWithCovarianceStamped",
      });

      window.NAV2D.pointsFromTopic.forEach((point) => {
        const messageObject = {
          header: { frame_id: "map" },
          pose: {
            pose: {
              position: {
                x: point.pose.pose.position.x,
                y: point.pose.pose.position.y,
                z: 0.0,
              },
              orientation: {
                z: point.pose.pose.orientation.z,
                w: point.pose.pose.orientation.w,
              },
            },
            covariance: point.pose.covariance || new Array(36).fill(0.0),
          },
        };

        wayPointTopic.publish(new window.ROSLIB.Message(messageObject));
      });
    }, 50);
  };

  // Run navigator only once per init cycle
  if (!window.NAV2D.mapInited) {
    window.NAV2D.mapInited = true;
    navigator(ros);
  }

  // Re-establish the costmap stream if it was left enabled across a remount.
  if (window.NAV2D.layerState?.costmap) {
    ensureCostmapSubscription();
  }
};

// Cleaning map
window.NAV2D.ClearMap = () => {
  const scene = getScene();
  if (!scene) {
    // If we have no scene yet, just reset arrays safely.
    window.NAV2D.pointsArray = [];
    return;
  }

  window.NAV2D.pointsArray.forEach((marker) => {
    try {
      scene.removeChild(marker);
    } catch (e) {
      // ignore individual marker removal issues
    }
  });

  window.NAV2D.pointsArray = [];
};

// ------------------------------------------------------------
// Internal functions
// ------------------------------------------------------------
const drawPoints = (points, canvas) => {
  if (!Array.isArray(points) || !canvas) return [];

  window.NAV2D.ClearMap();

  window.NAV2D.pointsArray = points.map((point) => {
    const defaultPointItem = serializePoint(point, canvas);
    defaultPointItem.visible = window.NAV2D.layerState?.waypoints !== false;
    canvas.addChild(defaultPointItem);
    return defaultPointItem;
  });

  return window.NAV2D.pointsArray;
};

const scaleMarkerToScene = (marker, scene) => {
  if (!marker || !scene) return;
  marker.scaleX = 1.0 / scene.scaleX;
  marker.scaleY = 1.0 / scene.scaleY;
};

const updateNavigationOverlayScale = (scene = getScene()) => {
  scaleMarkerToScene(window.NAV2D.goalMarkerItem, scene);
};

const applyLayerState = () => {
  const state = window.NAV2D.layerState || {};
  const opacity = window.NAV2D.layerOpacity || {};

  if (window.NAV2D.mapClient?.currentGrid) {
    window.NAV2D.mapClient.currentGrid.visible = state.map !== false;
  }
  if (window.NAV2D.costmapItem) {
    window.NAV2D.costmapItem.visible = state.costmap !== false;
    window.NAV2D.costmapItem.alpha = opacity.costmap ?? 0.35;
  }
  if (window.NAV2D.scanShape) {
    window.NAV2D.scanShape.visible = state.scan !== false;
  }
  if (window.NAV2D.pathShape) {
    window.NAV2D.pathShape.visible = state.path !== false;
    window.NAV2D.pathShape.alpha = opacity.path ?? 0.95;
  }
  if (window.NAV2D.goalMarkerItem) {
    window.NAV2D.goalMarkerItem.visible = state.goal !== false;
  }
  if (window.NAV2D.robotTrailShape) {
    window.NAV2D.robotTrailShape.visible = state.robotTrail !== false;
    window.NAV2D.robotTrailShape.alpha = opacity.robotTrail ?? 0.65;
  }
  (window.NAV2D.pointsArray || []).forEach((marker) => {
    marker.visible = state.waypoints !== false;
  });
  (window.NAV2D.queuedWaypointItems || []).forEach((marker) => {
    marker.visible = state.waypoints !== false;
  });
};

// Costmap grids are large and expensive through rosbridge, so only subscribe
// while the layer is actually toggled on; tear down when it's hidden again.
const ensureCostmapSubscription = () => {
  if (window.NAV2D.costmapTopic) return;
  const ros = window.NAV2D.ros;
  if (!ros) return;

  window.NAV2D.costmapTopic = createSubscribeTopic(
    ros,
    TOPICS.globalCostmap,
    "nav_msgs/OccupancyGrid",
    (message) => {
      const scene = getScene();
      if (!scene) return;

      const mapGrid = window.NAV2D.mapClient?.currentGrid;
      const insertIndex = window.NAV2D.costmapItem
        ? scene.getChildIndex(window.NAV2D.costmapItem)
        : mapGrid
          ? scene.getChildIndex(mapGrid) + 1
          : 0;

      if (window.NAV2D.costmapItem) {
        try {
          scene.removeChild(window.NAV2D.costmapItem);
        } catch (e) {
          // ignore
        }
      }

      const grid = new window.ROS2D.OccupancyGrid({ message });
      grid.alpha = window.NAV2D.layerOpacity?.costmap ?? 0.35;
      grid.visible = window.NAV2D.layerState?.costmap !== false;
      scene.addChildAt(grid, Math.max(insertIndex, 0));

      window.NAV2D.costmapItem = grid;
      bringNavigationOverlaysToFront(scene);
    },
  );
};

const teardownCostmapSubscription = () => {
  if (window.NAV2D.costmapTopic) {
    try {
      window.NAV2D.costmapTopic.unsubscribe();
    } catch (e) {
      // ignore
    }
    window.NAV2D.costmapTopic = null;
  }

  const scene = getScene();
  if (scene && window.NAV2D.costmapItem) {
    try {
      scene.removeChild(window.NAV2D.costmapItem);
    } catch (e) {
      // ignore
    }
  }
  window.NAV2D.costmapItem = null;
};

window.NAV2D.setLayerVisible = (layer, visible) => {
  window.NAV2D.layerState = {
    ...window.NAV2D.layerState,
    [layer]: visible,
  };

  if (layer === "costmap") {
    if (visible) ensureCostmapSubscription();
    else teardownCostmapSubscription();
  }

  applyLayerState();
};

// Renders the client-side (not-yet-executed) waypoint queue as persistent
// markers, distinct from the single in-flight goal marker and the backend's
// own /WayPoints_topic markers.
window.NAV2D.setQueuedWaypoints = (poses) => {
  const scene = getScene();
  if (!scene) return;

  (window.NAV2D.queuedWaypointItems || []).forEach((marker) => {
    try {
      scene.removeChild(marker);
    } catch (e) {
      // ignore
    }
  });

  window.NAV2D.queuedWaypointItems = (poses || []).map((pose) => {
    const marker = createCanvasPoint(16, { r: 8, g: 126, b: 164, a: 1 });
    marker.x = pose.position.x;
    marker.y = -pose.position.y;
    marker.rotation = scene.rosQuaternionToGlobalTheta(pose.orientation);
    scaleMarkerToScene(marker, scene);
    marker.visible = window.NAV2D.layerState?.waypoints !== false;
    scene.addChild(marker);
    return marker;
  });

  bringNavigationOverlaysToFront(scene);
};

window.NAV2D.clearQueuedWaypoints = () => {
  window.NAV2D.setQueuedWaypoints([]);
};

window.NAV2D.setLayerOpacity = (layer, opacity) => {
  window.NAV2D.layerOpacity = {
    ...window.NAV2D.layerOpacity,
    [layer]: Number(opacity),
  };
  applyLayerState();
};

const ensureGoalMarker = (scene = getScene()) => {
  if (!scene || !window.NAV2D.latestGoalPose) return;
  if (
    window.NAV2D.goalMarkerItem &&
    window.NAV2D.goalMarkerItem.parent === scene
  ) {
    scaleMarkerToScene(window.NAV2D.goalMarkerItem, scene);
    return;
  }

  const marker = createCanvasPoint(18, { r: 217, g: 119, b: 6, a: 1 });
  marker.x = window.NAV2D.latestGoalPose.position.x;
  marker.y = -window.NAV2D.latestGoalPose.position.y;
  marker.rotation = scene.rosQuaternionToGlobalTheta(
    window.NAV2D.latestGoalPose.orientation,
  );
  scaleMarkerToScene(marker, scene);

  window.NAV2D.goalMarkerItem = marker;
  scene.addChild(marker);
  applyLayerState();
};

const bringNavigationOverlaysToFront = (scene = getScene()) => {
  if (!scene) return;
  ensureGoalMarker(scene);
  if (window.NAV2D.pathShape) scene.addChild(window.NAV2D.pathShape);
  if (window.NAV2D.goalMarkerItem) scene.addChild(window.NAV2D.goalMarkerItem);
};

window.NAV2D.setGoalPose = (pose) => {
  const scene = getScene();
  if (!scene || !pose?.position || !pose?.orientation) return;

  if (window.NAV2D.goalMarkerItem) {
    try {
      scene.removeChild(window.NAV2D.goalMarkerItem);
    } catch (e) {
      // ignore
    }
  }

  const marker = createCanvasPoint(18, { r: 217, g: 119, b: 6, a: 1 });
  marker.x = pose.position.x;
  marker.y = -pose.position.y;
  marker.rotation = scene.rosQuaternionToGlobalTheta(pose.orientation);
  scaleMarkerToScene(marker, scene);

  window.NAV2D.latestGoalPose = pose;
  window.NAV2D.goalMarkerItem = marker;
  scene.addChild(marker);
  applyLayerState();
};

window.NAV2D.clearGoalPose = () => {
  const scene = getScene();
  if (scene && window.NAV2D.goalMarkerItem) {
    try {
      scene.removeChild(window.NAV2D.goalMarkerItem);
    } catch (e) {
      // ignore
    }
  }
  window.NAV2D.goalMarkerItem = null;
  window.NAV2D.latestGoalPose = null;
};

window.NAV2D.clearPath = () => {
  if (window.NAV2D.pathShape?.graphics) {
    window.NAV2D.pathShape.graphics.clear();
  }
};

const drawRobotTrail = (scene = getScene()) => {
  if (!scene || !window.NAV2D.robotTrailShape) return;
  const points = window.NAV2D.robotTrail || [];
  const g = window.NAV2D.robotTrailShape.graphics;
  g.clear();
  if (points.length < 2) return;

  const lineWidth = Math.max(
    0.02,
    Math.min(0.05, 0.5 / Math.abs(scene.scaleX || 1)),
  );
  g.setStrokeStyle(lineWidth, "round", "round");
  g.beginStroke("rgba(8, 126, 164, 0.95)");
  drawSmoothLine(g, points, 0.8);
  g.endStroke();
  applyLayerState();
};

const toPlanPoints = (poses) => {
  const points = [];
  poses.forEach((poseStamped) => {
    const pos = poseStamped?.pose?.position;
    if (!pos) return;
    const point = { x: Number(pos.x), y: Number(pos.y) };
    if (!Number.isFinite(point.x) || !Number.isFinite(point.y)) return;
    const last = points[points.length - 1];
    if (!last || Math.hypot(point.x - last.x, point.y - last.y) > 0.04) {
      points.push(point);
    }
  });
  return points;
};

const smoothVisualPoints = (points) => {
  if (!Array.isArray(points) || points.length < 4) return points || [];
  return points.map((point, index) => {
    if (index === 0 || index === points.length - 1) return point;
    const prev = points[index - 1];
    const next = points[index + 1];
    return {
      x: prev.x * 0.2 + point.x * 0.6 + next.x * 0.2,
      y: prev.y * 0.2 + point.y * 0.6 + next.y * 0.2,
    };
  });
};

const drawSmoothLine = (graphics, rawPoints, breakDistance = Infinity) => {
  const points = smoothVisualPoints(rawPoints);
  if (points.length < 2) return;

  let segment = [];
  const flush = () => {
    if (segment.length < 2) {
      segment = [];
      return;
    }
    graphics.moveTo(segment[0].x, -segment[0].y);
    if (segment.length === 2) {
      graphics.lineTo(segment[1].x, -segment[1].y);
    } else {
      for (let i = 1; i < segment.length - 1; i += 1) {
        const current = segment[i];
        const next = segment[i + 1];
        const mid = {
          x: (current.x + next.x) / 2,
          y: (current.y + next.y) / 2,
        };
        graphics.quadraticCurveTo(current.x, -current.y, mid.x, -mid.y);
      }
      const last = segment[segment.length - 1];
      graphics.lineTo(last.x, -last.y);
    }
    segment = [];
  };

  points.forEach((point) => {
    const last = segment[segment.length - 1];
    if (last && Math.hypot(point.x - last.x, point.y - last.y) > breakDistance) {
      flush();
    }
    segment.push(point);
  });
  flush();
};

const drawPath = (pathMsg, scene = getScene()) => {
  if (!scene) return;
  if (!window.NAV2D.pathShape || window.NAV2D.pathShape.parent !== scene) {
    window.NAV2D.pathShape = new window.createjs.Shape();
    scene.addChild(window.NAV2D.pathShape);
  }

  const poses = pathMsg?.poses || [];
  const g = window.NAV2D.pathShape.graphics;
  g.clear();
  if (!Array.isArray(poses) || poses.length < 2) return;
  const points = toPlanPoints(poses);
  if (points.length < 2) return;

  const lineWidth = Math.max(
    0.035,
    Math.min(0.08, 0.8 / Math.abs(scene.scaleX || 1)),
  );

  g.setStrokeStyle(lineWidth * 2.6, "round", "round");
  g.beginStroke("rgba(255, 255, 255, 0.82)");
  drawSmoothLine(g, points);
  g.endStroke();

  g.setStrokeStyle(lineWidth, "round", "round");
  g.beginStroke(
    `rgba(37, 99, 235, ${window.NAV2D.layerOpacity?.path ?? 0.95})`,
  );
  drawSmoothLine(g, points);
  g.endStroke();

  bringNavigationOverlaysToFront(scene);
  applyLayerState();
};

const navigator = (ros) => {
  const canvas = getScene();
  if (!canvas) {
    console.warn("navigator() called before NAV2D.canvas.scene is ready");
    return;
  }

  // Send message about map initialization
  const messageTopic = new window.ROSLIB.Topic({
    ros,
    name: TOPICS.uiMessage,
    messageType: "std_msgs/String",
  });

  // Receiving array of points
  createSubscribeTopic(
    ros,
    TOPICS.waypoints,
    "openamr_ui_msgs/ArrayPoseStampedWithCovariance",
    (data) => {
      if (!data || !Array.isArray(data.poses)) {
        console.error("WayPoints_topic data.poses is not an array");
        return;
      }

      window.NAV2D.pointsFromTopic = data.poses;

      const scene = getScene();
      if (!scene) return;

      window.NAV2D.pointsArray = drawPoints(data.poses, scene);
      applyLayerState();
    },
  );

  // ROBOT MARKER SECTION:
  const robotMarker = createCanvasPoint(25, { r: 0, g: 0, b: 255, a: 1 });
  robotMarker.visible = false;
  window.NAV2D.robotMarker = robotMarker;
  canvas.addChild(robotMarker);

  if (
    !window.NAV2D.robotTrailShape ||
    window.NAV2D.robotTrailShape.parent !== canvas
  ) {
    window.NAV2D.robotTrailShape = new window.createjs.Shape();
    canvas.addChild(window.NAV2D.robotTrailShape);
  }

  if (!window.NAV2D.pathShape || window.NAV2D.pathShape.parent !== canvas) {
    window.NAV2D.pathShape = new window.createjs.Shape();
    canvas.addChild(window.NAV2D.pathShape);
  }

  createSubscribeTopic(ros, TOPICS.plan, "nav_msgs/Path", (pathMsg) => {
    drawPath(pathMsg, getScene());
  });

  const tfGraph = new Map();

  const addTfEdge = (from, to, tf) => {
    if (!tfGraph.has(from)) tfGraph.set(from, new Map());
    tfGraph.get(from).set(to, tf);
  };

  const lookupTransform = (targetFrame, sourceFrame) => {
    const target = normalizeFrame(targetFrame);
    const source = normalizeFrame(sourceFrame);
    if (!target || !source) return null;
    if (target === source)
      return { x: 0, y: 0, yaw: 0, orientation: { x: 0, y: 0, z: 0, w: 1 } };

    const queue = [
      {
        frame: source,
        tf: { x: 0, y: 0, yaw: 0, orientation: { x: 0, y: 0, z: 0, w: 1 } },
      },
    ];
    const visited = new Set([source]);

    while (queue.length) {
      const current = queue.shift();
      const edges = tfGraph.get(current.frame) || new Map();
      for (const [nextFrame, edgeTf] of edges) {
        if (visited.has(nextFrame)) continue;
        const tf = composeTransforms2D(current.tf, edgeTf);
        if (nextFrame === target) return tf;
        visited.add(nextFrame);
        queue.push({ frame: nextFrame, tf });
      }
    }

    return null;
  };

  const applyRobotPose = (pose) => {
    const scene = getScene();
    if (!scene || !pose?.position || !pose?.orientation) return;

    robotMarker.x = pose.position.x;
    robotMarker.y = -pose.position.y;
    robotMarker.scaleX = 1.0 / scene.scaleX;
    robotMarker.scaleY = 1.0 / scene.scaleY;
    robotMarker.rotation = scene.rosQuaternionToGlobalTheta(pose.orientation);
    robotMarker.visible = true;
    window.NAV2D.currentPose = pose;

    const last = window.NAV2D.robotTrail[window.NAV2D.robotTrail.length - 1];
    const next = { x: pose.position.x, y: pose.position.y };
    if (!last || Math.hypot(next.x - last.x, next.y - last.y) > 0.05) {
      window.NAV2D.robotTrail.push(next);
      if (window.NAV2D.robotTrail.length > 350) window.NAV2D.robotTrail.shift();
      drawRobotTrail(scene);
    }
  };

  const updateRobotFromTf = () => {
    const baseToMap = lookupTransform("map", "base_link");
    if (!baseToMap) return false;
    applyRobotPose({
      position: { x: baseToMap.x, y: baseToMap.y, z: 0 },
      orientation: baseToMap.orientation,
    });
    return true;
  };

  const updateTfMessage = (data) => {
    if (!Array.isArray(data?.transforms)) return;
    data.transforms.forEach((transformStamped) => {
      const parent = normalizeFrame(transformStamped?.header?.frame_id);
      const child = normalizeFrame(transformStamped?.child_frame_id);
      if (!parent || !child || !transformStamped?.transform) return;

      const tf = toTransform2D(
        transformStamped.transform.translation,
        transformStamped.transform.rotation,
      );
      addTfEdge(child, parent, tf);
      addTfEdge(parent, child, invertTransform2D(tf));
    });
    updateRobotFromTf();
  };

  // Keep TF throttled: raw /tf through rosbridge can disturb sim timing.
  createSubscribeTopic(ros, TOPICS.tf, "tf2_msgs/TFMessage", updateTfMessage);
  createSubscribeTopic(
    ros,
    TOPICS.tfStatic,
    "tf2_msgs/TFMessage",
    updateTfMessage,
  );

  // AMCL fallback when the TF chain is not available yet.
  createSubscribeTopic(
    ros,
    TOPICS.amclPose,
    "geometry_msgs/PoseWithCovarianceStamped",
    (data) => {
      if (!canvas || !data?.pose?.pose) return;
      if (!updateRobotFromTf()) applyRobotPose(data.pose.pose);
    },
  );

  // LiDAR scan shape
  const scanShape = new window.createjs.Shape();
  window.NAV2D.scanShape = scanShape;
  canvas.addChild(scanShape);

  // Unsubscribe old scan subscription if any
  if (window.NAV2D.scanTopic) {
    try {
      window.NAV2D.scanTopic.unsubscribe();
    } catch (e) {}
  }

  const scanTopic = createSubscribeTopic(
    ros,
    TOPICS.scan,
    "sensor_msgs/LaserScan",
    (message) => {
      if (!message || !Array.isArray(message.ranges)) return;

      const frameId = message.header?.frame_id || "laser_link";
      const laserToMap = lookupTransform("map", frameId);
      if (!laserToMap) return; // Wait until TF transform is available

      const g = scanShape.graphics;
      g.clear();
      g.beginFill(
        `rgba(8, 126, 164, ${window.NAV2D.layerOpacity?.scan ?? 0.32})`,
      );

      const angleMin = message.angle_min;
      const angleIncrement = message.angle_increment;
      const ranges = message.ranges;
      const rangeMin = message.range_min || 0.0;
      const rangeMax = message.range_max || 100.0;

      const cosYaw = Math.cos(laserToMap.yaw);
      const sinYaw = Math.sin(laserToMap.yaw);

      const scanScale = Math.max(Math.abs(canvas.scaleX || 1), 1);
      const dotRadius = Math.max(0.015, Math.min(0.045, 0.35 / scanScale));
      const sampleStep = Math.max(1, Math.ceil(ranges.length / 180));

      for (let i = 0; i < ranges.length; i += sampleStep) {
        const r = ranges[i];
        if (isNaN(r) || r < rangeMin || r > rangeMax) continue;

        const angle = angleMin + i * angleIncrement;
        const xs = r * Math.cos(angle);
        const ys = r * Math.sin(angle);

        // Transform to map frame
        const xm = laserToMap.x + xs * cosYaw - ys * sinYaw;
        const ym = laserToMap.y + xs * sinYaw + ys * cosYaw;

        // Draw in map frame (with inverted y)
        g.drawCircle(xm, -ym, dotRadius);
      }
    },
  );
  window.NAV2D.scanTopic = scanTopic;

  // MOUSE EVENT SECTION
  let isMousePressed = false;
  let isMouseMoved = false;
  let positionVectorItem = null;
  let orientationMarker = null;

  const handleMouseDown = (event) => {
    isMousePressed = true;
    const positionItem = canvas.globalToRos(event.stageX, event.stageY);
    positionVectorItem = new window.ROSLIB.Vector3(positionItem);
  };

  const handleMouseMove = (event) => {
    if (!isMousePressed) return;
    isMouseMoved = true;

    if (orientationMarker) {
      try {
        canvas.removeChild(orientationMarker);
      } catch (e) {
        // ignore
      }
    }

    const currentPos = canvas.globalToRos(event.stageX, event.stageY);
    const currentPositionVectorItem = new window.ROSLIB.Vector3(currentPos);

    orientationMarker = createCanvasPoint(25, { r: 0, g: 255, b: 0, a: 1 });

    const xDelta = currentPositionVectorItem.x - positionVectorItem.x;
    const yDelta = currentPositionVectorItem.y - positionVectorItem.y;
    const thetaRadians = Math.atan2(xDelta, yDelta);
    let thetaDegrees = thetaRadians * (180.0 / Math.PI);

    if (thetaDegrees >= 0 && thetaDegrees <= 180) {
      thetaDegrees += 270;
    } else {
      thetaDegrees -= 90;
    }

    orientationMarker.x = positionVectorItem.x;
    orientationMarker.y = -positionVectorItem.y;
    orientationMarker.rotation = thetaDegrees;
    orientationMarker.scaleX = 1.0 / canvas.scaleX;
    orientationMarker.scaleY = 1.0 / canvas.scaleY;
    canvas.addChild(orientationMarker);
  };

  const handleMouseUp = (event) => {
    if (!isMousePressed) return;

    if (!isMouseMoved) {
      console.error("Please, set the direction of the WayPoint");
      messageTopic.publish(
        new window.ROSLIB.Message({
          data: "Please, set the direction of the WayPoint",
        }),
      );
      // Reset press state to avoid getting stuck
      isMousePressed = false;
      isMouseMoved = false;
      return;
    }

    isMousePressed = false;
    isMouseMoved = false;

    if (!orientationMarker) {
      console.warn("orientationMarker missing on mouse up");
      return;
    }

    const goalPos = canvas.globalToRos(event.stageX, event.stageY);
    const goalPosVec3 = new window.ROSLIB.Vector3(goalPos);
    const xDelta = goalPosVec3.x - positionVectorItem.x;
    const yDelta = goalPosVec3.y - positionVectorItem.y;
    const thetaRadians = calculateThetaRadians(xDelta, yDelta);

    const qz = Math.sin(-thetaRadians / 2.0);
    const qw = Math.cos(-thetaRadians / 2.0);

    const orientation = new window.ROSLIB.Quaternion({
      x: 0,
      y: 0,
      z: qz,
      w: qw,
    });

    const pose = new window.ROSLIB.Pose({
      position: positionVectorItem,
      orientation,
    });

    window.NAV2D.finishedPointItem = pose;
    window.NAV2D.setGoalPose(pose);

    // Notify ControlPage Goal/Pose mode callback
    if (typeof window.NAV2D._poseCallback === "function") {
      window.NAV2D._poseCallback(pose);
    }

    try {
      canvas.removeChild(orientationMarker);
    } catch (e) {
      // ignore
    }
    orientationMarker = null;
  };

  const handleCanvasEvent = (event, mouseEventType) => {
    if (!window.NAV2D.arePointsSettable) return;

    switch (mouseEventType) {
      case "down":
        handleMouseDown(event);
        break;
      case "move":
        handleMouseMove(event);
        break;
      case "up":
        handleMouseUp(event);
        break;
      default:
        break;
    }
  };

  const onCanvasMove = (event) => {
    handleCanvasEvent(event, "move");
  };

  canvas.addEventListener("stagemousedown", (event) => {
    handleCanvasEvent(event, "down");
    canvas.addEventListener("stagemousemove", onCanvasMove);
  });

  canvas.addEventListener("stagemouseup", (event) => {
    handleCanvasEvent(event, "up");
    canvas.removeEventListener("stagemousemove", onCanvasMove);
  });
};

const createSubscribeTopic = (ros, name, messageType, callback) => {
  const topicObject = { ros, name, messageType };

  if (name === "/odom") {
    topicObject.throttle_rate = 1;
  }

  if (name === TOPICS.tf || name === TOPICS.tfStatic || name === TOPICS.scan) {
    topicObject.throttle_rate = 1000;
    topicObject.queue_length = 1;
  }

  if (name === TOPICS.globalCostmap) {
    topicObject.throttle_rate = 3000;
    topicObject.queue_length = 1;
  }

  const topic = new window.ROSLIB.Topic(topicObject);
  topic.subscribe(callback);
  return topic;
};

const calculateThetaRadians = (xDelta, yDelta) => {
  let thetaRadians = Math.atan2(xDelta, yDelta);
  if (thetaRadians >= 0 && thetaRadians <= Math.PI) {
    thetaRadians += (3 * Math.PI) / 2;
  } else {
    thetaRadians -= Math.PI / 2;
  }
  return thetaRadians;
};

const createCanvasPoint = (size, color) => {
  return new window.ROS2D.NavigationArrow({
    size,
    strokeSize: 1,
    fillColor: window.createjs.Graphics.getRGB(
      color.r,
      color.g,
      color.b,
      color.a,
    ),
    pulse: false,
  });
};

window.NAV2D.sendPointToRobot = (ros, time) => {
  const pointDetails = window.NAV2D.finishedPointItem;
  if (!pointDetails) {
    console.warn("sendPointToRobot called but finishedPointItem is null");
    return;
  }

  const { hours, minutes } = time || {};

  const wayPoint = new window.ROSLIB.Topic({
    ros,
    name: TOPICS.newWaypoint,
    messageType: "geometry_msgs/PoseWithCovarianceStamped",
  });

  const sendDataArray = new Array(36).fill(0.0);

  // Keep behavior from original: always "navigate" type = 3
  sendDataArray[0] = 3;
  sendDataArray[1] = Number(hours) || 0;
  sendDataArray[2] = Number(minutes) || 0;

  const messageObject = {
    header: { frame_id: "map" },
    pose: {
      pose: {
        position: {
          x: pointDetails.position.x,
          y: pointDetails.position.y,
          z: 0.0,
        },
        orientation: {
          z: pointDetails.orientation.z,
          w: pointDetails.orientation.w,
        },
      },
      covariance: sendDataArray,
    },
  };

  const poseMessage = new window.ROSLIB.Message(messageObject);
  wayPoint.publish(poseMessage);

  // Reset after publish
  window.NAV2D.finishedPointItem = null;
};

const serializePoint = (point, canvas) => {
  const defaultPointItem = createCanvasPoint(15, {
    r: 255,
    g: 0,
    b: 0,
    a: 1,
  });

  defaultPointItem.x = point.pose.pose.position.x;
  defaultPointItem.y = -point.pose.pose.position.y;
  defaultPointItem.rotation = canvas.rosQuaternionToGlobalTheta(
    point.pose.pose.orientation,
  );
  defaultPointItem.scaleX = 1.0 / canvas.scaleX;
  defaultPointItem.scaleY = 1.0 / canvas.scaleY;

  // Drag and drop events for interactive waypoints
  let dragOffset = { x: 0, y: 0 };

  defaultPointItem.addEventListener("mousedown", (event) => {
    if (!window.NAV2D.arePointsSettable) return;
    const mousePos = canvas.globalToRos(event.stageX, event.stageY);
    dragOffset.x = mousePos.x - defaultPointItem.x;
    dragOffset.y = -mousePos.y - defaultPointItem.y;
  });

  defaultPointItem.addEventListener("pressmove", (event) => {
    if (!window.NAV2D.arePointsSettable) return;
    const mousePos = canvas.globalToRos(event.stageX, event.stageY);
    defaultPointItem.x = mousePos.x - dragOffset.x;
    defaultPointItem.y = -mousePos.y - dragOffset.y;
  });

  defaultPointItem.addEventListener("pressup", (event) => {
    if (!window.NAV2D.arePointsSettable) return;

    // Update the underlying pose position
    point.pose.pose.position.x = defaultPointItem.x;
    point.pose.pose.position.y = -defaultPointItem.y;

    // Republish current sequence of waypoints to sync backend
    if (
      window.NAV2D.ros &&
      typeof window.NAV2D.republishWaypoints === "function"
    ) {
      window.NAV2D.republishWaypoints(window.NAV2D.ros);
    }
  });

  return defaultPointItem;
};
