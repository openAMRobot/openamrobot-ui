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

window.NAV2D.mapInited = false;
window.NAV2D.scale = { x: 0, y: 0 };

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------
const getScene = () => {
  // This file is loaded before Map sometimes, so NAV2D.canvas may be null.
  const nav2d = window.NAV2D;
  if (!nav2d || !nav2d.canvas || !nav2d.canvas.scene) return null;
  return nav2d.canvas.scene;
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
  }
};

// Poll quickly until scene exists, then it self-stops inside checkScale()
window.NAV2D._checkScaleIntervalId = setInterval(window.NAV2D.checkScale, 100);

// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------
window.NAV2D.InitMap = (ros) => {
  console.log("InitMap");

  const scene = getScene();
  if (!scene) {
    // Map.jsx should set NAV2D.canvas before calling InitMap,
    // but guard anyway to avoid a hard crash.
    console.warn("NAV2D.InitMap called before NAV2D.canvas.scene is ready");
    return;
  }

  const topic = "/map";

  // Setup a client to get the map
  const client = new window.ROS2D.OccupancyGridClient({
    ros,
    rootObject: scene,
    continuous: true,
    topic,
  });

  // Updating map after topic event
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
  });

  // Run navigator only once per init cycle
  if (!window.NAV2D.mapInited) {
    window.NAV2D.mapInited = true;
    navigator(ros);
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
    canvas.addChild(defaultPointItem);
    return defaultPointItem;
  });

  return window.NAV2D.pointsArray;
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
    name: "/ui_message",
    messageType: "std_msgs/String",
  });

  // Receiving array of points
  createSubscribeTopic(
    ros,
    "/WayPoints_topic",
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

      // This flag looks odd in your original code (sets false on update),
      // but keeping behavior unchanged.
      window.NAV2D.mapInited = false;
    },
  );

  // ROBOT MARKER SECTION:
  const robotMarker = createCanvasPoint(25, { r: 0, g: 0, b: 255, a: 1 });
  robotMarker.visible = false;
  canvas.addChild(robotMarker);

  // Robot position watcher
  createSubscribeTopic(ros, "/odom", "nav_msgs/Odometry", (data) => {
    const scene = getScene();
    if (!scene || !data?.pose?.pose) return;

    const pose = data.pose.pose;

    robotMarker.x = pose.position.x;
    robotMarker.y = -pose.position.y;
    robotMarker.scaleX = 1.0 / scene.scaleX;
    robotMarker.scaleY = 1.0 / scene.scaleY;
    robotMarker.rotation = scene.rosQuaternionToGlobalTheta(pose.orientation);
    robotMarker.visible = true;
  });

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

    // Default color (kept from original)
    const pointColor = { r: 255, g: 0, b: 0, a: 1 };

    const goalMarkerItem = createCanvasPoint(15, pointColor);

    if (!orientationMarker) {
      console.warn("orientationMarker missing on mouse up");
      return;
    }

    goalMarkerItem.x = orientationMarker.x;
    goalMarkerItem.y = orientationMarker.y;
    goalMarkerItem.rotation = orientationMarker.rotation;
    goalMarkerItem.scaleX = orientationMarker.scaleX;
    goalMarkerItem.scaleY = orientationMarker.scaleY;

    window.NAV2D.orientatedPointItem = goalMarkerItem;
    window.NAV2D.pointsArray.push(goalMarkerItem);
    canvas.addChild(goalMarkerItem);

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
    name: "/new_way_point",
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

  return defaultPointItem;
};
