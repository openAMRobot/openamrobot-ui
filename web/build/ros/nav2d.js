window.NAV2D = window.NAV2D || {};

window.NAV2D.pointsArray = [];
window.NAV2D.pointsFromTopic = [];
window.NAV2D.arePointsSettable = false;
window.NAV2D.canvas = null;
window.NAV2D.pointType = null;
window.NAV2D.finishedPointItem = null;
window.NAV2D.orientatedPointItem = null;

window.NAV2D.mapInited = false;
window.NAV2D.scale = { x: 0, y: 0 };

window.NAV2D.checkScale = () => {
  // Your code to be executed periodically
  if (window.NAV2D.scale.x != window.NAV2D.canvas.scene.scaleX || window.NAV2D.scale.y != window.NAV2D.canvas.scene.scaleY) 
  {
    window.NAV2D.scale.x = window.NAV2D.canvas.scene.scaleX;
    window.NAV2D.scale.y = window.NAV2D.canvas.scene.scaleY;
    window.NAV2D.pointsArray = drawPoints(
      window.NAV2D.pointsFromTopic,
      window.NAV2D.canvas.scene,
    );
  }
}

// Call the function every 1 second (1000 milliseconds)
const intervalId = setInterval(window.NAV2D.checkScale, 100);

console.log("hello ");

window.NAV2D.InitMap = (ros) => {
  console.log("InitMap");
  const topic = "/map";

  /* setup a client to get the map */
  const client = new window.ROS2D.OccupancyGridClient({
    ros,
    rootObject: window.NAV2D.canvas.scene,
    continuous: true,
    topic,
  });

  /* Updating map after topic event */
  client.on("change", () => {
    /* Scale the canvas to fit the map */
    window.NAV2D.canvas.scaleToDimensions(
      client.currentGrid.width,
      client.currentGrid.height,
    );
    window.NAV2D.canvas.shift(
      client.currentGrid.pose.position.x,
      client.currentGrid.pose.position.y,
    );

    window.NAV2D.pointsArray = drawPoints(
      window.NAV2D.pointsFromTopic,
      window.NAV2D.canvas.scene,
    );
  });

  if (!window.NAV2D.mapInited) {
    window.NAV2D.mapInited = true;
    navigator(ros);
  }
};

/* Cleaning map */
window.NAV2D.ClearMap = () => {
  window.NAV2D.pointsArray.forEach((marker) =>
    window.NAV2D.canvas.scene.removeChild(marker),
  );
  window.NAV2D.pointsArray = [];
};

const drawPoints = (points, canvas) => {
  if (!(points && canvas)) return;
  window.NAV2D.ClearMap();

  window.NAV2D.pointsArray = points.map((point) => {
    const defaultPointItem = serializePoint(point, canvas);
    canvas.addChild(defaultPointItem);
    return defaultPointItem;
  });
  return window.NAV2D.pointsArray;
};


const navigator = (ros) => {
  const canvas = window.NAV2D.canvas.scene;

  /* Send message about map initialization */
  const messageTopic = new window.ROSLIB.Topic({
    ros,
    name: "/ui_message",
    messageType: "std_msgs/String",
  });
  /*
  messageTopic.publish(
    new window.ROSLIB.Message({ data: "Initialization completed" }),
  );
 */

  /* Receiving array of points*/

  createSubscribeTopic(
    ros,
    "/WayPoints_topic",
    "openamr_ui_msgs/ArrayPoseStampedWithCovariance",
    (data) => {
      if (!Array.isArray(data.poses)) {
        console.error("WayPoints_topic data.poses is not an array");
        return;
      }
      window.NAV2D.pointsFromTopic = data.poses;
      window.NAV2D.pointsArray = drawPoints(data.poses, canvas);
      window.NAV2D.mapInited = false;
    },
  );

  /* ROBOT MARKER SECTION: */
  const robotMarker = createCanvasPoint(25, {
    r: 0,
    g: 0,
    b: 255,
    a: 1,
  });
  robotMarker.visible = false;
  canvas.addChild(robotMarker);

  /* Robot position watcher */
  createSubscribeTopic(ros, "/odom", "nav_msgs/Odometry", (data) => {
    const pose = data.pose.pose;
    robotMarker.x = pose.position.x;
    robotMarker.y = -pose.position.y;
    robotMarker.scaleX = 1.0 / canvas.scaleX;
    robotMarker.scaleY = 1.0 / canvas.scaleY;
    robotMarker.rotation = canvas.rosQuaternionToGlobalTheta(pose.orientation);
    robotMarker.visible = true;

    console.log("odom", window.NAV2D.canvas.scene.scaleX, window.NAV2D.canvas.scene.scaleY);
  });

  /* MOUSE EVENT SECTION */
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
    canvas.removeChild(orientationMarker);

    const currentPos = canvas.globalToRos(event.stageX, event.stageY);
    const currentPositionVectorItem = new window.ROSLIB.Vector3(currentPos);

    orientationMarker = createCanvasPoint(25, {
      r: 0,
      g: 255,
      b: 0,
      a: 1,
    });

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
      return;
    }

    isMousePressed = false;
    isMouseMoved = false;

    let pointColor = {};
    /* Logic for different point types */
    /*
    if (window.NAV2D.pointType === "navigate") {
      pointColor = {
        r: 255,
        g: 0,
        b: 0,
        a: 1,
      };
    } else if (window.NAV2D.pointType === "home") {
      pointColor = {
        r: 124,
        g: 252,
        b: 0,
        a: 1,
      };
    } else if (window.NAV2D.pointType === "charge") {
      pointColor = {
        r: 186,
        g: 85,
        b: 211,
        a: 1,
      };
    } else {
      console.error("Point type was not selected");
      messageTopic.publish(
        new window.ROSLIB.Message({ data: "Point type was not selected" }),
      );
      return;
    }
    */
    pointColor = {
      r: 255,
      g: 0,
      b: 0,
      a: 1,
    };
    const goalMarkerItem = createCanvasPoint(15, pointColor);

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
    let thetaRadians = calculateThetaRadians(xDelta, yDelta);

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
    canvas.removeChild(orientationMarker);
  };

  const handleCanvasEvent = (event, mouseEventType) => {
    if (!window.NAV2D.arePointsSettable) {
      return;
    }

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
  const topicObject = {
    ros,
    name,
    messageType,
  };

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
  const { hours, minutes } = time;

  const wayPoint = new window.ROSLIB.Topic({
    ros,
    name: "/new_way_point",
    messageType: "geometry_msgs/PoseWithCovarianceStamped",
  });

  const sendDataArray = [
    0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  ];
  /*
  if (window.NAV2D.pointType === "navigate") {
    sendDataArray[0] = 3;
  } else if (window.NAV2D.pointType === "home") {
    sendDataArray[0] = 1;
  } else if (window.NAV2D.pointType === "charge") {
    sendDataArray[0] = 2;
  }
  */
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
