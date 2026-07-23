import React, { useRef, useState, useEffect, useCallback } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { useRos } from "../app/App";
import { AppConfig } from "../shared/constants/index";
import TimeModal from "../components/modal/TimeModal";
import RouteModal from "../components/modal/RouteModal";
import TextInputModal from "../components/modal/TextInputModal";

import Map from "../components/Map";
import MapLayers from "../components/MapLayers";
import Button from "../shared/ui/Button";
import {
  DashboardCard,
  SectionHeader,
  StatusBadge,
} from "../shared/ui/Dashboard";

const removeCsv = (data) => {
  if (Array.isArray(data)) {
    return data.map((item) => {
      if (typeof item === "string") {
        return item.replace(".csv", "");
      }
      return removeCsv(item);
    });
  } else if (typeof data === "object" && data !== null) {
    const result = {};
    for (const key in data) {
      result[key] = removeCsv(data[key]);
    }
    return result;
  }
  return data;
};

function replaceUnderscoresInKeysAndValues(obj) {
  if (Array.isArray(obj)) {
    return obj.map((item) => replaceUnderscoresInKeysAndValues(item));
  } else if (typeof obj === "object" && obj !== null) {
    const newObj = {};
    for (const key in obj) {
      if (Object.prototype.hasOwnProperty.call(obj, key)) {
        const newKey = key.replace(/_/g, " ");
        newObj[newKey] = replaceUnderscoresInKeysAndValues(obj[key]);
      }
    }
    return newObj;
  } else if (typeof obj === "string") {
    return obj.replace(/_/g, " ");
  }
  return obj;
}

const processObjectStrings = (obj) => {
  if (typeof obj === "object" && obj !== null) {
    const result = {};
    for (const key in obj) {
      if (Object.prototype.hasOwnProperty.call(obj, key)) {
        result[key] = processObjectStrings(obj[key]);
      }
    }
    return result;
  } else if (typeof obj === "string") {
    return obj.trim().replace(/\s/g, "_");
  }
  return obj;
};

const findMapArray = (structure, activeFiles) => {
  const groupObject = structure.find((floor) =>
    Object.prototype.hasOwnProperty.call(floor, activeFiles.group),
  );

  if (groupObject) {
    const mapsArray = groupObject[activeFiles.group];

    const mapObject = mapsArray.find((room) =>
      Object.prototype.hasOwnProperty.call(room, activeFiles.map),
    );

    if (mapObject) {
      return mapObject[activeFiles.map];
    }
  }

  return [];
};

const downsamplePath = (poses, interval = 1.0) => {
  if (poses.length === 0) return [];
  const downsampled = [poses[0]];
  let lastPos = poses[0].pose.position;

  for (let i = 1; i < poses.length; i++) {
    const currentPos = poses[i].pose.position;
    const dx = currentPos.x - lastPos.x;
    const dy = currentPos.y - lastPos.y;
    const dist = Math.sqrt(dx * dx + dy * dy);

    if (dist >= interval) {
      downsampled.push(poses[i]);
      lastPos = currentPos;
    }
  }

  if (
    downsampled.length > 0 &&
    downsampled[downsampled.length - 1] !== poses[poses.length - 1]
  ) {
    downsampled.push(poses[poses.length - 1]);
  }

  return downsampled;
};

const removePointFromCanvas = () => {
  if (window.NAV2D?.clearGoalPose) {
    window.NAV2D.clearGoalPose();
    window.NAV2D.finishedPointItem = null;
    return;
  }

  const markerOnMap = window.NAV2D.orientatedPointItem;
  window.NAV2D.pointsArray = window.NAV2D.pointsArray.filter(
    (marker) => marker !== markerOnMap,
  );
  window.NAV2D.canvas.scene.removeChild(markerOnMap);
  window.NAV2D.finishedPointItem = null;
  window.NAV2D.orientatedPointItem = null;
};

const RoutePage = () => {
  const ros = useRos();
  // eslint-disable-next-line no-unused-vars
  const [selectedPointType, setSelectedPointType] = useState(null);
  const [pointsSettable, setPointsSettable] = useState(false);
  // eslint-disable-next-line no-unused-vars
  const [hoursValue, setHoursValue] = useState("0");
  const latestHoursValue = useRef(hoursValue);
  // eslint-disable-next-line no-unused-vars
  const [minutesValue, setMinutesValue] = useState("0");
  const latestMinutesValue = useRef(minutesValue);

  useEffect(() => {
    latestHoursValue.current = hoursValue;
    latestMinutesValue.current = minutesValue;
  }, [hoursValue, minutesValue]);

  const [selectedFile, setSelectedFile] = useState({
    group: "",
    map: "",
    route: "",
  });
  const [openRouteModal, setOpenRouteModal] = useState(false);
  const [openTimeModal, setOpenTimeModal] = useState(false);
  const [openInputModal, setOpenInputModal] = useState(false);

  const [filesData, setFilesData] = useState([]);
  // Unfiltered group->maps->routes catalog (filesData above is narrowed to
  // just the routes on the *current* map) — kept so "Change map" can offer
  // every map across every group, not only the active one.
  const [allStructure, setAllStructure] = useState([]);

  const childRef = useRef(null);
  const routesModalType = useRef(null);
  const isRoutesModalWithInput = useRef(null);
  const routesModalHeader = useRef(null);
  const modalKey = useRef(null);

  const textInputHeader = useRef(null);
  const textInputPlaceholder = useRef(null);

  /* TOPICS */

  const filesReqTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.NAV_DATA_REQ_TOPIC,
      messageType: "std_msgs/Empty",
    }),
  );

  const filesResonseTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.NAV_DATA_RESP_TOPIC,
      messageType: "std_msgs/String",
    }),
  );

  const uiOperationTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.UI_OPERATION_TOPIC,
      messageType: "std_msgs/String",
    }),
  );

  const onMapClickHandler = useCallback(() => {
    if (!window.NAV2D.arePointsSettable) return;

    // Open the time configuration modal upon placing a waypoint
    setOpenTimeModal(true);
  }, []);

  useEffect(() => {
    window.NAV2D.ClearMap();

    const currentFilesResponseTopic = filesResonseTopic.current;
    const currentMap = childRef.current;

    currentFilesResponseTopic.subscribe((data) => {
      const response = data.data;
      const responseObject = JSON.parse(response);

      const serializedArray = removeCsv(responseObject.structure);
      const arrayWithSpaces =
        replaceUnderscoresInKeysAndValues(serializedArray);

      // const serializedActiveFile = {
      //   group: responseObject.active_files.group,
      //   map: responseObject.active_files.map,
      //   route: responseObject.active_files.route.replace(".csv", ""),
      // };

      const activeFilesWithSpaces = replaceUnderscoresInKeysAndValues({
        group: responseObject.active_files.group,
        map: responseObject.active_files.map,
        route: responseObject.active_files.route.replace(".csv", ""),
      });

      const filtredArrayBySelectedRoute = findMapArray(
        arrayWithSpaces,
        activeFilesWithSpaces,
      );

      setFilesData(filtredArrayBySelectedRoute);
      setAllStructure(arrayWithSpaces);
      setSelectedFile(activeFilesWithSpaces);
    });

    filesReqTopic.current.publish();

    return () => {
      currentFilesResponseTopic.unsubscribe();
      window.NAV2D.arePointsSettable = false;

      const mapElement = currentMap ? currentMap.getMapRef() : null;
      if (mapElement) {
        mapElement.removeEventListener("mouseup", onMapClickHandler);
      }
    };
  }, [onMapClickHandler]);

  const onOperationTopicPublish = (message) => {
    uiOperationTopic.current.publish(
      new window.ROSLIB.Message({ data: message }),
    );
  };

  /* FROM HANDLERS */

  const onRouteFormSubmitHandler = (data) => {
    setOpenRouteModal(false);

    if (data) {
      // CHANGE_MAP's modal list is "group / map" combined strings (see
      // mapOptions below) since a map lives one level up from routes —
      // split it back apart here rather than teaching RouteModal about
      // two-level selection.
      const [selectedGroup, selectedMap] =
        modalKey.current === "CHANGE_MAP" ? data.split(" / ") : [];

      const operationsConfig = {
        CHANGE_ROUTE: {
          path: "change_route",
          data: {
            group: selectedFile.group,
            map: selectedFile.map,
            route: data,
          },
          preActions: () => window.NAV2D.ClearMap(),
          postActions: () =>
            setSelectedFile({
              group: selectedFile.group,
              map: selectedFile.map,
              route: data,
            }),
        },
        CHANGE_MAP: {
          path: "change_map",
          data: { group: selectedGroup, map: selectedMap },
          preActions: () => window.NAV2D.ClearMap(),
          postActions: () => {
            setSelectedFile({ group: selectedGroup, map: selectedMap, route: "Null" });
            toast.info(
              "Map loaded — set the robot's initial pose before navigating; its old localization no longer matches the new map.",
            );
          },
        },
      };

      const currentOperation = operationsConfig[modalKey.current];
      if (currentOperation) {
        currentOperation.preActions && currentOperation.preActions();
        const objToSendWithoutSpaces = processObjectStrings(
          currentOperation.data,
        );

        const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);
        const messageToSend = `${currentOperation.path}/${stringifiedObjToSend}`;
        onOperationTopicPublish(messageToSend);
        currentOperation.postActions && currentOperation.postActions();
      }
    }

    modalKey.current = null;
    routesModalType.current = null;
    isRoutesModalWithInput.current = null;
    routesModalHeader.current = null;
  };

  // TODELETE
  const onTimeFormSubmitHandler = (data) => {
    setOpenTimeModal(false);

    if (data) {
      window.NAV2D.sendPointToRobot(ros, data);
    } else {
      const markerOnMap = window.NAV2D.orientatedPointItem;
      window.NAV2D.pointsArray = window.NAV2D.pointsArray.filter(
        (marker) => marker !== markerOnMap,
      );
      window.NAV2D.canvas.scene.removeChild(markerOnMap);
      window.NAV2D.finishedPointItem = null;
      window.NAV2D.orientatedPointItem = null;
    }
  };

  const onInputFormSubmitHandler = (data) => {
    setOpenInputModal(false);

    if (data) {
      const operationsConfig = {
        SAVE_ROUTE: {
          path: "save_route",
          data: {
            group: selectedFile.group,
            map: selectedFile.map,
            route: data,
          },

          postActions: () => {
            const mapElement = childRef.current.getMapRef();

            if (mapElement) {
              mapElement.removeEventListener("mouseup", onMapClickHandler);
            }

            setPointsSettable(false);
            setSelectedFile((prev) => ({ ...prev, route: data }));
            setSelectedPointType(null);
            window.NAV2D.pointType = null;
            window.NAV2D.arePointsSettable = false;
          },
        },

        RENAME_ROUTE: {
          path: "rename_route",
          data: {
            group: selectedFile.group,
            map: selectedFile.map,
            route_old: selectedFile.route,
            route_new: data,
          },
          postActions: () => {
            const newFileData = {
              group: selectedFile.group,
              map: selectedFile.map,
              route: data,
            };
            setSelectedFile(newFileData);
          },
        },
      };

      const currentOperation = operationsConfig[modalKey.current];

      if (currentOperation) {
        currentOperation.preActions && currentOperation.preActions();

        const objToSendWithoutSpaces = processObjectStrings(
          currentOperation.data,
        );
        const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);

        const messageToSend = `${currentOperation.path}/${stringifiedObjToSend}`;
        onOperationTopicPublish(messageToSend);
        currentOperation.postActions && currentOperation.postActions();
      }
    }

    modalKey.current = null;
    textInputHeader.current = null;
    textInputPlaceholder.current = null;
  };

  /* BUTTON HANDLERS */

  const onNewRouteClick = () => {
    setSelectedPointType(null);
    setSelectedFile({
      group: selectedFile.group,
      map: selectedFile.map,
      route: "New route",
    });

    window.NAV2D.pointType = null;
    window.NAV2D.arePointsSettable = true;

    const mapElement = childRef.current.getMapRef();

    if (mapElement) {
      mapElement.addEventListener("mouseup", onMapClickHandler);
    }

    setPointsSettable(true);
    window.NAV2D.ClearMap();
    onOperationTopicPublish("clear_route");
  };

  const onPlanRouteClick = () => {
    if (!ros) {
      toast.error("ROS connection is offline!");
      return;
    }

    if (!pointsSettable) {
      toast.warn(
        "Please click 'Edit' or 'Create' first to enable path planning!",
      );
      return;
    }

    const startPose = window.NAV2D.currentPose;
    if (!startPose) {
      toast.error(
        "Waiting for robot pose (TF map -> base_link or /amcl_pose) to start planning...",
      );
      return;
    }

    toast.info(
      "Click and drag on the map to set the Goal pose for automatic path planning.",
    );

    const originalPoseCallback = window.NAV2D._poseCallback;
    window.NAV2D._poseCallback = (goalPose) => {
      // Restore original callback
      window.NAV2D._poseCallback = originalPoseCallback;

      // Remove the goal marker we just placed temporarily
      removePointFromCanvas();

      toast.info("Requesting path from Nav2 planner...");
      const nav2Client = new window.ROSLIB.Service({
        ros,
        name: AppConfig.COMPUTE_PATH_SERVICE,
        serviceType: "nav2_msgs/srv/ComputePathToPose",
      });

      const request = new window.ROSLIB.ServiceRequest({
        pose: {
          header: {
            frame_id: "map",
            stamp: { secs: 0, nsecs: 0 },
          },
          pose: {
            position: {
              x: goalPose.position.x,
              y: goalPose.position.y,
              z: 0.0,
            },
            orientation: {
              z: goalPose.orientation.z,
              w: goalPose.orientation.w,
            },
          },
        },
        start: {
          header: {
            frame_id: "map",
            stamp: { secs: 0, nsecs: 0 },
          },
          pose: {
            position: {
              x: startPose.position.x,
              y: startPose.position.y,
              z: 0.0,
            },
            orientation: {
              z: startPose.orientation.z,
              w: startPose.orientation.w,
            },
          },
        },
        planner_id: "",
        use_start: true,
      });

      nav2Client.callService(
        request,
        (result) => {
          if (
            result &&
            result.path &&
            Array.isArray(result.path.poses) &&
            result.path.poses.length > 0
          ) {
            const poses = result.path.poses;
            toast.success(
              `Successfully planned path with ${poses.length} points!`,
            );

            const downsampled = downsamplePath(poses, 1.0);

            const wayPointTopic = new window.ROSLIB.Topic({
              ros,
              name: AppConfig.NEW_WAYPOINT_TOPIC,
              messageType: "geometry_msgs/PoseWithCovarianceStamped",
            });

            downsampled.forEach((wp) => {
              const sendDataArray = new Array(36).fill(0.0);
              sendDataArray[0] = 3; // "navigate" type

              const messageObject = {
                header: { frame_id: "map" },
                pose: {
                  pose: {
                    position: {
                      x: wp.pose.position.x,
                      y: wp.pose.position.y,
                      z: 0.0,
                    },
                    orientation: {
                      z: wp.pose.orientation.z,
                      w: wp.pose.orientation.w,
                    },
                  },
                  covariance: sendDataArray,
                },
              };

              wayPointTopic.publish(new window.ROSLIB.Message(messageObject));
            });
          } else {
            toast.error(
              "Nav2 failed to plan a path: empty or invalid response.",
            );
          }
        },
        (error) => {
          console.error("Nav2 planning service error:", error);
          toast.error(
            "Failed to contact Nav2 planning service. Is Nav2 running?",
          );
        },
      );
    };
  };

  const onSaveRouteClick = () => {
    if (!pointsSettable) return;

    if (selectedFile.route !== "New route") {
      const dataToSend = {
        group: selectedFile.group,
        map: selectedFile.map,
        route: selectedFile.route,
      };

      const objToSendWithoutSpaces = processObjectStrings(dataToSend);
      const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);

      const messageToSend = `save_route/${stringifiedObjToSend}`;
      onOperationTopicPublish(messageToSend);

      setSelectedPointType(null);
      window.NAV2D.pointType = null;
      window.NAV2D.arePointsSettable = false;

      const mapElement = childRef.current.getMapRef();

      if (mapElement) {
        mapElement.removeEventListener("mouseup", onMapClickHandler);
      }

      setPointsSettable(false);
      return;
    }

    modalKey.current = "SAVE_ROUTE";
    textInputHeader.current = "Enter new route name";
    textInputPlaceholder.current = "Name...";
    setOpenInputModal(true);
  };

  const onChangeRouteClick = () => {
    console.log("filesData", filesData);
    window.NAV2D.arePointsSettable = false;
    setPointsSettable(false);

    modalKey.current = "CHANGE_ROUTE";
    routesModalType.current = "selectRoute";
    isRoutesModalWithInput.current = false;
    routesModalHeader.current = "Select route you want to browse";
    setOpenRouteModal(true);
  };

  const onChangeMapClick = () => {
    window.NAV2D.arePointsSettable = false;
    setPointsSettable(false);

    modalKey.current = "CHANGE_MAP";
    routesModalType.current = "selectMap";
    isRoutesModalWithInput.current = false;
    routesModalHeader.current = "Select map you want to load";
    setOpenRouteModal(true);
  };

  const onEditRouteClick = () => {
    if (pointsSettable) {
      setPointsSettable(false);
    } else {
      window.NAV2D.ClearMap();

      const currentRouteData = {
        group: selectedFile.group,
        map: selectedFile.map,
        route: selectedFile.route,
      };

      const objToSendWithoutSpaces = processObjectStrings(currentRouteData);
      const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);

      const messageToSend = `edit_route/${stringifiedObjToSend}`;

      onOperationTopicPublish(messageToSend);

      window.NAV2D.arePointsSettable = true;

      const mapElement = childRef.current.getMapRef();
      if (mapElement) {
        mapElement.addEventListener("mouseup", onMapClickHandler);
      }

      setPointsSettable(true);
    }
  };

  const onClearRouteClick = () => {
    if (!pointsSettable) return;
    window.NAV2D.ClearMap();
    onOperationTopicPublish("clear_route");
  };

  const onDeleteRouteClick = () => {
    window.NAV2D.ClearMap();

    const currentRouteData = {
      group: selectedFile.group,
      map: selectedFile.map,
      route: selectedFile.route,
    };

    const objToSendWithoutSpaces = processObjectStrings(currentRouteData);
    const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);

    const messageToSend = `delete_route/${stringifiedObjToSend}`;

    onOperationTopicPublish(messageToSend);

    // modalKey.current = "DELETE_ROUTE";
    // routesModalType.current = "selectRoute";
    // isRoutesModalWithInput.current = false;
    // routesModalHeader.current = "Select route you want to delete";
    // setOpenRouteModal(true);
  };

  const onRenameRouteClick = () => {
    // console.log(!pointsSettable || selectedFile.route === "New route");
    // if (selectedFile.route === "New route") return;
    modalKey.current = "RENAME_ROUTE";
    textInputHeader.current = "Enter route new name";
    textInputPlaceholder.current = "Name...";
    setOpenInputModal(true);
  };

  // eslint-disable-next-line no-unused-vars
  const onPointClickHandler = (data) => {
    setSelectedPointType(data);
    window.NAV2D.pointType = data;
  };

  // Flat "group / map" list across every group, for the Change Map modal —
  // RouteModal only knows how to show/select a flat list of strings, so a
  // map (one level up from routes) is presented as a combined label rather
  // than teaching that modal a second selection level.
  const mapOptions = allStructure.flatMap((groupObj) =>
    Object.entries(groupObj).flatMap(([group, maps]) =>
      maps.flatMap((mapObj) => Object.keys(mapObj).map((mapName) => `${group} / ${mapName}`)),
    ),
  );

  return (
    <>
      <ToastContainer position="bottom-right" theme="dark" />

      {openRouteModal && (
        <RouteModal
          routesList={modalKey.current === "CHANGE_MAP" ? mapOptions : filesData}
          headerText={routesModalHeader.current}
          modalHandler={onRouteFormSubmitHandler}
        />
      )}

      {openTimeModal && <TimeModal modalHandler={onTimeFormSubmitHandler} />}

      {openInputModal && (
        <TextInputModal
          header={textInputHeader.current}
          placeholder={textInputPlaceholder.current}
          routesList={filesData}
          modalHandler={onInputFormSubmitHandler}
        />
      )}

      <div className="sectionHeight space-y-5 py-4 sm:space-y-6 sm:py-6">
        <SectionHeader
          eyebrow="Route authoring"
          title="Plan reusable robot routes"
          description="Place, edit, and manage waypoint sequences for the active map."
          action={
            <StatusBadge
              status={pointsSettable ? "active" : "idle"}
              label={pointsSettable ? "Editing route" : "View mode"}
              pulse={pointsSettable}
            />
          }
        />

        <div className="grid grid-cols-1 gap-3 sm:grid-cols-3">
          {[
            ["Group", selectedFile.group],
            ["Map", selectedFile.map],
            ["Current route", selectedFile.route],
          ].map(([label, value]) => (
            <DashboardCard key={label} className="min-w-0 px-4 py-3">
              <p className="font-[RobotoMono] text-[10px] font-bold uppercase tracking-[0.14em] text-themeTextGray">
                {label}
              </p>
              <p
                className="mt-1 truncate font-[RobotoMono] text-sm font-semibold text-textWhiteHover"
                title={value || "Not available"}
              >
                {value || "—"}
              </p>
            </DashboardCard>
          ))}
        </div>

        <MapLayers />

        <div className="grid min-w-0 gap-4 xl:grid-cols-[minmax(0,1fr)_360px] xl:gap-5">
          <section className="h-[440px] min-w-0 sm:h-[560px] xl:h-[calc(100vh-315px)] xl:min-h-[500px]">
            <Map ref={childRef} />
          </section>

          <DashboardCard className="h-fit p-4 sm:p-5">
            <div className="mb-5 border-b border-borderSubtle pb-4">
              <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
                Route operations
              </p>
              <p className="mt-2 text-sm leading-6 text-themeTextGray">
                {pointsSettable
                  ? "Click the map to add or adjust points, then save your changes."
                  : "Choose an operation to begin editing the current route or create a new one."}
              </p>
            </div>

            <div className="grid grid-cols-2 gap-2">
              <Button onBtnClick={onEditRouteClick}>
                <span className="iconMap" aria-hidden="true" />
                <span>{pointsSettable ? "Cancel edit" : "Edit route"}</span>
              </Button>
              <Button
                onBtnClick={onSaveRouteClick}
                type={pointsSettable ? "success" : "disabled"}
              >
                <span className="iconSave" aria-hidden="true" />
                <span>Save</span>
              </Button>
              <Button
                onBtnClick={onNewRouteClick}
                type={pointsSettable ? "disabled" : ""}
              >
                <span className="iconPlus" aria-hidden="true" />
                <span>Create</span>
              </Button>
              <Button
                onBtnClick={onChangeRouteClick}
                type={pointsSettable ? "disabled" : ""}
              >
                <span className="iconCharge" aria-hidden="true" />
                <span>Change</span>
              </Button>
              <Button
                onBtnClick={onChangeMapClick}
                type={pointsSettable ? "disabled" : ""}
              >
                <span className="iconMap" aria-hidden="true" />
                <span>Change map</span>
              </Button>
              <Button onBtnClick={onPlanRouteClick}>
                <span className="iconMap" aria-hidden="true" />
                <span>Auto-plan</span>
              </Button>
              <Button
                onBtnClick={onRenameRouteClick}
                type={
                  pointsSettable || selectedFile.route === "Null"
                    ? "disabled"
                    : ""
                }
              >
                <span className="iconMap" aria-hidden="true" />
                <span>Rename</span>
              </Button>
              <Button
                onBtnClick={onClearRouteClick}
                type={!pointsSettable ? "disabled" : "danger"}
              >
                <span className="iconTrash" aria-hidden="true" />
                <span>Clear points</span>
              </Button>
              <Button
                onBtnClick={onDeleteRouteClick}
                type={pointsSettable ? "disabled" : "danger"}
              >
                <span className="iconTrash" aria-hidden="true" />
                <span>Delete</span>
              </Button>
            </div>
          </DashboardCard>
        </div>
      </div>
    </>
  );
};

export default RoutePage;

{
  /* 
          <Button  onBtnClick={() => onPointClickHandler("home")}>
            <span className="iconPoint" />
            <span
              className={`mx-auto ${
                selectedPointType === "home" ? "text-green-600" : "color-white"
              }`}
            >
              Home point
            </span>
          </Button>
          <Button  onBtnClick={() => onPointClickHandler("charge")}>
            <span className="iconCharge" />
            <span
              className={`mx-auto ${
                selectedPointType === "charge"
                  ? "text-green-600"
                  : "color-white"
              }`}
            >
              Charge point
            </span>
          </Button>
          <Button  onBtnClick={() => onPointClickHandler("navigate")}>
            <span className="iconInfo" />
            <span
              className={`mx-auto ${
                selectedPointType === "navigate"
                  ? "text-green-600"
                  : "color-white"
              }`}
            >
              Nav point
            </span>
          </Button>
           */
}
