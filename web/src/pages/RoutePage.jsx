import React, {
  useContext,
  useRef,
  useState,
  useEffect,
  useCallback,
} from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { RosContext } from "../app/App";
import { AppConfig } from "../shared/constants/index";
import { testStructure } from "../shared/constants/testjson";

import TimeModal from "../components/modal/TimeModal";
import RouteModal from "../components/modal/RouteModal";
import TextInputModal from "../components/modal/TextInputModal";

import Map from "../components/Map";
import Logs from "../components/Logs";
import Button from "../shared/ui/Button";
import { cos } from "three/examples/jsm/nodes/Nodes.js";

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

const removePointFromCanvas = () => {
  const markerOnMap = window.NAV2D.orientatedPointItem;
  window.NAV2D.pointsArray = window.NAV2D.pointsArray.filter(
    (marker) => marker !== markerOnMap,
  );
  window.NAV2D.canvas.scene.removeChild(markerOnMap);
  window.NAV2D.finishedPointItem = null;
  window.NAV2D.orientatedPointItem = null;
};

const RoutePage = () => {
  const ros = useContext(RosContext);
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
      name: "/nav_data_req",
      messageType: "std_msgs/Empty",
    }),
  );

  const filesResonseTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: "/nav_data_resp",
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
    /**
     * У нас при отжатии кнопки канвас генерирует window.NAV2D.orientatedPointItem, который нужен внутри sendPointToRobot, onMapClickHandler тоже срабатывает при отжатии кнопки и на момент вызова sendPointToRobot orientatedPointItem ещё не создан. Для решения проблемы сейчас и используется костыль из таймера.
     */

    if (!window.NAV2D.arePointsSettable) return;

    let timeObj = {
      hours: latestHoursValue.current,
      minutes: latestMinutesValue.current,
    };

    setTimeout(() => {
      if (!timeObj.hours && !timeObj.minutes) {
        toast.warn("Enter hours and minutes");
        removePointFromCanvas();
        return;
      }

      if (timeObj.hours < 0 || timeObj.hours > 23) {
        toast.warn("Hours can't be less then 0 and more then 23");
        removePointFromCanvas();
        return;
      }

      if (timeObj.minutes < 0 || timeObj.minutes > 59) {
        toast.warn("Minutes can't be less then 0 and more then 59");
        removePointFromCanvas();
        return;
      }

      window.NAV2D.sendPointToRobot(ros, timeObj);
    }, 0);
  }, [ros]);

  // eslint-disable-next-line no-unused-vars
  const getMockedData = () => {
    const serializedTestArray = removeCsv(testStructure.structure);
    const serializedTestArrayWithSpaces =
      replaceUnderscoresInKeysAndValues(serializedTestArray);
    const testActiveFileWithSpaces = replaceUnderscoresInKeysAndValues(
      testStructure.active_files,
    );

    setFilesData(serializedTestArrayWithSpaces);
    setSelectedFile(testActiveFileWithSpaces);
  };

  useEffect(() => {
    window.NAV2D.ClearMap();
    onOperationTopicPublish("clear_route");

    getMockedData();

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
      setSelectedFile(activeFilesWithSpaces);
    });

    filesReqTopic.current.publish();

    return () => {
      currentFilesResponseTopic.unsubscribe();
      window.NAV2D.arePointsSettable = false;

      const mapElement = currentMap.getMapRef();
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
            setFilesData({
              group: selectedFile.group,
              map: selectedFile.map,
              route: data,
            }),
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
            selectedFile.route = data;
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
    toast.warn("Comming soon...");
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

  return (
    <>
      <ToastContainer />

      {openRouteModal && (
        <RouteModal
          routesList={filesData}
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

      <div className="sectionHeight flex flex-col gap-7 pt-[30px]">
        <h2 className="flex w-full flex-wrap items-center justify-center gap-3 text-center font-[RobotoMono] text-3xl font-bold text-themeBlue">
          <span>
            Group:{" "}
            <span className="text-themeDarkBlue">{selectedFile.group}</span>
          </span>
          <span>
            {" "}
            Map: <span className="text-themeDarkBlue">{selectedFile.map}</span>
          </span>
          <span>
            {selectedFile.route !== "New route" && <span>Current route:</span>}{" "}
            <span className="text-themeDarkBlue">{selectedFile.route}</span>
          </span>
        </h2>

        <div className="flex h-full flex-col justify-start gap-[73px] md:flex-row">
          <section className="color-white flex w-full flex-col items-start justify-start md:w-[55%]">
            <div className="h-[400px] w-full lg:h-[674px]">
              <Map ref={childRef} />
            </div>
          </section>

          <section className="color-white flex h-[400px] w-full flex-1 flex-col items-center justify-start gap-7 md:w-2/5 lg:gap-[233px]">
            <div className="flex w-full flex-col gap-6 lg:gap-3">
              <div className="flex w-full flex-col items-center justify-center gap-6 lg:flex-row lg:gap-3">
                <div className="w-full flex-1">
                  <Button onBtnClick={onEditRouteClick}>
                    {pointsSettable && <div>Cancel</div>}
                    {!pointsSettable && (
                      <div>
                        <span className="iconMap" />
                        <span className="mx-auto">Edit</span>
                      </div>
                    )}
                  </Button>
                </div>
                <div className="w-full flex-1">
                  <Button
                    onBtnClick={onChangeRouteClick}
                    type={pointsSettable ? "disabled" : ""}
                  >
                    <span className="iconCharge" />
                    <span className="mx-auto">Change</span>
                  </Button>
                </div>
                <div className="w-full flex-1">
                  <Button
                    onBtnClick={onSaveRouteClick}
                    type={pointsSettable ? "" : "disabled"}
                  >
                    <span className="iconSave" />
                    <span className="mx-auto">Save</span>
                  </Button>
                </div>
              </div>

              <div className="flex w-full flex-col items-center justify-center gap-6 lg:flex-row lg:gap-4">
                <div className="w-full flex-1">
                  <Button
                    onBtnClick={onNewRouteClick}
                    type={pointsSettable ? "disabled" : ""}
                  >
                    <span className="iconPlus" />
                    <span className="mx-auto">Create</span>
                  </Button>
                </div>
                <div className="w-full flex-1">
                  <Button
                    size="small"
                    onBtnClick={onPlanRouteClick}
                  >
                    <span className="iconMap" />
                    <span className="mx-auto">Plan</span>
                  </Button>
                </div>
                <div className="w-full flex-1">
                  <Button
                    onBtnClick={onRenameRouteClick}
                    type={
                      pointsSettable || selectedFile.route === "Null"
                        ? "disabled"
                        : ""
                    }
                  >
                    <span className="iconMap" />
                    <span className="mx-auto">Rename</span>
                  </Button>
                </div>
                <div className="w-full flex-1">
                  <Button
                    onBtnClick={onDeleteRouteClick}
                    type={pointsSettable ? "disabled" : ""}
                  >
                    <span className="iconTrash" />
                    <span className="mx-auto">Delete</span>
                  </Button>
                </div>
              </div>

              <div className="flex w-full flex-col items-center justify-center gap-6 lg:flex-row lg:gap-3">
                <div className="w-full flex-1">
                  <Button
                    onBtnClick={onClearRouteClick}
                    type={!pointsSettable ? "disabled" : ""}
                  >
                    <span className="iconMap" />
                    <span className="mx-auto">Clear</span>
                  </Button>
                </div>
              </div>
            </div>

            <div className="h-full max-h-[200px] min-h-[200px] w-full flex-grow">
              <Logs />
            </div>
          </section>
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
