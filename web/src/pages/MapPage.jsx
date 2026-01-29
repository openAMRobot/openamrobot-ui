import React, { useContext, useState, useRef, useEffect } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { AppConfig } from "../shared/constants/index";
import { testStructure } from "../shared/constants/testjson";

import { RosContext } from "../app/App";

import Map from "../components/Map";
import Camera from "../components/Camera";
import Logs from "../components/Logs";
import Joystick from "../components/Joystick";
import FilesModal from "../components/modal/FilesModal";
import Button from "../shared/ui/Button";

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

const MapPage = () => {
  const ros = useContext(RosContext);
  const [openModal, setOpenModal] = useState(false);
  const [filesData, setFilesData] = useState([]);
  const [selectedFile, setSelectedFile] = useState({ group: "", map: "" });
  const [inEditMode, setIsEditMode] = useState(false);

  const modalKey = useRef(null);
  const filesModalType = useRef(null);
  const isFilesModalWithInput = useRef(null);
  const filesModalHeader = useRef(null);
  const filesModalPlaceholder = useRef(null);

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
    getMockedData();

    const currentFilesResponseTopic = filesResonseTopic.current;

    currentFilesResponseTopic.subscribe((data) => {
      const response = data.data;
      const responseObject = JSON.parse(response);
      const serializedArray = removeCsv(responseObject.structure);
      const arrayWithSpaces =
        replaceUnderscoresInKeysAndValues(serializedArray);
      const activeFilesWithSpacec = replaceUnderscoresInKeysAndValues(
        responseObject.active_files,
      );

      setFilesData(arrayWithSpaces);
      setSelectedFile(activeFilesWithSpacec);
    });

    filesReqTopic.current.publish();

    return () => currentFilesResponseTopic.unsubscribe();
  }, []);

  /* BUTTON HANDLERS */

  const onNewMapClick = () => {
    uiOperationTopic.current.publish(
      new window.ROSLIB.Message({ data: "build_map" }),
    );
    setIsEditMode(true);
  };

  const onSaveMapClick = () => {
    modalKey.current = "SaveMap";
    filesModalType.current = "selectGroup";
    isFilesModalWithInput.current = true;
    filesModalHeader.current = "Select group for saving the map";
    filesModalPlaceholder.current = "Enter map name...";
    setOpenModal(true);
  };

  const onEditMapClick = () => {
    modalKey.current = "EditMap";
    setOpenModal(true);
  };

  const onCreateGroupClick = () => {
    modalKey.current = "CreateGroup";
    filesModalType.current = "selectGroup";
    isFilesModalWithInput.current = true;
    filesModalHeader.current = "Create group";
    filesModalPlaceholder.current = "Enter new group name...";
    setOpenModal(true);
  };

  const onChangeMapClick = () => {
    modalKey.current = "ChangeMap";
    filesModalType.current = "selectMap";
    isFilesModalWithInput.current = false;
    filesModalHeader.current = "Choose map:";
    setOpenModal(true);
  };

  const onRenameMapClick = () => {
    modalKey.current = "RenameMap";
    filesModalType.current = "selectMap";
    isFilesModalWithInput.current = true;
    filesModalHeader.current = "Choose map you want to rename";
    filesModalPlaceholder.current = "Enter map new name...";
    setOpenModal(true);
  };

  const onDeleteMapClick = () => {
    modalKey.current = "DeleteMap";
    filesModalType.current = "selectMap";
    isFilesModalWithInput.current = false;
    filesModalHeader.current = "Choose map you want to delete";
    setOpenModal(true);
  };

  const onDeleteGroupClick = () => {
    modalKey.current = "DeleteGroup";
    filesModalType.current = "selectGroup";
    isFilesModalWithInput.current = false;
    filesModalHeader.current = "Choose group you want to delete";
    setOpenModal(true);
  };

  /* FORM SUBMIT HANDLER */
  const onFormSubmitHandler = (data) => {
    setOpenModal(false);
    let isBreaked = false;

    if (data) {
      const operationsConfig = {
        SaveMap: {
          path: "save_map",
          data: { group: data.group, map: data.inputValue },
          preActions: () => {
            if (!data.group || !data.inputValue) {
              isBreaked = true;
              toast.warn("You need to select group and provide map name");
              return;
            }

            if (data.inputValue.toString().includes("_")) {
              isBreaked = true;
              toast.warn("Symbol '_' is forbidden");
              return;
            }
          },
        },
        EditMap:{
          preActions: () => {toast.warn("draw tools not implemented yet");}
        },
        CreateGroup: {
          path: "create_group",
          data: { group: data.inputValue },
          preActions: () => {
            if (!data.inputValue) {
              isBreaked = true;
              toast.warn("You need to provide group name");
              return;
            }

            if (data.inputValue.toString().includes("_")) {
              isBreaked = true;
              toast.warn("Symbol '_' is forbidden");
              return;
            }
          },
        },
        ChangeMap: {
          path: "change_map",
          data: { group: data.group, map: data.map },
        },
        RenameMap: {
          path: "rename_map",
          data: {
            group: data.group,
            map_old: data.map,
            map_new: data.inputValue,
          },
          preActions: () => {
            if (!data.map || !data.inputValue) {
              isBreaked = true;
              toast.warn("You need to choose map and provide map new name");
              return;
            }

            if (data.inputValue.toString().includes("_")) {
              isBreaked = true;
              toast.warn("Symbol '_' is forbidden");
              return;
            }
          },
        },
        DeleteMap: {
          path: "delete_map",
          data: { group: data.group, map: data.map },
        },
        DeleteGroup: {
          path: "delete_group",
          data: { group: data.group },
        },
      };

      const currentOperation = operationsConfig[modalKey.current];

      if (currentOperation) {
        currentOperation.preActions && currentOperation.preActions();
        if (isBreaked) return;
        const objectWithoutSpaces = processObjectStrings(currentOperation.data);

        const stringifiedObjToSend = JSON.stringify(objectWithoutSpaces);
        const messageToSend = `${currentOperation.path}/${stringifiedObjToSend}`;
        uiOperationTopic.current.publish(
          new window.ROSLIB.Message({ data: messageToSend }),
        );

        currentOperation.postActions && currentOperation.postActions();
      }
      setIsEditMode(false);
    }

    modalKey.current = null;
    filesModalType.current = null;
    isFilesModalWithInput.current = null;
    filesModalHeader.current = null;
    filesModalPlaceholder.current = null;
  };

  return (
    <>
      <ToastContainer />

      {openModal && (
        <FilesModal
          filesList={filesData}
          headerText={filesModalHeader.current}
          mode={filesModalType.current}
          hasInput={isFilesModalWithInput.current}
          inputPlaceholder={filesModalPlaceholder.current}
          modalHandler={onFormSubmitHandler}
        />
      )}
      <div className="sectionHeight flex flex-col gap-7 pt-[30px]">
        <h2 className="w-full text-center font-[RobotoMono] text-3xl font-bold text-themeBlue">
          Group:{" "}
          <span className="text-themeDarkBlue">{selectedFile.group} </span>
          Map: <span className="text-themeDarkBlue">{selectedFile.map}</span>
        </h2>
        <section className="color-white flex w-full gap-[8%]">
          <div className="h-[300px] w-1/2 xl:h-[486px] ">
            <Map />
          </div>
          <div className="h-[300px] w-1/2 xl:h-[486px]">
            <Camera />
          </div>
        </section>

        <section className="mt-6 flex w-full justify-between gap-10 xl:mt-0">
          <div className="flex h-full w-1/2 min-w-[400px] max-w-[700px] items-center justify-between gap-4">
            <div className="mr-auto flex w-full flex-col gap-2">
              <div className="flex w-full flex-col items-center justify-center gap-2 md:flex-row">
                <Button
                  size="small"
                  onBtnClick={onChangeMapClick}
                  type={inEditMode ? "disabled" : ""}
                >
                  <span className="iconMap" />
                  <span className="mx-auto">Change</span>
                </Button>

                <Button
                  size="small"
                  onBtnClick={onEditMapClick}
                  type={inEditMode ? "disabled" : ""}
                >
                  <span className="iconMap" />
                  <span className="mx-auto">Edit</span>
                </Button>

                <Button
                  size="small"
                  onBtnClick={onSaveMapClick}
                  type={inEditMode ? "" : "disabled"}
                  disabled={true}
                >
                  <span className="iconMap" />
                  <span className="mx-auto">Save</span>
                </Button>
              </div>

              <div className="flex w-full flex-col items-center justify-center gap-2 md:flex-row">
                <Button
                  size="small"
                  onBtnClick={onNewMapClick}
                  type={inEditMode ? "disabled" : ""}
                  disabled={false}
                >
                  <span className="iconMap" />
                  <span className="mx-auto"> Create </span>
                </Button>

                <Button
                  size="small"
                  onBtnClick={onRenameMapClick}
                  type={inEditMode ? "disabled" : ""}
                >
                  <span className="iconMap" />
                  <span className="mx-auto">Rename</span>
                </Button>

                <Button
                  size="small"
                  onBtnClick={onDeleteMapClick}
                  type={inEditMode ? "disabled" : ""}
                >
                  <span className="iconMap" />
                  <span className="mx-auto">Delete</span>
                </Button>
              </div>

              <div className="flex w-full flex-col items-center justify-center gap-2 md:flex-row">
                <Button size="small" onBtnClick={onCreateGroupClick}>
                  <span className="iconGroup" />
                  <span className="mx-auto">Create</span>
                </Button>

                <Button
                  size="small"
                  onBtnClick={onDeleteGroupClick}
                  type={inEditMode ? "disabled" : ""}
                >
                  <span className="iconGroup" />
                  <span className="mx-auto">Delete</span>
                </Button>
              </div>
            </div>
          </div>

          <Joystick />

          <div className="max-h-[180px] w-1/3 max-w-[700px] flex-grow">
            <Logs />
          </div>
        </section>
      </div>
    </>
  );
};

export default MapPage;
