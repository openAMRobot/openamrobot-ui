import React, { useState } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import Button from "../../shared/ui/Button";

const isGroupExists = (dataList, groupName) => {
  return dataList.some((group) => groupName in group);
};

// eslint-disable-next-line no-unused-vars
const isMapExistsInGroup = (dataList, groupName, mapName) => {
  const group = dataList.find((group) => groupName in group);
  if (!group) return true;
  return !Object.keys(group[groupName]).includes(mapName);
};

const MapComponent = ({
  mapKey,
  routes,
  onMapClick,
  mode,
  selectedMap,
  onRouteClick,
  selectedRoute,
}) => {
  const [showRoutes, setShowRoutes] = useState(false);

  const handleMapClick = () => {
    if (mode === "selectMap") {
      onMapClick(mapKey);
    }
  };

  const handleRouteClick = (route) => {
    if (mode === "selectRoute") {
      onRouteClick(route, mapKey);
    }
  };
  return (
    <div>
      <div className="flex items-center justify-between">
        <h4
          onClick={handleMapClick}
          className={`${
            mode === "selectMap"
              ? "cursor-pointer hover:text-themeMediumBlue"
              : ""
          } ${
            selectedMap === mapKey && mode === "selectMap"
              ? "text-themeDarkBlue"
              : "text-black"
          }`}
        >
          {mapKey} ({routes.length})
        </h4>
        <button
          onClick={() => setShowRoutes(!showRoutes)}
          className={`rounded bg-themeMediumBlue px-2 py-1 text-sm text-white  ${
            routes.length > 0 ? "hover:bg-themeDarkBlue" : "opacity-40"
          }`}
          disabled={routes.length === 0}
        >
          {showRoutes ? "hide" : "show"}
        </button>
      </div>
      {showRoutes && (
        <ul className="mx-16 mt-2 flex flex-col gap-2.5">
          {routes.map((route, index) => (
            <li
              key={index}
              className={`${
                mode === "selectRoute"
                  ? "cursor-pointer hover:text-themeMediumBlue"
                  : ""
              } ${
                selectedRoute === route ? "text-themeDarkBlue" : "text-black"
              }`}
              onClick={() => handleRouteClick(route)}
            >
              {route}
            </li>
          ))}
        </ul>
      )}
    </div>
  );
};

const FilesModal = ({
  filesList,
  headerText,
  hasInput,
  inputPlaceholder,
  mode,
  modalHandler,
}) => {
  const [selectedGroup, setSelectedGroup] = useState(null);
  const [selectedMap, setSelectedMap] = useState(null);
  const [selectedRoute, setSelectedRoute] = useState(null);
  const [inputValue, setInputValue] = useState("");

  const handleMapClick = (mapKey, group) => {
    if (mode === "selectMap" || mode === "selectRoute") {
      setSelectedMap(mapKey);
      setSelectedGroup(group);
    }
  };

  const onRouteClick = (mapKey, route, group) => {
    if (mode === "selectRoute") {
      setSelectedRoute(route);
      setSelectedMap(mapKey);
      setSelectedGroup(group);
    }
  };

  const handleGroupClick = (group) => {
    setSelectedGroup(group);
    if (mode === "selectGroup") {
      setSelectedMap(null);
      setSelectedRoute(null);
    }
  };

  const handleSubmitClick = () => {
    let dataToSend = {};

    if (mode === "selectGroup") {
      if (isGroupExists(filesList, inputValue)) {
        toast.warn("Group with current name is already exist");
        return;
      }

      dataToSend = { group: selectedGroup, inputValue };
    } else if (mode === "selectMap" && hasInput) {
      dataToSend = {
        group: selectedGroup,
        map: selectedMap,
        inputValue,
      };
    } else if (mode === "selectMap") {
      dataToSend = {
        group: selectedGroup,
        map: selectedMap,
      };
    } else if (mode === "selectRoute") {
      dataToSend = {
        group: selectedGroup,
        map: selectedMap,
        route: selectedRoute,
        inputValue,
      };
    }
    modalHandler(dataToSend);
  };

  const handleCancelClick = () => {
    modalHandler(false);
  };

  return (
    <>
      <ToastContainer />
      <div className="fixed bottom-0 left-0 right-0 top-0 z-[100] flex items-start justify-center bg-black bg-opacity-80">
        <section className="relative top-[5vh] flex max-h-[90vh] w-1/2 min-w-[400px] flex-col items-center justify-center gap-3 rounded-[30px] bg-white p-8 font-[RobotoMono] xl:w-1/3">
          <h2 className="w-full rounded-lg bg-themeMediumBlue p-7 text-center text-3xl text-white">
            {headerText}
          </h2>

          {hasInput && (
            <input
              type="text"
              value={inputValue}
              className="w-full rounded-lg px-8 py-3 text-xl text-black placeholder:text-themeTextGray"
              style={{ border: "1px solid #696969" }}
              placeholder={inputPlaceholder}
              onChange={(e) => setInputValue(e.target.value)}
            />
          )}
          <div className="h-full w-full overflow-x-auto rounded-lg bg-themeLightGray px-8 py-3">
            {filesList.length === 0 && (
              <h2 className="text-center">List is empty</h2>
            )}
            <div>
              {filesList.map((floor) => {
                const floorKey = Object.keys(floor)[0];
                return (
                  <article
                    className="mt-2.5 flex flex-col gap-2.5"
                    key={floorKey}
                  >
                    <h3
                      className={`text-center text-2xl ${
                        mode === "selectGroup"
                          ? "cursor-pointer hover:text-themeMediumBlue"
                          : ""
                      } ${
                        selectedGroup === floorKey && mode === "selectGroup"
                          ? "text-themeDarkBlue"
                          : "text-black"
                      }`}
                      onClick={() => {
                        if (mode === "selectGroup") {
                          handleGroupClick(floorKey);
                        }
                      }}
                    >
                      {floorKey}
                    </h3>
                    {floor[floorKey].map((roomObj) => {
                      const mapKey = Object.keys(roomObj)[0];
                      return (
                        <MapComponent
                          key={mapKey}
                          mapKey={mapKey}
                          routes={roomObj[mapKey]}
                          onMapClick={(map) => handleMapClick(map, floorKey)}
                          onRouteClick={(route) =>
                            onRouteClick(mapKey, route, floorKey)
                          }
                          mode={mode}
                          selectedRoute={selectedRoute}
                          selectedMap={selectedMap}
                        />
                      );
                    })}
                  </article>
                );
              })}
            </div>
          </div>
          <div className="flex w-full justify-center gap-8">
            <div className="w-1/4">
              <Button type={"gray"} onBtnClick={handleCancelClick}>
                Cancel
              </Button>
            </div>
            <div className="w-1/4">
              <Button type={"orange"} onBtnClick={handleSubmitClick}>
                Ok
              </Button>
            </div>
          </div>
        </section>
      </div>
    </>
  );
};

export default FilesModal;
