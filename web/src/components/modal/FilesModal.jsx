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
        <button
          type="button"
          disabled={mode !== "selectMap"}
          onClick={handleMapClick}
          className={`min-h-[40px] rounded-lg px-2 text-left text-sm font-semibold ${
            mode === "selectMap"
              ? "cursor-pointer hover:text-themeMediumBlue"
              : ""
          } ${
            selectedMap === mapKey && mode === "selectMap"
              ? "bg-themeBlue/10 text-themeBlue"
              : "text-textWhiteHover"
          }`}
        >
          {mapKey} ({routes.length})
        </button>
        <button
          type="button"
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
        <ul className="ml-4 mt-2 flex flex-col gap-1">
          {routes.map((route, index) => (
            <li key={index}>
              <button
                type="button"
                disabled={mode !== "selectRoute"}
                className={`min-h-[40px] w-full rounded-lg px-2 text-left text-sm ${
                  selectedRoute === route
                    ? "bg-themeBlue/10 text-themeBlue"
                    : "text-themeTextGray hover:bg-bgCard hover:text-textWhiteHover"
                }`}
                onClick={() => handleRouteClick(route)}
              >
                {route}
              </button>
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
      <ToastContainer theme="dark" />
      <div className="fixed inset-0 z-[100] flex items-center justify-center bg-black/80 p-4 backdrop-blur-sm">
        <section
          role="dialog"
          aria-modal="true"
          aria-labelledby="files-dialog-title"
          className="modal-surface flex max-h-[90vh] w-full max-w-xl flex-col gap-4 p-5 font-[RobotoMono] sm:p-6"
        >
          <h2
            id="files-dialog-title"
            className="text-xl font-bold text-textWhiteHover"
          >
            {headerText}
          </h2>

          {hasInput && (
            <input
              type="text"
              value={inputValue}
              className="min-h-[46px] w-full rounded-xl border border-borderSubtle bg-bgSurface px-4 py-3 text-base text-textWhiteHover placeholder:text-themeTextGray focus:border-themeBlue"
              placeholder={inputPlaceholder}
              onChange={(e) => setInputValue(e.target.value)}
            />
          )}
          <div className="w-full overflow-y-auto rounded-xl border border-borderSubtle bg-bgSurface p-3">
            {filesList.length === 0 && (
              <p className="py-8 text-center text-sm text-themeTextGray">
                No files available
              </p>
            )}
            <div>
              {filesList.map((floor) => {
                const floorKey = Object.keys(floor)[0];
                return (
                  <article
                    className="mt-2.5 flex flex-col gap-2.5"
                    key={floorKey}
                  >
                    <button
                      type="button"
                      disabled={mode !== "selectGroup"}
                      className={`min-h-[42px] w-full rounded-lg px-3 text-left text-base font-semibold ${
                        mode === "selectGroup"
                          ? "cursor-pointer hover:text-themeMediumBlue"
                          : ""
                      } ${
                        selectedGroup === floorKey && mode === "selectGroup"
                          ? "bg-themeBlue/10 text-themeBlue"
                          : "text-textWhiteHover"
                      }`}
                      onClick={() => {
                        if (mode === "selectGroup") {
                          handleGroupClick(floorKey);
                        }
                      }}
                    >
                      {floorKey}
                    </button>
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
          <div className="grid w-full grid-cols-2 gap-3">
            <div>
              <Button type={"gray"} onBtnClick={handleCancelClick}>
                Cancel
              </Button>
            </div>
            <div>
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
