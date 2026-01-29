import React, { useState } from "react";

import Button from "../../shared/ui/Button";

const RouteModal = ({ routesList, headerText, modalHandler }) => {
  const [selectedRoute, setSelectedRoute] = useState(null);

  const handleSubmitClick = () => {
    modalHandler(selectedRoute);
  };

  const handleCancelClick = () => {
    modalHandler(false);
  };

  return (
    <div className="fixed bottom-0 left-0 right-0 top-0 z-[100] flex items-start justify-center bg-black bg-opacity-80">
      <section className="relative top-[5vh] flex max-h-[90vh] w-1/2 min-w-[400px] flex-col items-center justify-center gap-3 rounded-[30px] bg-white p-8 font-[RobotoMono] xl:w-1/3">
        <h2 className="bg-themeMediumBlue text-white w-full rounded-lg p-7 text-center text-3xl">
          {headerText}
        </h2>

        <div className="h-full w-full overflow-x-auto rounded-lg bg-themeLightGray px-8 py-3">
          {routesList.length === 0 && (
            <h2 className="text-center">List is empty</h2>
          )}
          <div>
            <ul className="mx-16 mt-2 flex flex-col gap-2.5">
              {routesList.map((route, index) => (
                <li
                  key={index}
                  className={`hover:text-themeMediumBlue cursor-pointer ${
                    selectedRoute === route
                      ? "text-themeDarkBlue"
                      : "text-black"
                  }`}
                  onClick={() => setSelectedRoute(route)}
                >
                  {route}
                </li>
              ))}
            </ul>
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
  );
};

export default RouteModal;
