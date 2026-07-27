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
    <div className="fixed inset-0 z-[100] flex items-center justify-center bg-black/80 p-4 backdrop-blur-sm">
      <section
        role="dialog"
        aria-modal="true"
        aria-labelledby="route-dialog-title"
        className="modal-surface flex max-h-[90vh] w-full max-w-lg flex-col gap-4 p-5 font-[RobotoMono] sm:p-6"
      >
        <h2
          id="route-dialog-title"
          className="text-xl font-bold text-textWhiteHover"
        >
          {headerText}
        </h2>

        <div className="max-h-[50vh] w-full overflow-y-auto rounded-xl border border-borderSubtle bg-bgSurface p-3">
          {routesList.length === 0 && (
            <p className="py-8 text-center text-sm text-themeTextGray">
              No routes available
            </p>
          )}
          <div>
            <ul className="flex flex-col gap-2">
              {routesList.map((route, index) => (
                <li key={index}>
                  <button
                    type="button"
                    className={`min-h-[44px] w-full rounded-lg border px-3 py-2 text-left text-sm ${
                      selectedRoute === route
                        ? "border-themeBlue/40 bg-themeBlue/10 text-themeBlue"
                        : "border-transparent text-textWhiteHover hover:border-borderSubtle hover:bg-bgCard"
                    }`}
                    onClick={() => setSelectedRoute(route)}
                  >
                    {route}
                  </button>
                </li>
              ))}
            </ul>
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
  );
};

export default RouteModal;
