import React, { useState } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import Button from "../../shared/ui/Button";

const TextInputModal = ({ header, placeholder, routesList, modalHandler }) => {
  const [inputValue, setInputValue] = useState("");

  const handleSubmitClick = () => {
    const trimmedValue = inputValue.trim();
    if (!trimmedValue) {
      toast.warn("Enter route name");
      return;
    }

    if (trimmedValue === "Null" || trimmedValue === "New route") {
      toast.warn("This name is reserved");
      return;
    }

    if (trimmedValue.toString().includes("_")) {
      toast.warn("Symbol '_' is forbidden");
      return;
    }

    const isRouteWithPassedNameExist = routesList.find(
      (route) => route === trimmedValue,
    );
    if (isRouteWithPassedNameExist) {
      toast.warn("Route with passed name is already exist");
      return;
    }
    modalHandler(trimmedValue);
  };

  const handleCancelClick = () => {
    modalHandler(false);
  };

  return (
    <div className="fixed inset-0 z-[100] flex items-center justify-center bg-black/80 p-4 backdrop-blur-sm">
      <ToastContainer theme="dark" />

      <section
        role="dialog"
        aria-modal="true"
        aria-labelledby="text-input-dialog-title"
        className="modal-surface flex max-h-[90vh] w-full max-w-lg flex-col gap-4 p-5 font-[RobotoMono] sm:p-6"
      >
        <h2
          id="text-input-dialog-title"
          className="text-xl font-bold text-textWhiteHover"
        >
          {header}
        </h2>

        <input
          type="text"
          value={inputValue}
          autoFocus
          className="min-h-[46px] w-full rounded-xl border border-borderSubtle bg-bgSurface px-4 py-3 text-base text-textWhiteHover placeholder:text-themeTextGray focus:border-themeBlue"
          placeholder={placeholder}
          onChange={(e) => setInputValue(e.target.value)}
        />

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

export default TextInputModal;
