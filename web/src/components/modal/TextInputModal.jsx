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
    <div className="fixed bottom-0 left-0 right-0 top-0 z-[100] flex items-start justify-center bg-black bg-opacity-80">
      <ToastContainer />

      <section className="relative top-[5vh] flex max-h-[90vh] w-1/2 min-w-[400px] flex-col items-center justify-center gap-3 rounded-[30px] bg-white p-8 font-[RobotoMono] xl:w-1/3">
        <h2 className="bg-themeMediumBlue w-full rounded-lg p-7 text-center text-3xl text-white">
          {header}
        </h2>

        <input
          type="text"
          value={inputValue}
          className="w-full rounded-lg px-8 py-3 text-xl text-black placeholder:text-themeTextGray"
          style={{ border: "1px solid #696969" }}
          placeholder={placeholder}
          onChange={(e) => setInputValue(e.target.value)}
        />

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

export default TextInputModal;
