import React from "react";

const MapButton = ({ direction, type, onBtnClick }) => {
  let rotateClass = "";
  let typeClass = "";

  if (type === "arrow") {
    typeClass = "iconArrow";
    switch (direction) {
      case "left":
        rotateClass = "";
        break;
      case "top":
        rotateClass = "rotate-90";
        break;
      case "right":
        rotateClass = "rotate-180";
        break;
      case "bottom":
        rotateClass = "-rotate-90";
        break;
    }
  }

  if (type === "zoom" && direction === "plus") typeClass = "iconPlus";
  if (type === "zoom" && direction === "minus") typeClass = "iconMinus";

  return (
    <button
      className="flex h-[44px] w-[44px] items-center justify-center rounded-lg border border-borderSubtle bg-bgCard text-themeBlue shadow-sm shadow-slate-200 transition-colors hover:border-themeBlue hover:bg-themeBlue hover:text-white active:bg-themeDarkBlue xl:h-[48px] xl:w-[48px]"
      onClick={onBtnClick}
    >
      <span className={`${typeClass} ${rotateClass}`} />
    </button>
  );
};

export default MapButton;
