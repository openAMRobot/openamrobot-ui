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

  if (type === "plus") typeClass = "iconPlus";
  if (type === "minus") typeClass = "iconMinus";

  return (
    <button
      className="mapButton active:bg-themeDarkBlue flex h-[40px] w-[40px] items-center justify-center rounded-lg bg-white font-[RobotoMono] hover:bg-themeLightGray xl:h-[52px] xl:w-[52px]"
      onClick={onBtnClick}
    >
      <span className={`${typeClass} ${rotateClass}`} />
    </button>
  );
};
export default MapButton;
