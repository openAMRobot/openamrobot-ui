import React from "react";

const Button = ({ children, onBtnClick, type }) => {
  let buttonTypesClasses = "";
  switch (type) {
    case "gray":
      buttonTypesClasses =
        "hover:bg-themeMediumGray active:bg-themeTextGray active:text-white bg-themeLightGray text-black";
      break;
    case "orange":
      buttonTypesClasses =
        "text-white bg-themeMediumBlue hover:bg-themeDarkBlue text-black";
      break;
    case "disabled":
      buttonTypesClasses =
        "border border-themeBlue opacity-40 disabled cursor-auto bg-white text-themeDarkBlue";
      break;
    default:
      buttonTypesClasses =
        "border border-themeBlue hover:bg-themeLightGray active:bg-themeDarkBlue active:text-white bg-white text-themeDarkBlue";
  }

  // const buttonSize =
  //   size === "small"
  //     ? "px-3 2xl:px-[15px] py-2 2xl:py-[12px] "
  //     : "px-4 py-3 2xl:p-[20px]";

  return (
    <button
      className={`px-3 py-2 ${buttonTypesClasses} functionalButton flex w-full items-center justify-center gap-2 rounded-lg font-[RobotoMono] text-lg`}
      onClick={type !== "disabled" ? onBtnClick : () => {}}
    >
      {children}
    </button>
  );
};
export default Button;
