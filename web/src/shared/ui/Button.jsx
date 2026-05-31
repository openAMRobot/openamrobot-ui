import React from "react";

const Button = ({ children, onBtnClick, type }) => {
  let cls = "";
  switch (type) {
    case "disabled":
      cls =
        "opacity-30 cursor-not-allowed border border-borderSubtle text-themeTextGray bg-bgSurface";
      break;
    case "danger":
      cls =
        "bg-statusRed hover:bg-red-600 active:bg-red-700 text-white border border-statusRed";
      break;
    case "success":
      cls =
        "bg-statusGreen hover:bg-green-500 active:bg-green-600 text-white border border-statusGreen";
      break;
    case "orange":
      cls =
        "bg-themeMediumBlue hover:bg-themeDarkBlue text-white border border-themeMediumBlue";
      break;
    default:
      cls =
        "border border-themeBlue text-themeBlue hover:bg-themeBlue hover:text-white active:bg-themeDarkBlue active:text-white bg-transparent transition-colors";
  }

  return (
    <button
      className={`px-3 py-2 ${cls} flex w-full items-center justify-center gap-2 rounded-lg font-[RobotoMono] text-base`}
      onClick={type !== "disabled" ? onBtnClick : () => {}}
    >
      {children}
    </button>
  );
};

export default Button;
