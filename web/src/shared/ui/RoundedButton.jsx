import React from "react";

const RoundedButton = ({ type, children, onBtnClick }) => {
  let cls = "";
  switch (type) {
    case "success":
      cls =
        "bg-statusGreen hover:bg-green-500 active:bg-green-600 shadow-lg shadow-green-200 dark:shadow-green-900/40";
      break;
    case "danger":
      cls =
        "bg-statusRed hover:bg-red-500 active:bg-red-600 shadow-lg shadow-red-200 dark:shadow-red-900/40";
      break;
    default:
      cls = "bg-themeBlue hover:bg-themeMediumBlue active:bg-themeDarkBlue";
  }

  return (
    <button
      className={`${cls} flex h-[110px] w-[110px] items-center justify-center rounded-full font-[RobotoMono] text-[28px] font-bold text-white transition-colors lg:h-[130px] lg:w-[130px] 2xl:h-[160px] 2xl:w-[160px]`}
      onClick={onBtnClick}
    >
      {children}
    </button>
  );
};

export default RoundedButton;
