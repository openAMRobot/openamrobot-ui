import React from "react";

const Button = ({ children, onBtnClick, type }) => {
  let cls = "";
  const disabled = type === "disabled";
  switch (type) {
    case "disabled":
      cls =
        "opacity-30 cursor-not-allowed border border-borderSubtle text-themeTextGray bg-bgSurface";
      break;
    case "danger":
      // Errors, destructive operations, emergency controls — red, reserved.
      cls =
        "bg-statusRed hover:brightness-110 active:brightness-95 text-white border border-statusRed";
      break;
    case "success":
      cls =
        "bg-statusGreen hover:brightness-110 active:brightness-95 text-white border border-statusGreen";
      break;
    case "gradient":
      // Reserved for the single highest-emphasis action on a screen (e.g. a
      // primary "Run"/"Save") — the violet-to-pink accent gradient, not for
      // routine buttons.
      cls =
        "bg-[image:var(--gradient-primary)] hover:brightness-110 active:brightness-95 text-white border border-white/10 shadow-lg shadow-black/30";
      break;
    case "orange":
      cls =
        "bg-themeBlue hover:bg-themeMediumBlue text-white border border-themeBlue";
      break;
    default:
      cls =
        "border border-themeBlue text-themeBlue hover:bg-themeBlue hover:text-white active:bg-themeDarkBlue active:text-white bg-transparent transition-colors";
  }

  return (
    <button
      type="button"
      disabled={disabled}
      className={`min-h-[42px] px-3 py-2 ${cls} flex w-full items-center justify-center gap-2 rounded-xl font-[RobotoMono] text-xs font-semibold`}
      onClick={onBtnClick}
    >
      {children}
    </button>
  );
};

export default Button;
