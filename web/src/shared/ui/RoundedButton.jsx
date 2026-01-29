import React from "react";

const RoundedButton = ({ type, children, onBtnClick }) => {
  let buttonColors = "";
  switch (type) {
    case "success":
      buttonColors = "bg-[#17A700] hover:bg-[#117600] active:bg-[#0D6000]";
      break;
    case "danger":
      buttonColors = "bg-[#CD3100] hover:bg-[#A02700] active:bg-[#741500]";
      break;
  }
  return (
    <button
      className={`${buttonColors} flex h-[120px] w-[120px] items-center justify-center rounded-full font-[RobotoMono] text-[40px] text-white lg:h-[140px] lg:w-[140px] 2xl:h-[180px] 2xl:w-[180px]`}
      onClick={onBtnClick}
    >
      {children}
    </button>
  );
};
export default RoundedButton;
