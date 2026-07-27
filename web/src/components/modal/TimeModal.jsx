import React, { useState } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import Button from "../../shared/ui/Button";

const TimeModal = ({ modalHandler }) => {
  const [hoursValue, setHoursValue] = useState(0);
  const [minutesValue, setMinutesValue] = useState(0);

  const handleSubmitClick = () => {
    if (hoursValue === "" && minutesValue === "") {
      toast.warn("Enter hours and minutes");
      return;
    }

    if (hoursValue < 0 || hoursValue > 23) {
      toast.warn("Hours can't be less then 0 and more then 23");
      return;
    }

    if (minutesValue < 0 || minutesValue > 59) {
      toast.warn("Minutes can't be less then 0 and more then 59");
      return;
    }

    let dataToSend = {
      hours: Number(hoursValue),
      minutes: Number(minutesValue),
    };

    modalHandler(dataToSend);
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
        aria-labelledby="time-dialog-title"
        className="modal-surface flex max-h-[90vh] w-full max-w-lg flex-col gap-4 p-5 font-[RobotoMono] sm:p-6"
      >
        <h2
          id="time-dialog-title"
          className="text-xl font-bold text-textWhiteHover"
        >
          Waypoint wait time
        </h2>

        <p className="text-sm leading-6 text-themeTextGray">
          How long should the robot pause here before continuing? Leave both
          at 0 for no pause.
        </p>

        <div className="grid grid-cols-1 gap-3 sm:grid-cols-2">
          <input
            type="number"
            value={hoursValue}
            aria-label="Hours"
            className="min-h-[46px] w-full rounded-xl border border-borderSubtle bg-bgSurface px-4 py-3 text-base text-textWhiteHover placeholder:text-themeTextGray focus:border-themeBlue"
            placeholder="hours..."
            onChange={(e) => setHoursValue(e.target.value)}
          />
          <input
            type="number"
            value={minutesValue}
            aria-label="Minutes"
            className="min-h-[46px] w-full rounded-xl border border-borderSubtle bg-bgSurface px-4 py-3 text-base text-textWhiteHover placeholder:text-themeTextGray focus:border-themeBlue"
            placeholder="minutes..."
            onChange={(e) => setMinutesValue(e.target.value)}
          />
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

export default TimeModal;
