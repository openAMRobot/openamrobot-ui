import React, { useState } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import Button from "../../shared/ui/Button";

const TimeModal = ({ modalHandler }) => {
  const [hoursValue, setHoursValue] = useState("");
  const [minutesValue, setMinutesValue] = useState("");

  const handleSubmitClick = () => {
    if (!hoursValue && !minutesValue) {
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
    <div className="fixed bottom-0 left-0 right-0 top-0 z-[100] flex items-start justify-center bg-black bg-opacity-80">
      <ToastContainer />

      <section className="relative top-[5vh] flex max-h-[90vh] w-1/2 min-w-[400px] flex-col items-center justify-center gap-3 rounded-[30px] bg-white p-8 font-[RobotoMono] xl:w-1/3">
        <h2 className="bg-themeMediumBlue w-full rounded-lg p-7 text-center text-3xl">
          Enter the time
        </h2>

        <div className="flex gap-4">
          <input
            type="number"
            value={hoursValue}
            className="w-full rounded-lg px-8 py-3 text-xl text-black placeholder:text-themeTextGray"
            style={{ border: "1px solid #696969" }}
            placeholder="hours..."
            onChange={(e) => setHoursValue(e.target.value)}
          />
          <input
            type="number"
            value={minutesValue}
            className="w-full rounded-lg px-8 py-3 text-xl text-black placeholder:text-themeTextGray"
            style={{ border: "1px solid #696969" }}
            placeholder="minutes..."
            onChange={(e) => setMinutesValue(e.target.value)}
          />
        </div>

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

export default TimeModal;
