/* eslint-disable no-unused-vars */
import React, { useState, useEffect } from "react";
import { CircularProgressbar } from "react-circular-progressbar";
import "react-circular-progressbar/dist/styles.css";

const CircularProgressBar = ({
  sensorsData,
  sensorName,
  color,
  text,
  units,
  minValue,
  maxValue,
}) => {
  const [sensorData, setSensorsData] = useState("-");
  const [percentage, setPercentage] = useState(0);

  /**
   * CircularProgressbar prints only true values as text-props.
   * That is why 0 value passed as '0'.
   */
  useEffect(() => {
    let sensorValue = extractSensorValue(sensorsData, sensorName);

    if (sensorValue === 0) {
      setSensorsData("0");
    } else if (sensorValue) {
      setSensorsData(sensorValue);
    } else return;

    if (isNaN(parseFloat(minValue)) || isNaN(parseFloat(maxValue))) {
      console.error("Min or max value is NaN");
      setPercentage(0);
      return;
    }

    const percents = ((sensorValue - minValue) / (maxValue - minValue)) * 100;
    if (percents >= 100) {
      setPercentage(100);
    } else if (percents <= 0) {
      setPercentage(0);
    } else {
      setPercentage(percents);
    }
  }, [maxValue, minValue, sensorName, sensorsData, units]);

  const extractSensorValue = (sensorsData, sensorName) => {
    const parts = sensorsData.split("_");
    if (parts.length === 2 && parts[0] === sensorName) {
      const sensorValue = parseFloat(parts[1]);
      if (!isNaN(sensorValue)) {
        return sensorValue;
      }
    }
  };

  return (
    <div className="flex flex-col items-center justify-between gap-3 font-[RobotoMono]">
      <h3 className="text-center text-lg text-themeBlue">{text}</h3>
      <div className="max-h-[150px] max-w-[150px]">
        <CircularProgressbar
          value={percentage}
          text={sensorData}
          strokeWidth="9"
          styles={{
            path: {
              stroke: color,
              transition: "stroke-dashoffset 0.5s ease 0s",
            },
            text: {
              fill: "#22b7fc",
              fontSize: "16px",
              display: "flex",
              flexDirection: "column",
              flexWrap: "wrap",
            },
          }}
        />
      </div>
    </div>
  );
};

export default CircularProgressBar;
