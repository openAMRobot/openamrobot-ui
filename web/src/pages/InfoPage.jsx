/* eslint-disable no-unused-vars */
import React, { useRef, useContext, useState, useEffect } from "react";

import Camera from "../components/Camera";
import Logs from "../components/Logs";
import CircularProgress from "../components/CircularProgressBar";

import RobotState from "../components/RobotState";
import { RosContext } from "../app/App";
import { AppConfig } from "../shared/constants/index";

const InfoPage = () => {
  const ros = useContext(RosContext);
  const [sensorsData, setSensorsData] = useState("");
  const [batteryData, setBatteryData] = useState("");
  const [charge, setCharge] = useState(false);

  const sensorsTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.SENSORS_TOPIC,
      messageType: "std_msgs/String",
    }),
  );

  const batteryTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.BATTERY_TOPIC,
      messageType: "sensor_msgs/BatteryState",
    }),
  );

  const chargeTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.CHARGE_STATION_CONNECTED,
      messageType: "std_msgs/Bool",
    }),
  );

  useEffect(() => {
    sensorsTopic.current.subscribe(({ data }) => {
      setSensorsData(data);
    });

    batteryTopic.current.subscribe((data) => {
      const batteryData = data.percentage;
      setBatteryData(batteryData);
    });

    chargeTopic.current.subscribe(({ data }) => {
      setCharge(data);
    });
  }, []);

  return (
    <div className="items-stratch relative flex min-h-[70vh] flex-col gap-[30px] pt-[30px] xl:flex-row xl:gap-[7%]">
      <section className="color-white flex w-full flex-col items-start justify-center gap-[30px] xl:w-3/5">
        <h3 className="w-full text-center font-[RobotoMono] text-3xl font-bold text-themeBlue">
          Camera
        </h3>
        <div className="h-[450px] w-full">
          <Camera />
        </div>
        <div className="mt-auto w-full">
          <RobotState />
        </div>
      </section>

      <section className="color-white flex w-full flex-col items-center justify-center gap-[30px] xl:w-2/5">
        <h3 className="w-full text-center font-[RobotoMono] text-3xl font-bold text-themeBlue">
          Information
        </h3>

        <div className="grid grid-cols-4 grid-rows-2 gap-x-[30px] gap-y-2 2xl:gap-x-[60px]">
          <CircularProgress
            sensorsData={sensorsData}
            sensorName="batt1"
            color="##ff7a00"
            text="Battery 1, %"
            units="%"
            minValue={0}
            maxValue={100}
          />
          <CircularProgress
            sensorsData={sensorsData}
            sensorName="batt2"
            color="#00e1ff"
            text="Battery 2, %"
            units="%"
            minValue={0}
            maxValue={100}
          />
          <CircularProgress
            sensorsData={sensorsData}
            sensorName="sens3"
            color="#00ffb0"
            text="Sensor 3"
            units=""
            minValue={0}
            maxValue={100}
          />
          <CircularProgress
            sensorsData={sensorsData}
            sensorName="sens4"
            color="#c5cc02"
            text="Sensor 4"
            units=""
            minValue={0}
            maxValue={100}
          />
          <CircularProgress
            sensorsData={sensorsData}
            sensorName="sens5"
            color="#d96cff"
            text="Sensor 5"
            units=""
            minValue={0}
            maxValue={100}
          />
          <CircularProgress
            sensorsData={sensorsData}
            sensorName="sens6"
            color="#FFC0CB"
            text="Sensor 6"
            units=""
            minValue={0}
            maxValue={100}
          />
          <CircularProgress
            sensorsData={sensorsData}
            sensorName="temp1"
            color="#00a3ff"
            text="Temp 1, ℃"
            units="℃"
            minValue={0}
            maxValue={40}
          />
          <CircularProgress
            sensorsData={sensorsData}
            sensorName="temp2"
            color="#ff48ed"
            text="Temp 2, ℃"
            units="℃"
            minValue={0}
            maxValue={40}
          />
        </div>

        <h3 className="w-full text-center font-[RobotoMono] text-2xl font-bold ">
          {charge && (
            <span className="text-themeDarkBlue">
              Connected to the charging station
            </span>
          )}
          {!charge && (
            <span className="text-white">
              Not connected to the charging station
            </span>
          )}
        </h3>

        <div className="mt-auto h-[240px] w-full">
          <Logs />
        </div>
      </section>
    </div>
  );
};

export default InfoPage;
