import React, { useRef, useContext, useState, useEffect } from "react";

import Camera from "../components/Camera";
import RobotState from "../components/RobotState";
import SystemHealth from "../components/SystemHealth";
import { RosContext } from "../app/App";
import { AppConfig } from "../shared/constants/index";

const InfoPage = () => {
  const ros = useContext(RosContext);
  const [batteryPct, setBatteryPct] = useState(null);
  const [charging, setCharging] = useState(false);

  const batteryTopic = useRef(null);
  const chargeTopic = useRef(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    batteryTopic.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.BATTERY_TOPIC,
      messageType: "std_msgs/Float32",
    });

    chargeTopic.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.CHARGE_STATION_CONNECTED,
      messageType: "std_msgs/Bool",
    });

    // battery.py publishes percentage as a plain float (0–100)
    batteryTopic.current.subscribe(({ data }) => {
      setBatteryPct(Math.round(data));
    });

    chargeTopic.current.subscribe(({ data }) => {
      setCharging(data);
    });

    return () => {
      batteryTopic.current?.unsubscribe();
      chargeTopic.current?.unsubscribe();
    };
  }, [ros]);

  const batteryColor =
    batteryPct === null
      ? "bg-themeTextGray"
      : batteryPct > 60
      ? "bg-statusGreen"
      : batteryPct > 25
      ? "bg-statusYellow"
      : "bg-statusRed";

  return (
    <div className="sectionHeight flex flex-col gap-5 pb-4 pt-5 xl:flex-row xl:gap-6">
      {/* Left: Camera + Robot state */}
      <section className="flex w-full flex-col gap-4 xl:w-3/5">
        <h3 className="font-[RobotoMono] text-lg font-semibold uppercase tracking-wider text-themeBlue">
          Camera Feed
        </h3>
        <div className="h-[380px] w-full xl:h-[420px]">
          <Camera />
        </div>
        <RobotState />
      </section>

      {/* Right: Status panels */}
      <section className="flex w-full flex-col gap-4 xl:w-2/5">
        <h3 className="font-[RobotoMono] text-lg font-semibold uppercase tracking-wider text-themeBlue">
          System Status
        </h3>

        {/* Battery */}
        <div className="rounded-xl border border-borderSubtle bg-bgCard p-4 font-[RobotoMono]">
          <p className="mb-3 text-xs uppercase tracking-wider text-themeTextGray">
            Battery
          </p>
          {batteryPct !== null ? (
            <div className="space-y-2">
              <div className="flex items-center justify-between text-sm">
                <span className="text-textWhiteHover">{batteryPct}%</span>
                <span
                  className={`text-xs ${
                    batteryPct > 60
                      ? "text-statusGreen"
                      : batteryPct > 25
                      ? "text-statusYellow"
                      : "text-statusRed"
                  }`}
                >
                  {batteryPct > 60
                    ? "Good"
                    : batteryPct > 25
                    ? "Low"
                    : "Critical"}
                </span>
              </div>
              <div className="h-2 w-full overflow-hidden rounded-full bg-bgSurface">
                <div
                  className={`h-full rounded-full transition-all ${batteryColor}`}
                  style={{ width: `${batteryPct}%` }}
                />
              </div>
            </div>
          ) : (
            <p className="text-sm text-themeTextGray opacity-50">
              No battery data
            </p>
          )}
        </div>

        {/* Charging status */}
        <div className="rounded-xl border border-borderSubtle bg-bgCard p-4 font-[RobotoMono]">
          <p className="mb-2 text-xs uppercase tracking-wider text-themeTextGray">
            Charging Station
          </p>
          <div className="flex items-center gap-2">
            <span
              className={`h-2.5 w-2.5 rounded-full ${
                charging ? "bg-statusGreen" : "bg-themeTextGray"
              }`}
            />
            <span
              className={`text-sm ${
                charging ? "text-statusGreen" : "text-themeTextGray"
              }`}
            >
              {charging ? "Connected" : "Not connected"}
            </span>
          </div>
        </div>

        {/* System health */}
        <SystemHealth />
      </section>
    </div>
  );
};

export default InfoPage;
