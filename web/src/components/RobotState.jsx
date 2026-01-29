import React, { useState, useEffect, useCallback, useContext } from "react";
import * as Three from "three";

import { RosContext } from "../app/App";
import { AppConfig } from "../shared/constants/index";

const State = () => {
  const ros = useContext(RosContext);

  const [linear, setLinear] = useState("0.00");
  const [angular, setAngular] = useState("0.00");
  const [xCoord, setXCoord] = useState("0.0");
  const [yCoord, setYCoord] = useState("0.0");
  const [orientation, setOrientation] = useState("0.0");

  const getRobotState = useCallback(() => {
    let pose_sub = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.ROBOT_POSE_TOPIC,
      messageType: "nav_msgs/Odometry",
    });

    pose_sub.subscribe((message) => {
      const positionData = message.pose.pose;
      const velocityData = message.twist.twist;

      setXCoord(positionData.position.x.toFixed(2));
      setYCoord(positionData.position.y.toFixed(2));

      setLinear(velocityData.linear.x.toFixed(2));
      setAngular(velocityData.angular.z.toFixed(2));

      const orientationValue = serializeOrientation(positionData.orientation);
      setOrientation(orientationValue.toFixed(2));
    });
  }, [ros]);

  const serializeOrientation = (orientation) => {
    let q = new Three.Quaternion(
      orientation.x,
      orientation.y,
      orientation.z,
      orientation.w,
    );
    let RPY = new Three.Euler().setFromQuaternion(q);
    return RPY["_z"] * (180 / Math.PI);
  };

  useEffect(() => {
    getRobotState();
  }, [getRobotState]);

  return (
    <div className="flex w-full gap-14">
      <article className="h-[163px] w-full rounded-lg bg-white font-[RobotoMono] ">
        <header className="flex items-center justify-between rounded-t-lg bg-themeBlue px-4 py-2">
          <h2 className="text-2xl text-white">Velocity</h2>
        </header>
        <div className="flex h-[105px] flex-col justify-around overflow-y-auto rounded-b-lg border border-themeBlue px-4 py-3 font-[RobotoMono] text-[20px] text-themeTextGray">
          <p>Linear: {linear} m/s</p>
          <p>Angular: {angular} rad/s </p>
        </div>
      </article>

      <article className="h-[163px] w-full rounded-lg bg-white font-[RobotoMono] ">
        <header className="flex items-center justify-between rounded-t-lg bg-themeBlue px-4 py-2">
          <h2 className="text-2xl text-white">Position</h2>
        </header>
        <div className=" flex h-[105px] flex-col  justify-around overflow-y-auto rounded-b-lg border border-themeBlue px-4 py-3 font-[RobotoMono] text-[20px] text-themeTextGray">
          <p>
            X: {xCoord} Y: {yCoord}
          </p>
          <p>Orientation: {orientation}</p>
        </div>
      </article>
    </div>
  );
};

export default State;
