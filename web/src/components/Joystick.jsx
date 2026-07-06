import React, { useContext, useRef, useEffect, useState } from "react";
import { Joystick } from "react-joystick-component";

import { AppConfig } from "../shared/constants/index";
import { RosContext } from "../app/App";

const JoystickComponent = ({ maxSpeed, compact = false }) => {
  const ros = useContext(RosContext);
  const joysticRef = useRef();
  const [size, setSize] = useState(180);
  const [stickSize, setStickSize] = useState(120);

  const effectiveMax = maxSpeed ?? AppConfig.MAX_LINEAR_SPEED;

  const cmdVel = useRef(null);
  const intervalRef = useRef(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;
    cmdVel.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.CMD_VEL_TOPIC,
      messageType: "geometry_msgs/Twist",
    });
  }, [ros]);

  const setDataToRos = (coordsData) => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    intervalRef.current = setInterval(() => {
      if (cmdVel.current) {
        cmdVel.current.publish(new window.ROSLIB.Message(coordsData));
      }
    }, 100);
  };

  const handleJoysticMove = (evt) => {
    setDataToRos({
      linear: { x: evt.y * effectiveMax, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: -evt.x * AppConfig.MAX_ANGULAR_SPEED },
    });
  };

  const handleStop = () => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    if (cmdVel.current) {
      cmdVel.current.publish(
        new window.ROSLIB.Message({
          linear: { x: 0, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 },
        }),
      );
    }
  };

  useEffect(() => {
    if (!joysticRef.current) return;
    const blockWidth = joysticRef.current.getBoundingClientRect().width;
    setSize(blockWidth);
    setStickSize(blockWidth * 0.66);

    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
    };
  }, []);

  return (
    <div
      ref={joysticRef}
      className={`relative ${
        compact
          ? "h-[82px] w-[82px] lg:h-[88px] lg:w-[88px] 2xl:h-[96px] 2xl:w-[96px]"
          : "h-[100px] w-[100px] lg:h-[140px] lg:w-[140px] 2xl:h-[180px] 2xl:w-[180px]"
      }`}
    >
      <div className="absolute">
        <Joystick
          throttle={150}
          size={size}
          stickSize={stickSize}
          baseColor="#F5F5F5"
          stickColor="#22b7fc"
          move={handleJoysticMove}
          stop={handleStop}
        />
      </div>
    </div>
  );
};

export default JoystickComponent;
