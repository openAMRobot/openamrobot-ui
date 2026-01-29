import React, { useContext, useRef, useEffect, useState } from "react";
import { Joystick } from "react-joystick-component";

import { AppConfig } from "../shared/constants/index";
import { RosContext } from "../app/App";

let interval = null;

const JoystickComponent = () => {
  const ros = useContext(RosContext);
  const joysticRef = useRef();
  const [size, setSize] = useState(180);
  const [stickSize, setStickSize] = useState(120);

  const cmdVel = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.CMD_VEL_TOPIC,
      messageType: "geometry_msgs/Twist",
    }),
  );

  const setDataToRos = (coordsData) => {
    clearInterval(interval);
    interval = null;

    if (!interval) {
      interval = setInterval(() => {
        const twist_msg = new window.ROSLIB.Message(coordsData);
        cmdVel.current.publish(twist_msg);
      }, 100);
    }
  };

  const handleJoysticMove = (evt) => {
    let x = evt.y * AppConfig.MAX_LINEAR_SPEED;
    let z = -evt.x * AppConfig.MAX_LINEAR_SPEED;

    setDataToRos({
      linear: {
        x,
        y: 0,
        z: 0,
      },
      angular: {
        x: 0,
        y: 0,
        z,
      },
    });
  };
  const handleStop = () => {
    clearInterval(interval);
    interval = null;
    const zeroCoords = {
      linear: {
        x: 0,
        y: 0,
        z: 0,
      },
      angular: {
        x: 0,
        y: 0,
        z: 0,
      },
    };
    const twist_msg = new window.ROSLIB.Message(zeroCoords);
    cmdVel.current.publish(twist_msg);
  };

  useEffect(() => {
    const blockWidth = joysticRef.current.getBoundingClientRect().width;
    setSize(blockWidth);
    setStickSize(blockWidth * 0.66);
  }, []);

  return (
    <div
      ref={joysticRef}
      className="relative h-[100px] w-[100px] lg:h-[140px] lg:w-[140px] 2xl:h-[180px] 2xl:w-[180px]"
    >
      <div className="absolute">
        <Joystick
          throttle={150}
          size={size}
          stickSize={stickSize}
          baseColor="#F5F5F5"
          stickColor="#22b7fc"
          move={(evt) => handleJoysticMove(evt)}
          stop={handleStop}
        />
      </div>
    </div>
  );
};

export default JoystickComponent;
