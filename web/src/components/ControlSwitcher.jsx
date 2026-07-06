import React, { useState, useContext, useRef, useEffect } from "react";

import { RosContext } from "../app/App";

import Switcher from "../shared/ui/Switcher";

const ControlSwitcher = ({ text, activeValue, disabledValue, topicName }) => {
  const ros = useContext(RosContext);

  const [switcherValue, setSwitcherValue] = useState(false);
  const peripheryOperation = useRef(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;
    peripheryOperation.current = new window.ROSLIB.Topic({
      ros,
      name: topicName,
      messageType: "std_msgs/String",
    });
  }, [ros, topicName]);

  const onSwitcherChange = (value) => {
    setSwitcherValue(value);
    if (!peripheryOperation.current) return;
    if (value) {
      peripheryOperation.current.publish(
        new window.ROSLIB.Message({ data: activeValue }),
      );
    } else {
      peripheryOperation.current.publish(
        new window.ROSLIB.Message({ data: disabledValue }),
      );
    }
  };
  return (
    <div className="flex items-center justify-between gap-5">
      <span>{text}</span>
      <Switcher onChange={onSwitcherChange} switcherValue={switcherValue} />
    </div>
  );
};

export default ControlSwitcher;
