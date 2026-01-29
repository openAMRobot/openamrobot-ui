import React, { useState, useContext } from "react";

import { RosContext } from "../app/App";

import Switcher from "../shared/ui/Switcher";

const ControlSwitcher = ({ text, activeValue, disabledValue, topicName }) => {
  const ros = useContext(RosContext);

  const [switcherValue, setSwitcherValue] = useState(false);
  const peripheryOperation = new window.ROSLIB.Topic({
    ros,
    name: topicName,
    messageType: "std_msgs/String",
  });

  const onSwitcherChange = (value) => {
    setSwitcherValue(value);
    if (value) {
      peripheryOperation.publish(
        new window.ROSLIB.Message({ data: activeValue }),
      );
    } else {
      peripheryOperation.publish(
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
