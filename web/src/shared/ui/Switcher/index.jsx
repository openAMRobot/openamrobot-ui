import React, { useEffect, useState } from "react";
import "./styles.css";

const Switcher = (props) => {
  const { onChange, switcherValue, withoutColor } = props;
  const [isChecked, setIsChecked] = useState(false);

  useEffect(() => {
    setIsChecked(switcherValue);
  }, [switcherValue]);

  const onSwitchHandler = () => {
    onChange(!isChecked);
    setIsChecked(!isChecked);
  };

  return (
    <label className="switch w-[40px] min-w-[40px]">
      <input checked={isChecked} type="checkbox" onChange={onSwitchHandler} />
      <span className={withoutColor ? "sliderWithoutColor" : "slider"} />
    </label>
  );
};

export default Switcher;
