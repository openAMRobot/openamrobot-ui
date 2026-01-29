import React, { useContext, useRef } from "react";

import { AppConfig } from "../shared/constants/index";
import { RosContext } from "../app/App";

import Map from "../components/Map";
import Logs from "../components/Logs";
import ControlSwitcher from "../components/ControlSwitcher";

import Button from "../shared/ui/Button";
import RoundedButton from "../shared/ui/RoundedButton";

const ControlPage = () => {
  const ros = useContext(RosContext);

  const uiOperationTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.UI_OPERATION_TOPIC,
      messageType: "std_msgs/String",
    }),
  );

  const onControlBtnClick = (message) => {
    uiOperationTopic.current.publish(
      new window.ROSLIB.Message({ data: message }),
    );
  };

  const onChargeBtnClick = (message) => {
    toast.warn("Sending the robot to the charging station");
  };

  const onPeripheryClickHandler = (message) => {
    const peripheryOperation = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.UI_OPERATION,
      messageType: "std_msgs/String",
    });
    peripheryOperation.publish(new window.ROSLIB.Message({ data: message }));
  };

  return (
    <div className="sectionHeight flex flex-col items-stretch gap-10 pb-6 pt-[30px] xl:flex-row">
      <section className="color-white mb-auto flex w-full flex-col justify-center xl:w-[65%]">
        <h3 className="w-full text-center font-[RobotoMono] text-3xl font-bold text-white">
          Control
        </h3>
        <div className="mt-[30px] flex w-full flex-1 items-start justify-evenly gap-10 xl:justify-between">
          <div className="flex flex-1 flex-grow flex-col gap-4 2xl:gap-24">
            <div className="grid w-full grid-cols-1 flex-col items-start justify-between gap-10">
              <Button onBtnClick={() => onControlBtnClick("follow_route")}>
                <span className="mx-auto">Follow</span>
              </Button>
              <Button onBtnClick={() => onControlBtnClick("next_point")}>
                <span className="mx-auto">Next point</span>
              </Button>
              <Button onBtnClick={() => onControlBtnClick("previous_point")}>
                <span className="mx-auto">Prev point</span>
              </Button>
              <Button size="big" onBtnClick={() => onControlBtnClick("home")}>
                <span className="mx-auto">Home</span>
              </Button>
              <Button onBtnClick={() => onChargeBtnClick("charge")}>
                <span className="mx-auto">Charge</span>
              </Button>
              <Button
                onBtnClick={() => onPeripheryClickHandler("function_1_1")}
              >
                <span className="mx-auto">Function 1</span>
              </Button>
              <Button
                onBtnClick={() => onPeripheryClickHandler("function_2_1")}
              >
                <span className="mx-auto">Function 2</span>
              </Button>
            </div>
          </div>

          <div className="text-themeDarkBlue flex flex-1 flex-col justify-between gap-10 font-[RobotoMono] text-2xl">
            <ControlSwitcher
              topicName={AppConfig.UI_OPERATION}
              text={"Function 3"}
              activeValue={"function_3_1"}
              disabledValue={"function_3_2"}
            />
            <ControlSwitcher
              topicName={AppConfig.UI_OPERATION}
              text={"Function 4"}
              activeValue={"function_4_1"}
              disabledValue={"function_4_2"}
            />
            <ControlSwitcher
              topicName={AppConfig.UI_OPERATION}
              text={"Function 5"}
              activeValue={"function_5_1"}
              disabledValue={"function_5_2"}
            />
            <ControlSwitcher
              topicName={AppConfig.UI_OPERATION}
              text={"Function 6"}
              activeValue={"function_6_1"}
              disabledValue={"function_6_2"}
            />
            <ControlSwitcher
              topicName={AppConfig.UI_OPERATION}
              text={"Function 7"}
              activeValue={"function_7_1"}
              disabledValue={"function_7_2"}
            />
            <ControlSwitcher
              topicName={AppConfig.UI_OPERATION}
              text={"Function 8"}
              activeValue={"function_8_1"}
              disabledValue={"function_8_2"}
            />
            <ControlSwitcher
              topicName={AppConfig.UI_OPERATION}
              text={"Function 9"}
              activeValue={"function_9_1"}
              disabledValue={"function_9_2"}
            />
          </div>

          <div className="hidden flex-col justify-between gap-[100px] xl:flex">
            <RoundedButton
              type="success"
              onBtnClick={() => onControlBtnClick("start")}
            >
              Start
            </RoundedButton>
            <RoundedButton
              type="danger"
              onBtnClick={() => onControlBtnClick("stop")}
            >
              Stop
            </RoundedButton>
          </div>
        </div>
      </section>

      <section className="color-white mb-auto mt-6 flex w-full flex-col items-center justify-center gap-7 xl:mt-0 xl:w-[40%]">
        <h3 className="w-full text-center font-[RobotoMono] text-3xl font-bold text-white">
          Map
        </h3>
        <div className="h-[400px] w-full">
          <Map />
        </div>

        <div className="h-[165px] w-full">
          <Logs />
        </div>

        <div className="flex justify-between gap-[90px] xl:hidden">
          <RoundedButton
            type="success"
            onBtnClick={() => onControlBtnClick("start")}
          >
            Start
          </RoundedButton>
          <RoundedButton
            type="danger"
            onBtnClick={() => onControlBtnClick("stop")}
          >
            Stop
          </RoundedButton>
        </div>
      </section>
    </div>
  );
};

export default ControlPage;
