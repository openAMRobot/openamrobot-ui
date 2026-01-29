import React, { useContext, useRef, useEffect } from "react";
import { useDispatch, useSelector } from "react-redux";
import moment from "moment";

import { AppConfig } from "../shared/constants/index";
import { RosContext } from "../app/App";
import { logsSelector, setLogs, addSingleLog } from "../stores";

const RobotLog = () => {
  const ros = useContext(RosContext);
  const dispatch = useDispatch();
  const { logs } = useSelector(logsSelector);

  const messageTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.UI_MESSAGE_TOPIC,
      messageType: "std_msgs/String",
    }),
  );
  useEffect(() => {
    console.log("Logs component update");

    const currentMessageTopic = messageTopic.current;
    currentMessageTopic.subscribe(({ data }) => {
      console.log("DATA LOGS", data);
      console.log("OLD LOGS", logs);
      const currentTime = moment().format("HH:mm:ss");
      const message = `${currentTime}: ${data}`;
      // const newMsgList = [message, ...logs];
      dispatch(addSingleLog(message));
    });
    return () => currentMessageTopic.unsubscribe();
  }, [dispatch]);

  const Messages = () => {
    if (logs.length === 0) {
      return (
        <>
          <li className="flex h-full items-center justify-center text-center font-[RobotoMono] text-themeTextGray">
            Console is empty
          </li>
        </>
      );
    }

    const messageItems = logs.map((msgItem) => {
      return <li key={Math.random()}>{msgItem}</li>;
    });
    return messageItems;
  };

  const onClearClick = () => {
    dispatch(setLogs([]));
  };

  // eslint-disable-next-line no-unused-vars
  const onAddNewMessage = () => {
    const currentTime = moment().format("HH:mm:ss");
    const message = `${currentTime}: ${currentTime}`;
    const newMsgList = [message, ...logs];

    dispatch(setLogs(newMsgList));
  };

  return (
    <article className="flex h-full w-full flex-col rounded-lg bg-white font-[RobotoMono]">
      <header className="flex items-center justify-between rounded-t-lg bg-themeBlue px-4">
        <h2 className="text-lg text-white xl:text-xl">Messages</h2>
        <button
          className="my-2 rounded-lg bg-white px-[10px] py-[7px] text-lg text-themeDarkBlue hover:bg-themeLightGray active:bg-themeDarkBlue active:text-white xl:text-xl"
          onClick={onClearClick}
        >
          Clear
        </button>
        {/* <button
          className="my-2 rounded-lg bg-white px-[10px] py-[7px] text-lg hover:bg-themeLightGray active:bg-themeTextGray active:text-white xl:text-xl"
          onClick={onAddNewMessage}
        >
          Add
        </button> */}
      </header>
      <ul className="flex flex-1 flex-col-reverse overflow-y-auto rounded-b-lg border border-themeBlue px-4 py-3">
        <Messages />
      </ul>
    </article>
  );
};

export default RobotLog;
