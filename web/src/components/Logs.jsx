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

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;
    const messageTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.UI_MESSAGE_TOPIC,
      messageType: "std_msgs/String",
    });
    messageTopic.subscribe(({ data }) => {
      const currentTime = moment().format("HH:mm:ss");
      dispatch(addSingleLog(`${currentTime}: ${data}`));
    });
    return () => messageTopic.unsubscribe();
  }, [ros, dispatch]);

  return (
    <article className="flex h-full w-full flex-col overflow-hidden rounded-xl border border-borderSubtle bg-bgCard font-[RobotoMono]">
      <header className="flex items-center justify-between border-b border-borderSubtle bg-bgSurface px-4 py-2">
        <h2 className="text-sm font-semibold uppercase tracking-wider text-themeBlue">
          Messages
        </h2>
        <button
          className="rounded-lg border border-borderSubtle px-3 py-1 text-xs text-themeTextGray transition-colors hover:border-themeBlue hover:text-themeBlue"
          onClick={() => dispatch(setLogs([]))}
        >
          Clear
        </button>
      </header>
      <ul className="flex flex-1 flex-col-reverse gap-0.5 overflow-y-auto px-3 py-2 text-xs text-themeTextGray">
        {logs.length === 0 ? (
          <li className="flex h-full items-center justify-center text-themeTextGray opacity-50">
            No messages
          </li>
        ) : (
          logs.map((msgItem, i) => (
            <li
              key={i}
              className="border-b border-borderSubtle/30 py-0.5 last:border-0"
            >
              {msgItem}
            </li>
          ))
        )}
      </ul>
    </article>
  );
};

export default RobotLog;
