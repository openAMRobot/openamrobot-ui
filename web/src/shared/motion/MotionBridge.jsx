import { useEffect, useRef } from "react";

import { useRos, useRosStatus } from "../../app/App";
import { addEvent } from "../events/eventLog";
import { abortUiMotion } from "./abortBus";
import {
  resetMotionGate,
  setClockSource,
  setClockStatus,
  setConnection,
  setLockState,
} from "./motionGate";
import { createRobotClock } from "./robotClock";
import { createStopLock, LOCK_RELEASED } from "./stopLock";
import { attachRos, releaseAll } from "./uiCommand";

/**
 * Headless, mounted once in AppLayout. Connects the shared ROS connection to
 * the motion gate: per connection it creates the stop lock subscription and
 * the robot clock, and drops both on disconnect, so a reconnect always starts
 * from an unknown lock and an unknown clock.
 *
 * A lock that is locked or unknown aborts running programs and missions and
 * drops any held manual command.
 */
const MotionBridge = () => {
  const ros = useRos();
  const rosStatus = useRosStatus();
  const lastLockRef = useRef(null);

  useEffect(() => {
    attachRos(ros);
  }, [ros]);

  useEffect(() => {
    resetMotionGate(rosStatus === "connected" ? "connected" : "disconnected");
    lastLockRef.current = null;
    // Disconnected is the same as an unknown lock: nothing may keep running.
    releaseAll();
    abortUiMotion("rosbridge disconnected or reconnecting");
    if (rosStatus !== "connected" || !ros || !window.ROSLIB) return undefined;

    const lock = createStopLock({
      ros,
      onChange: (state) => {
        setLockState(state);
        if (state !== LOCK_RELEASED) {
          releaseAll();
          abortUiMotion(state === "locked" ? "Stopped (software)" : "Stop lock state unknown");
        }
        if (lastLockRef.current !== null && lastLockRef.current !== state) {
          addEvent({
            type: "safety",
            severity: state === LOCK_RELEASED ? "info" : "warning",
            message: `Stop lock is now ${state}`,
          });
        }
        lastLockRef.current = state;
      },
    });
    const clock = createRobotClock({ ros, onChange: setClockStatus });
    setClockSource(clock);
    setClockStatus(clock.getStatus());

    return () => {
      lock.stop();
      clock.stop();
      resetMotionGate("disconnected");
    };
  }, [ros, rosStatus]);

  return null;
};

export default MotionBridge;
