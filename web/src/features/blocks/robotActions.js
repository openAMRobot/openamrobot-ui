import { AppConfig } from "../../shared/constants";
import { navGoalBlockedReason } from "../../shared/motion/navGoalGate";
import { holdVelocity, releaseVelocity } from "../../shared/motion/uiCommand";

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

const poseMessage = ({ x, y, yaw }) =>
  new window.ROSLIB.Message({
    header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
    pose: {
      position: { x, y, z: 0 },
      orientation: {
        x: 0,
        y: 0,
        z: Math.sin(yaw / 2),
        w: Math.cos(yaw / 2),
      },
    },
  });

// Blockly drive/rotate/stop blocks and voice-planned drive actions (which
// arrive here as Blockly plans) are manual velocity sources: they go through
// shared/motion/uiCommand.js like the joystick, never a topic of their own.
const PROGRAM_SOURCE = "program";

const holdOrThrow = (command) => {
  const { ok, reason } = holdVelocity(PROGRAM_SOURCE, command);
  if (!ok) throw new Error(`Manual drive is disabled: ${reason}`);
};

const assertSafeSpeed = ({ linear = 0, angular = 0 }) => {
  if (Math.abs(linear) > AppConfig.MAX_LINEAR_SPEED) {
    throw new Error(
      `Linear speed ${linear} m/s exceeds the ${AppConfig.MAX_LINEAR_SPEED} m/s limit`,
    );
  }

  if (Math.abs(angular) > AppConfig.MAX_ANGULAR_SPEED) {
    throw new Error(
      `Angular speed ${angular} rad/s exceeds the ${AppConfig.MAX_ANGULAR_SPEED} rad/s limit`,
    );
  }
};

export const createRobotActionClient = (ros) => {
  const goalPoseTopic = new window.ROSLIB.Topic({
    ros,
    name: AppConfig.GOAL_POSE_TOPIC,
    messageType: "geometry_msgs/PoseStamped",
  });

  const dockTopic = new window.ROSLIB.Topic({
    ros,
    name: AppConfig.DOCK_TRIGGER_TOPIC,
    messageType: "std_msgs/Bool",
  });

  const undockTopic = new window.ROSLIB.Topic({
    ros,
    name: AppConfig.UNDOCK_TRIGGER_TOPIC,
    messageType: "std_msgs/Bool",
  });

  const navStatusTopic = new window.ROSLIB.Topic({
    ros,
    name: AppConfig.NAV_STATUS_TOPIC,
    messageType: "action_msgs/GoalStatusArray",
  });

  const batteryTopic = new window.ROSLIB.Topic({
    ros,
    name: AppConfig.BATTERY_TOPIC,
    messageType: "std_msgs/Float32",
  });

  const operationTopic = new window.ROSLIB.Topic({
    ros,
    name: AppConfig.UI_OPERATION_TOPIC,
    messageType: "std_msgs/String",
  });

  const uiMessageTopic = new window.ROSLIB.Topic({
    ros,
    name: AppConfig.UI_MESSAGE_TOPIC,
    messageType: "std_msgs/String",
  });

  const cancelClient = new window.ROSLIB.Service({
    ros,
    name: AppConfig.NAV_CANCEL_GOAL_SERVICE,
    serviceType: "action_msgs/CancelGoal",
  });

  const cancelNavigation = () =>
    cancelClient.callService(
      new window.ROSLIB.ServiceRequest({
        goal_info: {
          goal_id: { uuid: new Array(16).fill(0) },
          stamp: { sec: 0, nanosec: 0 },
        },
      }),
    );

  // Program "stop" block and the Blocks page Stop button: end the program's
  // manual command (one zero on /ui/cmd_vel) and cancel navigation. This is
  // not Stop (software): it does not engage the stop lock.
  const stopProgramMotion = () => {
    releaseVelocity(PROGRAM_SOURCE);
    cancelNavigation();
  };

  const stopMovement = () => {
    releaseVelocity(PROGRAM_SOURCE);
  };

  const waitForNavigationComplete = (timeoutSeconds) =>
    new Promise((resolve, reject) => {
      const timeout = setTimeout(() => {
        navStatusTopic.unsubscribe(listener);
        reject(
          new Error(`Navigation did not finish within ${timeoutSeconds}s`),
        );
      }, timeoutSeconds * 1000);

      const listener = (msg) => {
        if (!msg.status_list?.length) return;
        const latest = msg.status_list[msg.status_list.length - 1];

        if (latest.status === 4) {
          clearTimeout(timeout);
          navStatusTopic.unsubscribe(listener);
          resolve();
        } else if (latest.status === 5 || latest.status === 6) {
          clearTimeout(timeout);
          navStatusTopic.unsubscribe(listener);
          reject(new Error("Navigation was canceled or failed"));
        }
      };

      navStatusTopic.subscribe(listener);
    });

  const readBatteryPercent = () =>
    new Promise((resolve) => {
      const timeout = setTimeout(() => {
        batteryTopic.unsubscribe(listener);
        resolve(null);
      }, 1500);

      const listener = (msg) => {
        clearTimeout(timeout);
        batteryTopic.unsubscribe(listener);
        const value = Number(msg.data);
        resolve(Number.isFinite(value) ? value : null);
      };

      batteryTopic.subscribe(listener);
    });

  const runActions = async (actions, options) => {
    for (const action of actions) {
      if (options?.shouldStop?.()) break;
      await actionClient.runAction(action, options);
    }
  };

  const actionClient = {
    async runAction(action, options) {
      switch (action.type) {
        case "navigate": {
          // No goal while the stop lock is engaged or unknown.
          const blocked = navGoalBlockedReason();
          if (blocked) throw new Error(`Navigation goal not sent: ${blocked}`);
          goalPoseTopic.publish(poseMessage(action));
          return;
        }
        case "wait":
          await sleep(action.seconds * 1000);
          return;
        case "set_speed":
          // Held at 20 Hz until a stop block, another drive block, the end
          // of the program or an abort.
          assertSafeSpeed(action);
          holdOrThrow({ linear: action.linear, angular: action.angular });
          return;
        case "drive_for":
          assertSafeSpeed({ linear: action.linear });
          holdOrThrow({ linear: action.linear, angular: 0 });
          await sleep(action.seconds * 1000);
          stopMovement();
          return;
        case "rotate_for":
          assertSafeSpeed({ angular: action.angular });
          holdOrThrow({ linear: 0, angular: action.angular });
          await sleep(action.seconds * 1000);
          stopMovement();
          return;
        case "stop_movement":
          stopMovement();
          return;
        case "wait_nav_complete":
          await waitForNavigationComplete(action.timeout);
          return;
        case "repeat":
          for (let count = 0; count < action.times; count += 1) {
            if (options?.shouldStop?.()) break;
            await runActions(action.actions, options);
          }
          return;
        case "log":
          console.info(`[Blockly] ${action.message}`);
          uiMessageTopic.publish(
            new window.ROSLIB.Message({ data: action.message }),
          );
          return;
        case "battery_below": {
          const batteryPercent = await readBatteryPercent();
          if (batteryPercent !== null && batteryPercent < action.percent) {
            await runActions(action.actions, options);
          }
          return;
        }
        case "set_mode":
          operationTopic.publish(
            new window.ROSLIB.Message({ data: action.mode }),
          );
          return;
        case "dock": {
          const blocked = navGoalBlockedReason();
          if (blocked) throw new Error(`Dock not started: ${blocked}`);
          dockTopic.publish(new window.ROSLIB.Message({ data: true }));
          return;
        }
        case "undock": {
          const blocked = navGoalBlockedReason();
          if (blocked) throw new Error(`Undock not started: ${blocked}`);
          undockTopic.publish(new window.ROSLIB.Message({ data: true }));
          return;
        }
        case "stop":
          stopProgramMotion();
          return;
        default:
          throw new Error(`Unknown action: ${action.type}`);
      }
    },
    stop: stopProgramMotion,
  };

  return {
    runAction: actionClient.runAction,
    stop: stopProgramMotion,
    stopMovement,
  };
};

export const executeRobotPlan = async (
  ros,
  plan,
  { shouldStop, onStep, onStepStart, onStepSuccess, onStepError } = {},
) => {
  if (!ros || !window.ROSLIB) {
    throw new Error("ROSBridge is not available");
  }

  const client = createRobotActionClient(ros);

  try {
    for (let index = 0; index < plan.length; index += 1) {
      if (shouldStop?.()) break;
      const action = plan[index];
      onStep?.(index, action);
      onStepStart?.(index, action);
      try {
        await client.runAction(action, { shouldStop });
        onStepSuccess?.(index, action);
      } catch (error) {
        onStepError?.(index, action, error);
        throw error;
      }
    }
  } finally {
    // A program never leaves a manual command held after it ends.
    client.stopMovement();
  }

  return client;
};
