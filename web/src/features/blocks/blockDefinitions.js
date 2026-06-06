import * as Blockly from "blockly";

export const OPEN_AMR_LOCATIONS = {
  Home: { x: 0, y: 0, yaw: 0 },
  "Charging Station": { x: 0.5, y: 0, yaw: 0 },
  "Pickup Point": { x: 2, y: 1, yaw: 1.57 },
  "Dropoff Point": { x: 0, y: 2, yaw: 3.14 },
};

const locationOptions = Object.keys(OPEN_AMR_LOCATIONS).map((name) => [
  name,
  name,
]);

export const registerOpenAmrBlocks = () => {
  if (Blockly.Blocks.openamr_start) return;

  Blockly.common.defineBlocksWithJsonArray([
    {
      type: "openamr_start",
      message0: "start robot program",
      nextStatement: null,
      colour: "#0e9fbc",
      tooltip: "Program entry point. Connect robot actions below this block.",
    },
    {
      type: "openamr_navigate",
      message0: "navigate to x %1 y %2 yaw %3",
      args0: [
        { type: "field_number", name: "X", value: 0, precision: 0.01 },
        { type: "field_number", name: "Y", value: 0, precision: 0.01 },
        { type: "field_number", name: "YAW", value: 0, precision: 0.01 },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#087ea4",
      tooltip: "Publish a goal pose in the map frame. Yaw is in radians.",
    },
    {
      type: "openamr_wait",
      message0: "wait %1 seconds",
      args0: [
        {
          type: "field_number",
          name: "SECONDS",
          value: 2,
          min: 0,
          precision: 0.1,
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#516173",
      tooltip: "Pause before running the next robot action.",
    },
    {
      type: "openamr_set_speed",
      message0: "set speed linear %1 angular %2",
      args0: [
        { type: "field_number", name: "LINEAR", value: 0.1, precision: 0.01 },
        { type: "field_number", name: "ANGULAR", value: 0, precision: 0.01 },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#3f7c9b",
      tooltip: "Publish a Twist message to cmd_vel.",
    },
    {
      type: "openamr_drive_for",
      message0: "drive linear speed %1 for %2 seconds",
      args0: [
        { type: "field_number", name: "LINEAR", value: 0.1, precision: 0.01 },
        {
          type: "field_number",
          name: "SECONDS",
          value: 2,
          min: 0,
          precision: 0.1,
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#3f7c9b",
      tooltip: "Drive at a linear speed, then publish zero velocity.",
    },
    {
      type: "openamr_rotate_for",
      message0: "rotate angular speed %1 for %2 seconds",
      args0: [
        { type: "field_number", name: "ANGULAR", value: 0.5, precision: 0.01 },
        {
          type: "field_number",
          name: "SECONDS",
          value: 2,
          min: 0,
          precision: 0.1,
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#3f7c9b",
      tooltip: "Rotate in place, then publish zero velocity.",
    },
    {
      type: "openamr_stop_movement",
      message0: "stop movement",
      previousStatement: null,
      nextStatement: null,
      colour: "#6b7280",
      tooltip: "Publish zero velocity without canceling navigation.",
    },
    {
      type: "openamr_navigate_named",
      message0: "navigate to location %1",
      args0: [
        { type: "field_dropdown", name: "LOCATION", options: locationOptions },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#087ea4",
      tooltip: "Navigate to a saved named map location.",
    },
    {
      type: "openamr_wait_nav_complete",
      message0: "wait until navigation complete timeout %1 seconds",
      args0: [
        {
          type: "field_number",
          name: "TIMEOUT",
          value: 60,
          min: 1,
          precision: 1,
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#516173",
      tooltip: "Wait for the navigate_to_pose status to finish.",
    },
    {
      type: "openamr_repeat",
      message0: "repeat %1 times",
      args0: [
        { type: "field_number", name: "TIMES", value: 2, min: 1, precision: 1 },
      ],
      message1: "do %1",
      args1: [{ type: "input_statement", name: "DO" }],
      previousStatement: null,
      nextStatement: null,
      colour: "#7c3aed",
      tooltip: "Repeat the nested robot actions.",
    },
    {
      type: "openamr_patrol",
      message0: "patrol A x %1 y %2 yaw %3 B x %4 y %5 yaw %6",
      args0: [
        { type: "field_number", name: "AX", value: 0, precision: 0.01 },
        { type: "field_number", name: "AY", value: 0, precision: 0.01 },
        { type: "field_number", name: "AYAW", value: 0, precision: 0.01 },
        { type: "field_number", name: "BX", value: 2, precision: 0.01 },
        { type: "field_number", name: "BY", value: 0, precision: 0.01 },
        { type: "field_number", name: "BYAW", value: 3.14, precision: 0.01 },
      ],
      message1: "repeat %1 times wait %2 seconds",
      args1: [
        { type: "field_number", name: "TIMES", value: 2, min: 1, precision: 1 },
        {
          type: "field_number",
          name: "WAIT",
          value: 1,
          min: 0,
          precision: 0.1,
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#7c3aed",
      tooltip: "Move between two points for a set number of cycles.",
    },
    {
      type: "openamr_log",
      message0: "log %1",
      args0: [
        { type: "field_input", name: "MESSAGE", text: "Starting action" },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#64748b",
      tooltip:
        "Write a debugging message to the browser console and UI message topic.",
    },
    {
      type: "openamr_battery_below",
      message0: "if battery below %1 percent",
      args0: [
        {
          type: "field_number",
          name: "PERCENT",
          value: 20,
          min: 0,
          max: 100,
          precision: 1,
        },
      ],
      message1: "then %1",
      args1: [{ type: "input_statement", name: "DO" }],
      previousStatement: null,
      nextStatement: null,
      colour: "#ca8a04",
      tooltip:
        "Run nested actions only when battery is below the chosen percentage.",
    },
    {
      type: "openamr_set_mode",
      message0: "set mode %1",
      args0: [
        {
          type: "field_dropdown",
          name: "MODE",
          options: [
            ["autonomous", "autonomous"],
            ["manual", "manual"],
            ["idle", "idle"],
          ],
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#0f766e",
      tooltip: "Publish the selected UI operation mode.",
    },
    {
      type: "openamr_dock",
      message0: "dock robot",
      previousStatement: null,
      nextStatement: null,
      colour: "#22a06b",
      tooltip: "Trigger the docking sequence.",
    },
    {
      type: "openamr_undock",
      message0: "undock robot",
      previousStatement: null,
      nextStatement: null,
      colour: "#d09322",
      tooltip: "Trigger the undocking sequence.",
    },
    {
      type: "openamr_stop",
      message0: "emergency stop",
      previousStatement: null,
      nextStatement: null,
      colour: "#dc3545",
      tooltip: "Publish zero velocity and cancel the active navigation goal.",
    },
  ]);
};

const toNumber = (value, fallback = 0) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
};

const pointAction = (prefix, block) => ({
  type: "navigate",
  x: toNumber(block.getFieldValue(`${prefix}X`)),
  y: toNumber(block.getFieldValue(`${prefix}Y`)),
  yaw: toNumber(block.getFieldValue(`${prefix}YAW`)),
});

const sequenceToRobotPlan = (firstBlock) => {
  const plan = [];
  let block = firstBlock;

  while (block) {
    const action = blockToAction(block);
    if (Array.isArray(action)) {
      plan.push(...action);
    } else if (action) {
      plan.push(action);
    }
    block = block.getNextBlock();
  }

  return plan;
};

const blockToAction = (block) => {
  switch (block.type) {
    case "openamr_navigate":
      return {
        type: "navigate",
        x: toNumber(block.getFieldValue("X")),
        y: toNumber(block.getFieldValue("Y")),
        yaw: toNumber(block.getFieldValue("YAW")),
      };
    case "openamr_wait":
      return {
        type: "wait",
        seconds: Math.max(0, toNumber(block.getFieldValue("SECONDS"))),
      };
    case "openamr_set_speed":
      return {
        type: "set_speed",
        linear: toNumber(block.getFieldValue("LINEAR")),
        angular: toNumber(block.getFieldValue("ANGULAR")),
      };
    case "openamr_drive_for":
      return {
        type: "drive_for",
        linear: toNumber(block.getFieldValue("LINEAR")),
        seconds: Math.max(0, toNumber(block.getFieldValue("SECONDS"))),
      };
    case "openamr_rotate_for":
      return {
        type: "rotate_for",
        angular: toNumber(block.getFieldValue("ANGULAR")),
        seconds: Math.max(0, toNumber(block.getFieldValue("SECONDS"))),
      };
    case "openamr_stop_movement":
      return { type: "stop_movement" };
    case "openamr_navigate_named": {
      const location = block.getFieldValue("LOCATION");
      return {
        type: "navigate",
        location,
        ...OPEN_AMR_LOCATIONS[location],
      };
    }
    case "openamr_wait_nav_complete":
      return {
        type: "wait_nav_complete",
        timeout: Math.max(1, toNumber(block.getFieldValue("TIMEOUT"), 60)),
      };
    case "openamr_repeat":
      return {
        type: "repeat",
        times: Math.max(
          1,
          Math.floor(toNumber(block.getFieldValue("TIMES"), 1)),
        ),
        actions: sequenceToRobotPlan(block.getInputTargetBlock("DO")),
      };
    case "openamr_patrol": {
      const wait = Math.max(0, toNumber(block.getFieldValue("WAIT")));
      return {
        type: "repeat",
        label: "patrol",
        times: Math.max(
          1,
          Math.floor(toNumber(block.getFieldValue("TIMES"), 1)),
        ),
        actions: [
          pointAction("A", block),
          { type: "wait_nav_complete", timeout: 60 },
          { type: "wait", seconds: wait },
          pointAction("B", block),
          { type: "wait_nav_complete", timeout: 60 },
          { type: "wait", seconds: wait },
        ],
      };
    }
    case "openamr_log":
      return {
        type: "log",
        message: block.getFieldValue("MESSAGE"),
      };
    case "openamr_battery_below":
      return {
        type: "battery_below",
        percent: Math.max(0, toNumber(block.getFieldValue("PERCENT"))),
        actions: sequenceToRobotPlan(block.getInputTargetBlock("DO")),
      };
    case "openamr_set_mode":
      return {
        type: "set_mode",
        mode: block.getFieldValue("MODE"),
      };
    case "openamr_dock":
      return { type: "dock" };
    case "openamr_undock":
      return { type: "undock" };
    case "openamr_stop":
      return { type: "stop" };
    default:
      return null;
  }
};

export const workspaceToRobotPlan = (workspace) => {
  const startBlock = workspace
    .getTopBlocks(false)
    .find((block) => block.type === "openamr_start");

  if (!startBlock) return [];

  return sequenceToRobotPlan(startBlock.getNextBlock());
};
