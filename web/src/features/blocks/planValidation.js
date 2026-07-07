import { AppConfig } from "../../shared/constants";

const START_BLOCK_TYPE = "openamr_start";

const actionChildren = (action) => {
  if (action.type === "repeat" || action.type === "battery_below") {
    return action.actions || [];
  }
  return [];
};

const collectActions = (actions, depth = 0) =>
  actions.flatMap((action, index) => [
    {
      action,
      index,
      depth,
      label: depth === 0 ? `Step ${index + 1}` : `Nested step ${index + 1}`,
    },
    ...collectActions(actionChildren(action), depth + 1),
  ]);

const blockTypeLabel = (type) =>
  type
    .replace(/^openamr_/, "")
    .replaceAll("_", " ")
    .replace(/\b\w/g, (letter) => letter.toUpperCase());

export const validateRobotPlan = (workspace, plan) => {
  const messages = [];
  const topBlocks = workspace?.getTopBlocks(false) || [];
  const startBlocks = topBlocks.filter(
    (block) => block.type === START_BLOCK_TYPE,
  );
  const looseBlocks = topBlocks.filter(
    (block) => block.type !== START_BLOCK_TYPE,
  );
  const allActions = collectActions(plan);

  if (startBlocks.length === 0) {
    messages.push({
      severity: "error",
      text: "Add one start robot program block before running.",
    });
  }

  if (startBlocks.length > 1) {
    messages.push({
      severity: "warning",
      text: "Only the first start robot program block will run. Remove extra start blocks to avoid confusion.",
    });
  }

  if (startBlocks.length === 1 && plan.length === 0) {
    messages.push({
      severity: "warning",
      text: "The start block has no connected robot actions.",
    });
  }

  looseBlocks.forEach((block) => {
    messages.push({
      severity: "warning",
      text: `Loose ${blockTypeLabel(
        block.type,
      )} block is not connected to the start chain and will not run.`,
    });
  });

  allActions.forEach(({ action, label }) => {
    if (
      action.type === "set_speed" &&
      Math.abs(action.linear) > AppConfig.MAX_LINEAR_SPEED
    ) {
      messages.push({
        severity: "error",
        text: `${label}: linear speed ${action.linear} m/s exceeds the ${AppConfig.MAX_LINEAR_SPEED} m/s limit.`,
      });
    }

    if (
      action.type === "set_speed" &&
      Math.abs(action.angular) > AppConfig.MAX_ANGULAR_SPEED
    ) {
      messages.push({
        severity: "error",
        text: `${label}: angular speed ${action.angular} rad/s exceeds the ${AppConfig.MAX_ANGULAR_SPEED} rad/s limit.`,
      });
    }

    if (
      action.type === "drive_for" &&
      Math.abs(action.linear) > AppConfig.MAX_LINEAR_SPEED
    ) {
      messages.push({
        severity: "error",
        text: `${label}: drive speed ${action.linear} m/s exceeds the ${AppConfig.MAX_LINEAR_SPEED} m/s limit.`,
      });
    }

    if (
      action.type === "rotate_for" &&
      Math.abs(action.angular) > AppConfig.MAX_ANGULAR_SPEED
    ) {
      messages.push({
        severity: "error",
        text: `${label}: rotate speed ${action.angular} rad/s exceeds the ${AppConfig.MAX_ANGULAR_SPEED} rad/s limit.`,
      });
    }

    if (
      (action.type === "repeat" || action.type === "battery_below") &&
      action.actions.length === 0
    ) {
      messages.push({
        severity: "warning",
        text: `${label}: ${
          action.type === "repeat" ? "repeat" : "battery check"
        } block has no nested actions.`,
      });
    }
  });

  plan.forEach((action, index) => {
    if (action.type === "navigate" && action.missingLocation) {
      messages.push({
        severity: "error",
        text: `Step ${index + 1}: named location "${
          action.location
        }" is not available from the backend.`,
      });
    }

  });

  return messages;
};

export const hasValidationErrors = (messages) =>
  messages.some((message) => message.severity === "error");
