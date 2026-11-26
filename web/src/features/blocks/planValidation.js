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

// Mirrors the backend rules in openamr_ui_package/voice_limits.py; the
// limits themselves come from GET /api/limits (see backendLimits.js).
const isFiniteNumber = (value) => typeof value === "number" && Number.isFinite(value);

const NUMERIC_FIELDS = {
  set_speed: ["linear", "angular"],
  drive_for: ["linear", "seconds"],
  rotate_for: ["angular", "seconds"],
  wait: ["seconds"],
};

export const limitErrors = (action, limits) => {
  const fields = NUMERIC_FIELDS[action.type] || [];
  const notFinite = fields.filter((field) => !isFiniteNumber(action[field]));
  if (notFinite.length) {
    return notFinite.map((field) => `${field} must be a finite number.`);
  }
  const errors = [];
  const maxLinear = limits.max_linear_speed_mps;
  const maxAngular = limits.max_angular_speed_radps;
  const maxDuration = limits.max_duration_per_action_s;
  const maxDistance = limits.max_distance_per_action_m;
  if (fields.includes("linear") && Math.abs(action.linear) > maxLinear) {
    errors.push(`linear speed ${action.linear} m/s exceeds the ${maxLinear} m/s limit.`);
  }
  if (fields.includes("angular") && Math.abs(action.angular) > maxAngular) {
    errors.push(`angular speed ${action.angular} rad/s exceeds the ${maxAngular} rad/s limit.`);
  }
  if (fields.includes("seconds")) {
    if (action.seconds < 0) errors.push("duration must not be negative.");
    else if (action.seconds > maxDuration) {
      errors.push(`duration ${action.seconds} s exceeds the ${maxDuration} s limit.`);
    }
  }
  if (action.type === "drive_for") {
    const distance = Math.abs(action.linear) * Math.abs(action.seconds);
    if (distance > maxDistance) {
      errors.push(`distance ${distance} m (speed x duration) exceeds the ${maxDistance} m limit.`);
    }
  }
  return errors;
};

export const validateRobotPlan = (workspace, plan, limits) => {
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

  if (!limits) {
    messages.push({
      severity: "error",
      text: "Plan limits have not been loaded from the backend, so the plan cannot be checked or run.",
    });
  }

  allActions.forEach(({ action, label }) => {
    if (limits) {
      limitErrors(action, limits).forEach((text) =>
        messages.push({ severity: "error", text: `${label}: ${text}` }),
      );
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
