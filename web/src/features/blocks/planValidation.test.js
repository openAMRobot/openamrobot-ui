import { hasValidationErrors, validateRobotPlan } from "./planValidation";

// A workspace with exactly one start block, so only the plan rules apply.
const workspace = {
  getTopBlocks: () => [{ type: "openamr_start" }],
};

// The limits GET /api/limits serves today (param/voice_limits.yaml).
const BACKEND_LIMITS = {
  max_linear_speed_mps: 0.2,
  max_angular_speed_radps: 2.0,
  max_distance_per_action_m: 5.0,
  max_duration_per_action_s: 30.0,
};

const errorsFor = (plan, limits = BACKEND_LIMITS) =>
  validateRobotPlan(workspace, plan, limits).filter((m) => m.severity === "error");

describe("plan validation uses the backend limits", () => {
  it("blocks running until the limits are loaded", () => {
    const messages = validateRobotPlan(workspace, [{ type: "wait", seconds: 1 }], null);
    expect(hasValidationErrors(messages)).toBe(true);
    expect(messages.map((m) => m.text).join(" ")).toMatch(/limits have not been loaded/);
  });

  it("follows the limits it is given, not a built-in constant", () => {
    const tighter = { ...BACKEND_LIMITS, max_linear_speed_mps: 0.1 };
    const plan = [{ type: "set_speed", linear: 0.15, angular: 0 }];
    expect(errorsFor(plan)).toEqual([]);
    expect(errorsFor(plan, tighter)[0].text).toMatch(/0.1 m\/s limit/);
  });

  it.each([
    ["linear speed", { type: "set_speed", linear: 0.25, angular: 0 }],
    ["angular speed", { type: "rotate_for", angular: -2.5, seconds: 1 }],
    ["duration", { type: "wait", seconds: 31 }],
    ["distance", { type: "drive_for", linear: 0.2, seconds: 26 }],
    ["non-finite", { type: "drive_for", linear: Number.NaN, seconds: 1 }],
    ["nested", { type: "repeat", times: 2, actions: [{ type: "set_speed", linear: 1, angular: 0 }] }],
  ])("rejects a %s violation", (_name, action) => {
    expect(errorsFor([action]).length).toBeGreaterThan(0);
  });

  it("accepts a plan at the limits", () => {
    expect(
      errorsFor([
        { type: "set_speed", linear: 0.2, angular: 2 },
        { type: "drive_for", linear: 0.1, seconds: 30 },
        { type: "rotate_for", angular: -2, seconds: 30 },
      ]),
    ).toEqual([]);
  });
});
