/** Message builders for reporting tests and screenshots. Test data only. */

export const navStatus = (overrides = {}) => ({
  header: { stamp: { sec: 100, nanosec: 0 }, frame_id: "" },
  contract_version: 1,
  profile_id: "sim",
  thresholds_id: "UNSET",
  health: 1,
  navigation_readiness: 1,
  not_ready_reasons: [],
  active_reasons: [],
  sensor_gaps: 0,
  stack: { state: 3, reason: 0, inactive_nodes: [] },
  localization: { state: 3, health: 1, reason: 0, tf_available: true, correction_paused: false },
  sensors: [],
  task: { state: 1, reason: 0, native_error_code: 0, distance_remaining: 0 },
  recovery: { action: 0, attempt: 0, attempt_limit: 6, reason: 0 },
  protection: { collision_monitor: 2, collision_monitor_reason: 0, motion_sources: [], rear_coverage: 2, rear_coverage_reason: 0 },
  constraints: [],
  ...overrides,
});

export const uuid = (n) => Array.from({ length: 16 }, (_, i) => (i === 15 ? n : 0));

export const goalStatus = (n, status, sec = 100 + n) => ({
  status_list: [{ goal_info: { goal_id: { uuid: uuid(n) }, stamp: { sec, nanosec: 0 } }, status }],
});

export const navFeedback = (n, distance, eta = 10, recoveries = 0) => ({
  goal_id: { uuid: uuid(n) },
  feedback: {
    distance_remaining: distance,
    estimated_time_remaining: { sec: eta, nanosec: 0 },
    number_of_recoveries: recoveries,
  },
});

export const diagnostics = (level = 0, name = "navigation_status_node: health", message = "") => ({
  header: { stamp: { sec: 100, nanosec: 0 } },
  status: [{ level, name, message, hardware_id: "", values: [] }],
});

export const battery = (percent) => ({ data: percent });
export const lock = (engaged) => ({ data: engaged });
