// Shared between ParamsPage (interactive tuning) and the support-package
// exporter (read-only snapshot) so both agree on which params are "the
// curated set" without duplicating the starter list or the rcl_interfaces
// value-decoding logic.

export const PARAM_ROWS_STORAGE_KEY = "openamrParamRows";

// A starter set of commonly-tuned Nav2 parameters. Exact names depend on the
// robot's Nav2 config, so every field is editable and the list is persisted —
// operators curate their own working set. Node is the fully-qualified node
// name; the get/set services live at <node>/{get,set}_parameters.
export const DEFAULT_PARAM_ROWS = [
  { node: "/controller_server", param: "FollowPath.max_vel_x", type: "double" },
  { node: "/controller_server", param: "FollowPath.max_vel_theta", type: "double" },
  { node: "/controller_server", param: "general_goal_checker.xy_goal_tolerance", type: "double" },
  { node: "/planner_server", param: "GridBased.tolerance", type: "double" },
  { node: "/global_costmap/global_costmap", param: "inflation_layer.inflation_radius", type: "double" },
  { node: "/local_costmap/local_costmap", param: "inflation_layer.inflation_radius", type: "double" },
];

// rcl_interfaces/msg/ParameterType values.
export const PARAM_TYPES = { bool: 1, int: 2, double: 3, string: 4 };

export const readParamValue = (pv) => {
  if (!pv) return "";
  switch (pv.type) {
    case PARAM_TYPES.bool:
      return String(pv.bool_value);
    case PARAM_TYPES.int:
      return String(pv.integer_value);
    case PARAM_TYPES.double:
      return String(pv.double_value);
    case PARAM_TYPES.string:
      return pv.string_value;
    default:
      return "";
  }
};

export const buildParamValue = (type, raw) => {
  const v = {
    type: 0,
    bool_value: false,
    integer_value: 0,
    double_value: 0,
    string_value: "",
    byte_array_value: [],
    bool_array_value: [],
    integer_array_value: [],
    double_array_value: [],
    string_array_value: [],
  };
  if (type === "bool") {
    v.type = PARAM_TYPES.bool;
    v.bool_value = raw === true || raw === "true" || raw === "1";
  } else if (type === "int") {
    v.type = PARAM_TYPES.int;
    v.integer_value = parseInt(raw, 10) || 0;
  } else if (type === "double") {
    v.type = PARAM_TYPES.double;
    v.double_value = parseFloat(raw) || 0;
  } else {
    v.type = PARAM_TYPES.string;
    v.string_value = String(raw ?? "");
  }
  return v;
};

export const loadParamRows = () => {
  try {
    const stored = JSON.parse(localStorage.getItem(PARAM_ROWS_STORAGE_KEY) || "null");
    return Array.isArray(stored) && stored.length ? stored : DEFAULT_PARAM_ROWS;
  } catch {
    return DEFAULT_PARAM_ROWS;
  }
};
