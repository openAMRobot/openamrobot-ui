export const AppConfig = {
  ROSBRIDGE_SERVER_IP: "192.168.0.100",
  ROSBRIDGE_SERVER_PORT: "9090",
  CAMERA_PORT: "8080",
  RECONNECTION_TIME: 1000,

  // Manual drive input (geometry_msgs/TwistStamped). The platform relay
  // (openamr-platform-sw#90, ui_cmd_vel_relay_node) forwards it to twist_mux;
  // collision_monitor stays the only publisher of the final /cmd_vel.
  // Only shared/motion/uiCommand.js may publish here.
  UI_CMD_VEL_TOPIC: "/ui/cmd_vel",
  // Stop lock owned by safety_stop_gate_node (openamr-platform-sw#90):
  // std_msgs/Bool, true = engaged, RELIABLE + TRANSIENT_LOCAL depth 1.
  STOP_LOCK_TOPIC: "/safety/stop_lock",
  STOP_LOCK_ENGAGE_SERVICE: "/safety/stop_lock_engage", // std_srvs/Trigger
  STOP_LOCK_RELEASE_SERVICE: "/safety/stop_lock_release", // std_srvs/Trigger
  // Robot clock sources for stamping /ui/cmd_vel (see shared/motion/robotClock.js).
  UI_CMD_VEL_RELAY_GET_PARAMETERS_SERVICE: "/ui_cmd_vel_relay_node/get_parameters",
  CLOCK_TOPIC: "/clock",
  ROSAPI_GET_TIME_SERVICE: "/rosapi/get_time",
  ROBOT_POSE_TOPIC: "/odom",
  ROBOT_VELOCITY_TOPIC: "/odom",
  MAP_TOPIC: "/ui/map",
  GLOBAL_COSTMAP_TOPIC: "/global_costmap/costmap",
  LOCAL_COSTMAP_TOPIC: "/local_costmap/costmap",
  SCAN_TOPIC: "/scan_filtered",
  PLAN_TOPIC: "/plan",
  TF_TOPIC: "/tf",
  TF_STATIC_TOPIC: "/tf_static",
  AMCL_POSE_TOPIC: "/ui/amcl_pose",
  NAV_STATUS_TOPIC: "/ui/navigate_to_pose/status",
  NAV_FEEDBACK_TOPIC: "/navigate_to_pose/_action/feedback",
  NAV_CANCEL_GOAL_SERVICE: "/navigate_to_pose/_action/cancel_goal",
  // Cancel services for the other actions the UI starts, directly
  // (compute_path_to_pose from the Route page) or through dock_trigger
  // (undock_robot; dock_robot status is relayed to /ui/dock_robot/status).
  COMPUTE_PATH_CANCEL_GOAL_SERVICE: "/compute_path_to_pose/_action/cancel_goal",
  DOCK_CANCEL_GOAL_SERVICE: "/dock_robot/_action/cancel_goal",
  UNDOCK_CANCEL_GOAL_SERVICE: "/undock_robot/_action/cancel_goal",
  DOCK_STATUS_TOPIC: "/ui/dock_robot/status",
  DOCK_TRIGGER_STATUS_TOPIC: "/dock_trigger_status",
  DOCK_TRIGGER_TOPIC: "/dock_trigger",
  UNDOCK_STATUS_TOPIC: "/ui/undock_robot/status",
  UNDOCK_TRIGGER_TOPIC: "/undock_robot",
  GOAL_POSE_TOPIC: "/goal_pose",
  INITIAL_POSE_TOPIC: "/initialpose",
  UI_OPERATION_TOPIC: "/ui_operation",
  UI_MESSAGE_TOPIC: "/ui_message",
  UI_OPERATION: "/ui_operation",
  SET_POINT: "/may_set_point",
  WP_REQ: "/WP_req",
  SENSORS_TOPIC: "/sensors",
  BATTERY_TOPIC: "/battery_status",
  // Navigation health (openamr_nav_msgs/NavigationStatus, contract version 1),
  // published by openamr-platform-sw navigation_status_node.
  NAVIGATION_STATUS_TOPIC: "/navigation/status",
  DIAGNOSTICS_TOPIC: "/diagnostics",
  CHARGE_STATION_CONNECTED: "/charge_station_connected",

  // Robot Description page — Live Mode joint telemetry (sensor_msgs/JointState)
  JOINT_STATES_TOPIC: "/joint_states",
  // robot_state_publisher's robot_description (std_msgs/String), subscribed
  // read-only so the page can show the model the robot itself publishes.
  ROBOT_DESCRIPTION_TOPIC: "/robot_description",

  // Route editor: file/waypoint exchange with openamr_ui_package's folders_handler node
  NAV_DATA_REQ_TOPIC: "/nav_data_req",
  NAV_DATA_RESP_TOPIC: "/nav_data_resp",
  NEW_WAYPOINT_TOPIC: "/new_way_point",
  // compute_path_to_pose is a ROS2 action, not a service — there is no
  // "/compute_path_to_pose" service on the graph to call directly. Every
  // ROS2 action implicitly exposes a goal-submission and a result-retrieval
  // service (the same pattern NAV_FEEDBACK_TOPIC/NAV_CANCEL_GOAL_SERVICE
  // above already rely on for navigate_to_pose), so planning goes through
  // these two instead.
  COMPUTE_PATH_SEND_GOAL_SERVICE: "/compute_path_to_pose/_action/send_goal",
  COMPUTE_PATH_GET_RESULT_SERVICE: "/compute_path_to_pose/_action/get_result",

  MAX_LINEAR_SPEED: 0.2,
  MAX_ANGULAR_SPEED: 2,
};

// Every action cancel service Stop (software) calls. A CancelGoal request
// with a zero goal id and zero stamp cancels all goals of that action server.
export const ACTION_CANCEL_SERVICES = [
  AppConfig.NAV_CANCEL_GOAL_SERVICE,
  AppConfig.COMPUTE_PATH_CANCEL_GOAL_SERVICE,
  AppConfig.DOCK_CANCEL_GOAL_SERVICE,
  AppConfig.UNDOCK_CANCEL_GOAL_SERVICE,
];

// Camera image topics selectable on the Camera panel; value must match a
// topic actually published by the robot/simulation stack or web_video_server
// will just show nothing for that selection.
export const CAMERA_TOPIC_OPTIONS = [
  { value: "/camera/color/image_raw", label: "Color" },
  { value: "/rgb_image", label: "RGB Sim" },
  { value: "/camera/image_raw", label: "Raw" },
];
export const DEFAULT_CAMERA_TOPIC = "/rgb_image";

// Head camera of the OpenAMRobot 2.0 upper body. No contract or producer
// defines its image topic yet, so it is null and the Camera panel shows it as
// "not reported". Set it only when a contracted topic exists.
export const HEAD_CAMERA_TOPIC = null;

// Nav2 lifecycle-managed nodes polled/controlled from the Health page.
// `base` is the ROS node namespace exposing get_state/change_state services.
export const LIFECYCLE_NODES = [
  { name: "map_server", base: "/map_server" },
  { name: "amcl", base: "/amcl" },
  { name: "controller", base: "/controller_server" },
  { name: "planner", base: "/planner_server" },
  { name: "bt_navigator", base: "/bt_navigator" },
];
