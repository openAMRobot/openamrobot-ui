export const AppConfig = {
  ROSBRIDGE_SERVER_IP: "192.168.0.100",
  ROSBRIDGE_SERVER_PORT: "9090",
  CAMERA_PORT: "8080",
  RECONNECTION_TIME: 1000,

  CMD_VEL_TOPIC: "/cmd_vel",
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
  CHARGE_STATION_CONNECTED: "/charge_station_connected",

  // Robot Description page — Live Mode joint telemetry (sensor_msgs/JointState)
  JOINT_STATES_TOPIC: "/joint_states",

  // Route editor: file/waypoint exchange with openamr_ui_package's folders_handler node
  NAV_DATA_REQ_TOPIC: "/nav_data_req",
  NAV_DATA_RESP_TOPIC: "/nav_data_resp",
  NEW_WAYPOINT_TOPIC: "/new_way_point",
  COMPUTE_PATH_SERVICE: "/compute_path_to_pose",

  MAX_LINEAR_SPEED: 0.2,
  MAX_ANGULAR_SPEED: 2,
};

// Camera image topics selectable on the Camera panel; value must match a
// topic actually published by the robot/simulation stack or web_video_server
// will just show nothing for that selection.
export const CAMERA_TOPIC_OPTIONS = [
  { value: "/camera/color/image_raw", label: "Color" },
  { value: "/rgb_image", label: "RGB Sim" },
  { value: "/camera/image_raw", label: "Raw" },
];
export const DEFAULT_CAMERA_TOPIC = "/rgb_image";

// Nav2 lifecycle-managed nodes polled/controlled from the Health page.
// `base` is the ROS node namespace exposing get_state/change_state services.
export const LIFECYCLE_NODES = [
  { name: "map_server", base: "/map_server" },
  { name: "amcl", base: "/amcl" },
  { name: "controller", base: "/controller_server" },
  { name: "planner", base: "/planner_server" },
  { name: "bt_navigator", base: "/bt_navigator" },
];
