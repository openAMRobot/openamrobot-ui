export const AppConfig = {
  ROSBRIDGE_SERVER_IP: "192.168.0.100",
  ROSBRIDGE_SERVER_PORT: "9090",
  CAMERA_PORT: "8090",
  RECONNECTION_TIME: 1000,

  CMD_VEL_TOPIC: "/cmd_vel",
  ROBOT_POSE_TOPIC: "/odom",
  ROBOT_VELOCITY_TOPIC: "/odom",
  UI_OPERATION_TOPIC: "/ui_operation",
  UI_MESSAGE_TOPIC: "/ui_message",
  UI_OPERATION: "/ui_operation",
  SET_POINT: "/may_set_point",
  WP_REQ: "/WP_req",
  SENSORS_TOPIC: "/sensors",
  BATTERY_TOPIC: "/battery_status",
  CHARGE_STATION_CONNECTED: "/charge_station_connected",

  MAX_LINEAR_SPEED: 0.2,
  MAX_ANGULAR_SPEED: 2,
};
