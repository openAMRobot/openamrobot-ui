cmake_minimum_required(VERSION 3.8)
project(openamr_ui_msgs)

# Basic build tools
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Interface dependencies used by your .msg / .action files
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Generate interfaces (Msg / Action)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArrayPoseStampedWithCovariance.msg"
  "action/MoveBase.action"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    action_msgs
)

# Export runtime dependency so downstream packages can use these interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()
