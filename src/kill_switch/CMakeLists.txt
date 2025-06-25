find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "kill_switch/action/ConnectKillSwitch.action"
  DEPENDENCIES builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()