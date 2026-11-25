cmake_minimum_required(VERSION 3.8)
project(charging_dock_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DockTagObservation.msg"
  "msg/DockTagObservationArray.msg"
  "msg/DockObservation.msg"
  "action/Dock.action"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
