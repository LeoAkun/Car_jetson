cmake_minimum_required(VERSION 3.8)
project(multi_map_navigation_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/WaypointList.msg"
  "msg/MapSwitchCancel.msg"
  "msg/MapSwitchResult.msg"
  "msg/MapSwitchTrigger.msg"
  "msg/RobotStatus.msg"
  "msg/Edge.msg"
  "msg/StartEndGraph.msg"
  "msg/LocalizationResult.msg"
  "msg/LocalizationSession.msg"
  "msg/LioReadiness.msg"
  "msg/LioCorrectionEvent.msg"
  "msg/EpochOdometry.msg"
  "msg/LioMapOdomCorrection.msg"
  "srv/StartProcess.srv"
  "srv/ShutdownProcess.srv"
  "srv/GetProcessStatus.srv"
  "srv/PubNewPath.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces nav_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
