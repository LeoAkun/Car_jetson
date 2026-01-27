cmake_minimum_required(VERSION 3.8)
project(lock_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LockStatus.msg"
  "msg/AllLockStatus.msg"
  "srv/UnlockCommand.srv"
  "srv/QueryLockStatus.srv"
  "srv/UnlockMultiple.srv"
  "srv/LongPowerControl.srv"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(include)

# Add executable
add_executable(lock_controller_node src/lock_controller.cpp)
ament_target_dependencies(lock_controller_node rclcpp std_msgs)

# Link generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(lock_controller_node "${cpp_typesupport_target}")

# Install executable
install(TARGETS
  lock_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/unlock_example.py
  scripts/query_status_example.py
  scripts/lock_status_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
