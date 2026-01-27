cmake_minimum_required(VERSION 3.8)
project(gnss_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)

# 添加可执行文件
add_executable(gnss_converter_node src/gnss_converter_node.cpp)

# 包含目录
target_include_directories(gnss_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 设置 C++ 标准
target_compile_features(gnss_converter_node PUBLIC c_std_99 cxx_std_17)

# 链接依赖
ament_target_dependencies(gnss_converter_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  serial
)

# 安装可执行文件
install(TARGETS gnss_converter_node
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# 安装 config 文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
