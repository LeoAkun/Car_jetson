cmake_minimum_required(VERSION 3.5)
project(re_localization)

# ===============================
# 编译标准设置
# ===============================
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

# ===============================
# 依赖包
# ===============================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # ✅ 新增：生成srv所需
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(OpenMP)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)  # ✅ 可能被srv使用

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ===============================
# 生成自定义srv接口
# ===============================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ReLocalization.srv"          # ✅ 你的服务文件
  DEPENDENCIES geometry_msgs std_msgs  # ✅ 根据srv文件内容添加依赖
)

# ===============================
# 组件库
# ===============================
add_library(pcl_localization_component SHARED
  src/pcl_localization_component.cpp
)

ament_target_dependencies(pcl_localization_component
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
  ndt_omp_ros2
  rosidl_default_runtime  # ✅ 为了使用srv
  std_msgs
)

include_directories(
  include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>  # ✅ srv生成的头文件目录
  ${PCL_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

# ===============================
# 可执行文件
# ===============================
add_executable(pcl_localization_node src/pcl_localization_node.cpp)
target_link_libraries(pcl_localization_node
  pcl_localization_component
  ${PCL_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(pcl_localization_component ${cpp_typesupport_target})
target_link_libraries(pcl_localization_node ${cpp_typesupport_target})

link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# ===============================
# 安装与导出
# ===============================
ament_export_libraries(pcl_localization_component)

install(TARGETS
  pcl_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pcl_localization_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/
)

# ✅ 安装srv文件（方便其他包依赖）
install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
