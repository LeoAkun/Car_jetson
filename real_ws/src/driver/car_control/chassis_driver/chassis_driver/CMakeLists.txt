cmake_minimum_required(VERSION 3.5)
project(chassis_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yunle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Threads)


###################################
## catkin specific configuration ##
###################################
## The ament_package macro generates cmake config files for your package



###################################
## catkin specific configuration ##
###################################


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  link_directories("${PROJECT_SOURCE_DIR}/lib/x86/linux_64")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_32")
  link_directories("${PROJECT_SOURCE_DIR}/lib/x86/linux_32")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  link_directories("${PROJECT_SOURCE_DIR}/lib/arm/64bit")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch32")
  link_directories("${PROJECT_SOURCE_DIR}/lib/arm/32bit")
endif()

include_directories(
  include/
)
# message(${CMAKE_SYSTEM})  # Linux-4.15.0-50-generic

add_executable(chassis_driver_node
              src/usb2can_node.cpp
              src/usb2can_core.cpp 
              src/Msg.cpp
              src/MsgBase.cpp
              src/MsgJD02.cpp
              src/MsgJD01.cpp
              src/MsgNWD.cpp
              )
ament_target_dependencies(chassis_driver_node
  rclcpp
  sensor_msgs
  std_msgs
  yunle_msgs
  tf2
  geometry_msgs
  tf2_geometry_msgs
)

target_link_libraries(chassis_driver_node
  ${rclcpp_LIBRARIES}
  ${yunle_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  controlcan  # 将自动从系统库/自定义库中查找libcontrolcan.so
  pthread
)

# 添加运行时库搜索路径
if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(CONTROLCAN_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/x86/linux_64")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_32")
  set(CONTROLCAN_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/x86/linux_32")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  set(CONTROLCAN_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/arm/64bit")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch32")
  set(CONTROLCAN_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/arm/32bit")
endif()

install(TARGETS
  chassis_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

# 将库路径添加到运行时搜索路径
set_target_properties(chassis_driver_node PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE
  INSTALL_RPATH ${CONTROLCAN_LIB_DIR}
)
