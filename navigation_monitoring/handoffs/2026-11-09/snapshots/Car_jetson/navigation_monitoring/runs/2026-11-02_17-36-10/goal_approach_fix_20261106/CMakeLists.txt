cmake_minimum_required(VERSION 3.16)
project(goal_approach_offline_validation LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(xtensor REQUIRED)
find_package(xsimd REQUIRED)

get_filename_component(CAR_PROJECT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
set(MPPI_SOURCE "${CAR_PROJECT}/nav2/src/navigation2/nav2_mppi_controller")
add_executable(critic_harness
  critic_harness.cpp
  "${MPPI_SOURCE}/src/critics/goal_critic.cpp"
  "${MPPI_SOURCE}/src/critics/path_follow_critic.cpp")
target_compile_features(critic_harness PRIVATE cxx_std_17)
target_compile_options(critic_harness PRIVATE -fconcepts)
target_compile_definitions(critic_harness PRIVATE XTENSOR_ENABLE_XSIMD XTENSOR_USE_XSIMD)
target_include_directories(critic_harness BEFORE PRIVATE "${MPPI_SOURCE}/include")
ament_target_dependencies(critic_harness nav2_mppi_controller)
target_link_libraries(critic_harness xtensor xtensor::optimize xtensor::use_xsimd)
