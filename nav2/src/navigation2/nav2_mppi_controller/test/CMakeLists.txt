set(TEST_NAMES
  optimizer_smoke_test
  controller_state_transition_test
  models_test
  noise_generator_test
  parameter_handler_test
  motion_model_tests
  trajectory_visualizer_tests
  utils_test
  path_handler_test
  critic_manager_test
  optimizer_unit_tests
)

foreach(name IN LISTS TEST_NAMES)
  ament_add_gtest(${name}
    ${name}.cpp
  )

  ament_target_dependencies(${name}
    ${dependencies_pkgs}
  )

  target_link_libraries(${name}
    mppi_controller
  )

  if(${TEST_DEBUG_INFO})
    target_compile_definitions(${name} PUBLIC -DTEST_DEBUG_INFO)
  endif()

endforeach()

# A native regression executable: no rclcpp initialization, ROS nodes or DDS.
# It calls real optimizer/critic methods with synthetic critics and plain data.
add_executable(optimizer_retry_regression_test optimizer_retry_regression_test.cpp)
# Keep std::isfinite meaningful despite the package's production -ffast-math.
target_compile_options(optimizer_retry_regression_test PRIVATE -fno-finite-math-only)
ament_target_dependencies(optimizer_retry_regression_test ${dependencies_pkgs})
target_link_libraries(optimizer_retry_regression_test mppi_controller)
add_test(NAME optimizer_retry_regression_test COMMAND optimizer_retry_regression_test)

# This is a special case requiring linking against the critics library
ament_add_gtest(critics_tests critics_tests.cpp)
ament_target_dependencies(critics_tests ${dependencies_pkgs})
target_link_libraries(critics_tests mppi_controller mppi_critics)
if(${TEST_DEBUG_INFO})
  target_compile_definitions(critics_tests PUBLIC -DTEST_DEBUG_INFO)
endif()
