// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <chrono>
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

// #define BENCHMARK_TESTING

namespace nav2_mppi_controller
{

void MPPIController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<ParametersHandler>(parent);

  auto node = parent_.lock();
  clock_ = node->get_clock();
  last_time_called_ = clock_->now();
  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false);
  getParam(reset_period_, "reset_period", 1.0);

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_ros_->getGlobalFrameID(), parameters_handler_.get());

  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void MPPIController::cleanup()
{
  optimizer_.shutdown();
  trajectory_visualizer_.on_cleanup();
  parameters_handler_.reset();
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void MPPIController::activate()
{
  trajectory_visualizer_.on_activate();
  parameters_handler_->start();
  RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
}

void MPPIController::deactivate()
{
  trajectory_visualizer_.on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated MPPI Controller: %s", name_.c_str());
}

void MPPIController::reset()
{
  optimizer_.reset();
}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  // std::cout << "mppi控制器" << std::endl;
#ifdef BENCHMARK_TESTING
  auto start = std::chrono::system_clock::now();
#endif

  if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
    reset();
  }
  last_time_called_ = clock_->now();

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  // 打印costmap信息
// {
//     // 基本信息
//   unsigned int size_x = costmap->getSizeInCellsX();
//   unsigned int size_y = costmap->getSizeInCellsY();
//   double res = costmap->getResolution();
//   double ox = costmap->getOriginX();
//   double oy = costmap->getOriginY();

//   RCLCPP_INFO(logger_, "COSTMAP: size %u x %u, res %.3f, origin (%.3f, %.3f)",
//               size_x, size_y, res, ox, oy);

//   // 将 robot_pose 转为 map 索引（若超界则跳过）
//   unsigned int mx, my;
//   bool ok = costmap->worldToMap(robot_pose.pose.position.x, robot_pose.pose.position.y, mx, my);
//   if (ok) {
//     int cx = static_cast<int>(mx);
//     int cy = static_cast<int>(my);
//     unsigned char center_cost = costmap->getCost(cx, cy);
//     RCLCPP_INFO(logger_, "Robot map index: (%d,%d) center cost: %u", cx, cy, center_cost);

//     // 抽样 5x5 周围格子
//     int radius = 2;
//     for (int dy=-radius; dy<=radius; ++dy) {
//       std::stringstream ss;
//       for (int dx=-radius; dx<=radius; ++dx) {
//         int sx = cx + dx;
//         int sy = cy + dy;
//         if (sx >= 0 && sy >= 0 && sx < (int)size_x && sy < (int)size_y) {
//           unsigned char cost = costmap->getCost(sx, sy);
//           ss << std::setw(3) << (int)cost << " ";
//         } else {
//           ss << "NA  ";
//         }
//       }
//       RCLCPP_INFO(logger_, "cost row: %s", ss.str().c_str());
//     }
//   } else {
//     RCLCPP_WARN(logger_, "Robot pose not inside costmap bounds");
//   }
// }

  geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal_checker); // 计算速度

#ifdef BENCHMARK_TESTING
  auto end = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
#endif

  if (visualize_) {
    visualize(std::move(transformed_plan));
  }

  return cmd;
}

void MPPIController::visualize(nav_msgs::msg::Path transformed_plan)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory");
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_.setPath(path);
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace nav2_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)
