// Offline scoring only. No ROS initialization, node, context, costmap node or publisher.
// Compile the unmodified local GoalCritic and PathFollowCritic implementations.
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_mppi_controller/critics/goal_critic.hpp"
#include "nav2_mppi_controller/critics/path_follow_critic.hpp"

class OfflineGoal : public mppi::critics::GoalCritic
{
public:
  OfflineGoal(float threshold, float weight, unsigned power)
  {
    enabled_ = true;
    threshold_to_consider_ = threshold;
    weight_ = weight;
    power_ = power;
  }
};

class OfflinePath : public mppi::critics::PathFollowCritic
{
public:
  OfflinePath(float threshold, float weight, unsigned power, size_t offset)
  {
    enabled_ = true;
    threshold_to_consider_ = threshold;
    weight_ = weight;
    power_ = power;
    offset_from_furthest_ = offset;
  }
};

struct Score
{
  std::array<float, 3> goal{}, path{}, total{};
  bool goal_active{}, path_active{};
};

struct Settings
{
  float old_goal, new_goal, path_threshold, goal_weight, path_weight;
  unsigned goal_power, path_power, path_offset, steps;
  float dt, speed;
};

Score score_case(const Settings & p, float signed_goal_x, float goal_threshold)
{
  mppi::models::State state;
  state.reset(3, p.steps);
  state.pose.pose.orientation.w = 1.0;
  mppi::models::Trajectories trajectories;
  trajectories.reset(3, p.steps);
  // Three straight example trajectories: forward, stationary, reverse.
  // Velocity is the preserved physical low-speed compensation, in SI units.
  const std::array<float, 3> velocities{p.speed, 0.0f, -p.speed};
  for (size_t row = 0; row < velocities.size(); ++row) {
    for (size_t step = 0; step < p.steps; ++step) {
      state.vx(row, step) = velocities[row];
      trajectories.x(row, step) = velocities[row] * p.dt * (step + 1);
    }
  }

  mppi::models::Path path;
  const size_t points = std::max<size_t>(2, std::ceil(std::abs(signed_goal_x) / 0.05f) + 1);
  path.reset(points);
  for (size_t i = 0; i < points; ++i) {
    path.x(i) = signed_goal_x * i / (points - 1);
  }
  xt::xtensor<float, 1> costs = xt::zeros<float>({3});
  float dt = p.dt;
  // This represents an explicitly all-free synthetic path. Precomputed validity
  // bypasses only map lookup; the real furthest-point calculation still runs.
  mppi::CriticData data{state, trajectories, path, costs, dt, false, nullptr,
    nullptr, std::vector<bool>(points, true), std::nullopt};
  OfflineGoal goal(goal_threshold, p.goal_weight, p.goal_power);
  OfflinePath follow(p.path_threshold, p.path_weight, p.path_power, p.path_offset);
  Score result;
  result.goal_active = mppi::utils::withinPositionGoalTolerance(
    goal_threshold, state.pose.pose, path);
  result.path_active = !mppi::utils::withinPositionGoalTolerance(
    p.path_threshold, state.pose.pose, path);
  goal.score(data);
  for (size_t i = 0; i < 3; ++i) {result.goal[i] = costs(i);}
  costs.fill(0.0f);
  follow.score(data);
  for (size_t i = 0; i < 3; ++i) {
    result.path[i] = costs(i);
    result.total[i] = result.goal[i] + result.path[i];
  }
  return result;
}

void print_array(const std::array<float, 3> & values)
{
  std::cout << '[' << values[0] << ',' << values[1] << ',' << values[2] << ']';
}

void print_score(const Score & s)
{
  std::cout << "{\"goal_active\":" << std::boolalpha << s.goal_active
            << ",\"path_active\":" << s.path_active << ",\"goal\":";
  print_array(s.goal);
  std::cout << ",\"path\":";
  print_array(s.path);
  std::cout << ",\"sum_of_two_critics\":";
  print_array(s.total);
  std::cout << '}';
}

bool all_zero(const std::array<float, 3> & values)
{
  return std::all_of(values.begin(), values.end(), [](float v) {return std::abs(v) < 1e-6f;});
}

int main(int argc, char ** argv)
{
  if (argc != 12) {
    std::cerr << "Expected old/new goal threshold, path threshold, two weights, "
              << "two powers, path offset, steps, dt, SI low speed\n";
    return 2;
  }
  Settings p{std::stof(argv[1]), std::stof(argv[2]), std::stof(argv[3]),
    std::stof(argv[4]), std::stof(argv[5]), static_cast<unsigned>(std::stoul(argv[6])),
    static_cast<unsigned>(std::stoul(argv[7])), static_cast<unsigned>(std::stoul(argv[8])),
    static_cast<unsigned>(std::stoul(argv[9])), std::stof(argv[10]), std::stof(argv[11])};
  std::vector<std::string> failed;
  size_t checks = 0;
  auto check = [&](bool condition, const std::string & label) {
      ++checks;
      if (!condition) {failed.push_back(label);}
    };
  check(p.old_goal < p.path_threshold, "historical gap exists");
  check(p.new_goal > p.path_threshold, "new goal/path activation overlaps");
  std::cout << std::setprecision(9) << "{\"candidate_order\":[\"forward\",\"stationary\",\"reverse\"],"
            << "\"cases\":[";
  std::vector<float> distances{0.5f, 0.8f, 1.0f, 1.2f, 1.4f, p.new_goal, 3.0f};
  for (float boundary : {p.old_goal, p.path_threshold, p.new_goal}) {
    distances.push_back(boundary - 0.0001f);
    distances.push_back(boundary + 0.0001f);
  }
  std::sort(distances.begin(), distances.end());
  distances.erase(std::unique(distances.begin(), distances.end()), distances.end());
  bool comma = false;
  for (float distance : distances) {
    auto old = score_case(p, distance, p.old_goal);
    auto current = score_case(p, distance, p.new_goal);
    if (comma) {std::cout << ',';}
    comma = true;
    std::cout << "{\"signed_goal_x_m\":" << distance << ",\"old\":";
    print_score(old);
    std::cout << ",\"new\":";
    print_score(current);
    std::cout << '}';
    check(!all_zero(current.total), "new scoring active at " + std::to_string(distance));
    check(current.total[0] < current.total[1] && current.total[1] < current.total[2],
      "goal ahead prefers forward over stationary over reverse at " + std::to_string(distance));
    if (distance > p.old_goal + 0.00001f && distance < p.path_threshold - 0.00001f) {
      check(all_zero(old.total), "old two-critic gap at " + std::to_string(distance));
      check(current.goal_active, "new goal attraction fills gap at " + std::to_string(distance));
    }
  }
  // The reverse case deliberately uses a path ending behind the car. There is
  // no forward-preference penalty; a legitimate reverse approach stays valid.
  auto behind = score_case(p, -1.2f, p.new_goal);
  std::cout << ",{\"signed_goal_x_m\":-1.2,\"new\":";
  print_score(behind);
  std::cout << "}],";
  check(behind.total[2] < behind.total[1] && behind.total[1] < behind.total[0],
    "goal behind prefers reverse over stationary over forward");
  for (float threshold : {p.old_goal, p.new_goal}) {
    auto inside = score_case(p, threshold - 0.0001f, threshold);
    auto outside = score_case(p, threshold + 0.0001f, threshold);
    check(inside.goal_active && !outside.goal_active, "goal threshold +/- epsilon");
  }
  auto inside_path = score_case(p, p.path_threshold - 0.0001f, p.new_goal);
  auto outside_path = score_case(p, p.path_threshold + 0.0001f, p.new_goal);
  check(!inside_path.path_active && outside_path.path_active, "path threshold +/- epsilon");
  auto far = score_case(p, 3.0f, p.new_goal);
  check(!far.goal_active && far.path_active && !all_zero(far.path), "far path-follow remains active");
  size_t old_gap = 0, new_gap = 0;
  for (int mm = 501; mm <= 4000; ++mm) {
    auto old = score_case(p, mm / 1000.0f, p.old_goal);
    auto current = score_case(p, mm / 1000.0f, p.new_goal);
    old_gap += all_zero(old.total);
    new_gap += all_zero(current.total);
  }
  check(old_gap > 0 && new_gap == 0, "0.501-4m sweep has no new two-critic attraction gap");
  std::cout << "\"sweep\":{\"samples\":3500,\"old_zero_cost_samples\":" << old_gap
            << ",\"new_zero_cost_samples\":" << new_gap << "},\"checks\":" << checks
            << ",\"passed\":" << (failed.empty() ? "true" : "false") << ",\"failures\":[";
  for (size_t i = 0; i < failed.size(); ++i) {
    if (i) {std::cout << ',';}
    std::cout << '"' << failed[i] << '"';
  }
  std::cout << "]}\n";
  return failed.empty() ? 0 : 1;
}
