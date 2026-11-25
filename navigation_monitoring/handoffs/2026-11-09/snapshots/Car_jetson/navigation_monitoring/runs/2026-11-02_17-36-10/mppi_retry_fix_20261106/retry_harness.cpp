// No ROS initialization, node, context, publisher, costmap node or vehicle command.
// Invoke real Optimizer prepare/optimize/fallback and real CriticManager dispatch.
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <xtensor/xrandom.hpp>
#include "nav2_mppi_controller/optimizer.hpp"

struct Probe
{
  size_t calls{0}, tail_calls{0}, fail_calls{0}, caches_dirty_on_entry{0}, rollouts{0};
  size_t failures_remaining{0};
  bool always_fail{false};
};

class CountedMotionModel : public mppi::DiffDriveMotionModel
{
public:
  explicit CountedMotionModel(Probe & probe) : probe_(probe) {}
  void predict(mppi::models::State & state) override
  {
    ++probe_.rollouts;
    mppi::DiffDriveMotionModel::predict(state);
  }
private:
  Probe & probe_;
};

class MarkerGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    const std::string &, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override {return false;}
  bool getTolerances(geometry_msgs::msg::Pose &, geometry_msgs::msg::Twist &) override
  {return false;}
};

class ScriptedFailure : public mppi::critics::CriticFunction
{
public:
  explicit ScriptedFailure(Probe & probe) : probe_(probe) {}
  void initialize() override {}
  void score(mppi::CriticData & data) override
  {
    ++probe_.calls;
    if (data.path_pts_valid || data.furthest_reached_path_point) {
      ++probe_.caches_dirty_on_entry;
    }
    if (probe_.always_fail || probe_.failures_remaining) {
      ++probe_.fail_calls;
      if (probe_.failures_remaining) {--probe_.failures_remaining;}
      data.fail_flag = true;
      data.path_pts_valid = std::vector<bool>{true, false, true};
      data.furthest_reached_path_point = 2;
      // Make failure costs nonuniform so an accidental control update is visible.
      for (size_t i = 0; i < data.costs.size(); ++i) {data.costs(i) = 100.0f + i;}
    }
  }
private:
  Probe & probe_;
};

class CountTail : public mppi::critics::CriticFunction
{
public:
  explicit CountTail(Probe & probe) : probe_(probe) {}
  void initialize() override {}
  void score(mppi::CriticData & data) override
  {
    ++probe_.tail_calls;
    data.costs += 1.0f;
  }
private:
  Probe & probe_;
};

class ManagerAccess : public mppi::CriticManager
{
public:
  // A public pointer to a protected base member permits access to the actual
  // embedded manager without casts, layout assumptions or macro access changes.
  static auto critics_member() {return &ManagerAccess::critics_;}
};

class OfflineOptimizer : public mppi::Optimizer
{
public:
  explicit OfflineOptimizer(Probe & probe, size_t retry_limit = 1, unsigned iterations = 1)
  {
    settings_.batch_size = 32;
    settings_.time_steps = 12;
    settings_.iteration_count = iterations;
    settings_.model_dt = 0.05f;
    settings_.temperature = 0.3f;
    settings_.gamma = 0.015f;
    settings_.sampling_std = {0.2f, 0.2f, 0.3f};
    settings_.base_constraints = {0.8f, -0.8f, 0.0f, 1.0f};
    settings_.constraints = settings_.base_constraints;
    settings_.retry_attempt_limit = retry_limit;
    motion_model_ = std::make_shared<CountedMotionModel>(probe);
    auto & list = critic_manager_.*ManagerAccess::critics_member();
    list.emplace_back(std::make_unique<ScriptedFailure>(probe));
    list.emplace_back(std::make_unique<CountTail>(probe));
    reset();
  }

  void begin_cycle()
  {
    retries_ = 0;
    robot_pose_.header.frame_id = "map";
    robot_pose_.pose.orientation.w = 1.0;
    robot_pose_.pose.position.x = 3.0;
    robot_pose_.pose.position.y = -2.0;
    robot_speed_.linear.x = 0.1;
    plan_.header.frame_id = "map";
    plan_.poses = {robot_pose_, robot_pose_};
    plan_.poses.back().pose.position.x = 6.0;
    prepare(robot_pose_, robot_speed_, plan_, &checker_);
  }

  void real_optimize() {optimize();}
  bool real_fallback()
  {
#ifdef LEGACY_FALLBACK_SIGNATURE
    return fallback(critics_data_.fail_flag);
#else
    return fallback(critics_data_.fail_flag, retries_);
#endif
  }

  // Same prepare/do-while structure as evalControl, ending before Twist packing
  // (packing consults Costmap2DROS for a frame name and needs a ROS node).
  bool run_cycle()
  {
    begin_cycle();
    size_t guard = 0;
    try {
      do {
        if (++guard > 20) {throw std::logic_error("offline retry guard exceeded");}
        optimize();
      } while (real_fallback());
      return true;
    } catch (const std::runtime_error & error) {
      if (std::string(error.what()) != "Optimizer fail to compute path") {throw;}
      return false;
    }
  }

  void clear_legacy_counter_for_independent_case()
  {
    // Calling the real successful fallback also matches the production end of a
    // successful cycle. This only isolates legacy static-counter test cases.
#ifdef LEGACY_FALLBACK_SIGNATURE
    fallback(false);
#endif
  }
  bool failed() const {return critics_data_.fail_flag;}
  bool caches_empty() const
  {
    return !critics_data_.path_pts_valid && !critics_data_.furthest_reached_path_point;
  }
  void seed_controls() {control_sequence_.vx.fill(0.125f); control_sequence_.wz.fill(0.02f);}
  bool seeded_controls_unchanged() const
  {
    return xt::all(xt::equal(control_sequence_.vx, 0.125f)) &&
           xt::all(xt::equal(control_sequence_.wz, 0.02f));
  }
  void set_dynamic_limit()
  {
    settings_.constraints.vx_max = 0.25f;
    settings_.constraints.vx_min = -0.20f;
    settings_.constraints.wz = 0.4f;
  }
  bool dynamic_limit_preserved() const
  {
    return settings_.constraints.vx_max == 0.25f && settings_.constraints.vx_min == -0.20f &&
           settings_.constraints.wz == 0.4f;
  }
  bool inputs_preserved() const
  {
    return state_.pose.pose.position.x == 3.0 && state_.pose.pose.position.y == -2.0 &&
           state_.speed.linear.x == 0.1 && path_.x.size() == 2 && path_.x(1) == 6.0f &&
           critics_data_.motion_model == motion_model_ && critics_data_.goal_checker == &checker_ &&
           &critics_data_.state == &state_ && &critics_data_.path == &path_ &&
           &critics_data_.costs == &costs_ && &critics_data_.trajectories == &generated_trajectories_;
  }
  bool finite_controls() const
  {
    for (auto value : control_sequence_.vx) {if (!std::isfinite(value)) {return false;}}
    for (auto value : control_sequence_.wz) {if (!std::isfinite(value)) {return false;}}
    return true;
  }
private:
  size_t retries_{0};
  geometry_msgs::msg::PoseStamped robot_pose_;
  geometry_msgs::msg::Twist robot_speed_;
  nav_msgs::msg::Path plan_;
  MarkerGoalChecker checker_;
};

struct Check {std::string label; bool passed;};

int main()
{
  xt::random::seed(20261106);
  std::vector<Check> checks;
  auto check = [&](bool pass, const std::string & label) {checks.push_back({label, pass});};
  {
    Probe probe;
    probe.failures_remaining = 1;
    OfflineOptimizer optimizer(probe);
    optimizer.clear_legacy_counter_for_independent_case();
    check(optimizer.run_cycle(), "first-fail then success completes a cycle");
    check(probe.calls == 2, "real CriticManager invokes failing critic again on retry");
    check(probe.tail_calls == 1, "remaining critic runs on successful retry only");
    check(probe.fail_calls == 1, "only the scripted initial failure occurred");
    check(probe.caches_dirty_on_entry == 0, "retry invalidates path and furthest-point caches");
    check(!optimizer.failed() && optimizer.finite_controls(), "successful retry has clear flag and finite controls");
  }
  {
    Probe probe;
    probe.always_fail = true;
    OfflineOptimizer optimizer(probe, 2);
    optimizer.clear_legacy_counter_for_independent_case();
    check(!optimizer.run_cycle(), "persistent failure still throws at retry limit two");
    check(probe.calls == 3, "persistent failure actually scores initial plus two retries");
    check(probe.tail_calls == 0, "CriticManager stops immediately after each failure");
    probe = Probe{};
    probe.failures_remaining = 1;
    check(optimizer.run_cycle() && probe.calls == 2, "new cycle after exhausted retries has fresh budget");
  }
  {
    Probe probe;
    probe.failures_remaining = 1;
    OfflineOptimizer optimizer(probe, 1, 3);
    optimizer.clear_legacy_counter_for_independent_case();
    optimizer.begin_cycle();
    optimizer.seed_controls();
    optimizer.set_dynamic_limit();
    optimizer.real_optimize();
    check(probe.calls == 1 && probe.tail_calls == 0, "iteration_count three stops scoring after first failure");
    check(probe.rollouts == 1, "failed iteration exits optimize before generating another candidate batch");
    check(optimizer.seeded_controls_unchanged(), "failed scoring does not update the selected control sequence");
    bool scheduled = false;
    try {scheduled = optimizer.real_fallback();} catch (const std::runtime_error &) {}
    check(scheduled, "failed batch schedules its one allowed retry");
    check(!optimizer.failed() && optimizer.caches_empty(), "fallback reset clears failure and stale caches");
    check(optimizer.dynamic_limit_preserved(), "fallback preserves current dynamic speed limits");
    check(optimizer.inputs_preserved(), "retry retains pose speed plan model checker and CriticData references");
    optimizer.real_optimize();
    check(probe.calls == 4 && probe.tail_calls == 3, "retry with three iterations really executes all successful critics");
    bool completed = false;
    try {completed = !optimizer.real_fallback();} catch (const std::runtime_error &) {}
    check(completed, "successful retry exits fallback loop");
  }
  {
    Probe a, b;
    a.failures_remaining = b.failures_remaining = 1;
    OfflineOptimizer first(a), second(b);
    first.clear_legacy_counter_for_independent_case();
    first.begin_cycle(); second.begin_cycle();
    first.real_optimize(); second.real_optimize();
    bool first_allowed = false, second_allowed = false;
    try {first_allowed = first.real_fallback();} catch (const std::runtime_error &) {}
    try {second_allowed = second.real_fallback();} catch (const std::runtime_error &) {}
    check(first_allowed && second_allowed, "two optimizer cycles have independent retry budgets");
    first.real_optimize(); second.real_optimize();
    check(a.calls == 2 && b.calls == 2, "both interleaved cycles rescore after initial failure");
    first.clear_legacy_counter_for_independent_case();
  }
  {
    Probe probe;
    probe.failures_remaining = 1;
    OfflineOptimizer optimizer(probe, 1);
    optimizer.clear_legacy_counter_for_independent_case();
    optimizer.begin_cycle(); optimizer.real_optimize();
    try {optimizer.real_fallback();} catch (const std::runtime_error &) {}
    // Abandon that cycle and begin a new one before the previous retry finishes.
    probe = Probe{}; probe.failures_remaining = 1;
    check(optimizer.run_cycle() && probe.calls == 2, "new cycle does not inherit an abandoned cycle retry count");
    optimizer.clear_legacy_counter_for_independent_case();
  }
  {
    Probe probe;
    probe.always_fail = true;
    OfflineOptimizer optimizer(probe, 0);
    optimizer.clear_legacy_counter_for_independent_case();
    check(!optimizer.run_cycle() && probe.calls == 1, "zero retry limit never schedules another batch");
  }
  size_t passed = 0;
  std::cout << "{\"checks\":[";
  for (size_t i = 0; i < checks.size(); ++i) {
    if (i) {std::cout << ',';}
    std::cout << "{\"label\":\"" << checks[i].label << "\",\"passed\":"
              << (checks[i].passed ? "true" : "false") << '}';
    passed += checks[i].passed;
  }
  std::cout << "],\"total\":" << checks.size() << ",\"passed\":" << passed
            << ",\"failed\":" << checks.size() - passed << "}\n";
  return passed == checks.size() ? 0 : 1;
}
