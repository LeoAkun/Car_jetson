#ifndef LIO_SAM__VEHICLE_STATIONARY_HISTORY_HPP_
#define LIO_SAM__VEHICLE_STATIONARY_HISTORY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>

namespace lio_sam
{

struct VehicleStationaryDecision
{
  bool stationary{false};
  int64_t sample_stamp_ns{0};
  double speed_kmh{std::numeric_limits<double>::quiet_NaN()};
  const char * reason{"disabled"};
};

class VehicleStationaryHistory
{
public:
  void add(int64_t stamp_ns, double speed_kmh)
  {
    if (stamp_ns <= 0) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::lower_bound(
      samples_.begin(), samples_.end(), stamp_ns,
      [](const Sample & sample, int64_t value) {return sample.stamp_ns < value;});
    if (it != samples_.end() && it->stamp_ns == stamp_ns) {
      if (it->speed_kmh != speed_kmh) {
        it->speed_kmh = std::numeric_limits<double>::quiet_NaN();
      }
    } else {
      samples_.insert(it, Sample{stamp_ns, speed_kmh});
    }
    while (samples_.size() > kMaxSamples) {
      samples_.pop_front();
    }
  }

  VehicleStationaryDecision evaluate(
    int64_t correction_stamp_ns,
    double timeout_sec,
    int min_samples,
    double stationary_speed_threshold_kmh) const
  {
    VehicleStationaryDecision decision;
    if (correction_stamp_ns <= 0 || !std::isfinite(timeout_sec) || timeout_sec < 0.0) {
      decision.reason = "invalid correction time or timeout";
      return decision;
    }
    if (min_samples < 1 || !std::isfinite(stationary_speed_threshold_kmh) ||
      stationary_speed_threshold_kmh < 0.0)
    {
      decision.reason = "invalid stationary policy";
      return decision;
    }

    const double max_age_ns = timeout_sec * 1e9;
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::upper_bound(
      samples_.begin(), samples_.end(), correction_stamp_ns,
      [](int64_t value, const Sample & sample) {return value < sample.stamp_ns;});
    if (it == samples_.begin()) {
      decision.reason = "no vehicle status at or before correction";
      return decision;
    }

    --it;
    decision.sample_stamp_ns = it->stamp_ns;
    decision.speed_kmh = it->speed_kmh;
    const auto sample_age_ns = correction_stamp_ns - it->stamp_ns;
    if (sample_age_ns > max_age_ns || sample_age_ns > kMaxSampleGapNs) {
      decision.reason = "latest vehicle status is stale";
      return decision;
    }

    int accepted = 0;
    int64_t previous_stamp_ns = correction_stamp_ns;
    while (true) {
      if (correction_stamp_ns - it->stamp_ns > max_age_ns ||
        previous_stamp_ns - it->stamp_ns > kMaxSampleGapNs)
      {
        decision.reason = "vehicle status history has a gap";
        return decision;
      }
      if (!std::isfinite(it->speed_kmh)) {
        decision.reason = "vehicle speed is invalid";
        return decision;
      }
      if (std::abs(it->speed_kmh) > stationary_speed_threshold_kmh) {
        decision.reason = "vehicle was moving at correction time";
        return decision;
      }
      ++accepted;
      previous_stamp_ns = it->stamp_ns;
      if (accepted >= min_samples) {
        decision.stationary = true;
        decision.reason = "fresh consecutive stationary vehicle status";
        return decision;
      }
      if (it == samples_.begin()) {
        break;
      }
      --it;
    }

    decision.reason = "not enough consecutive stationary vehicle status";
    return decision;
  }

private:
  struct Sample
  {
    int64_t stamp_ns;
    double speed_kmh;
  };

  static constexpr std::size_t kMaxSamples = 4096;
  // A gap of ten nominal 100 Hz status periods interrupts stationary evidence.
  static constexpr int64_t kMaxSampleGapNs = 100000000;
  mutable std::mutex mutex_;
  std::deque<Sample> samples_;
};

}  // namespace lio_sam

#endif  // LIO_SAM__VEHICLE_STATIONARY_HISTORY_HPP_
