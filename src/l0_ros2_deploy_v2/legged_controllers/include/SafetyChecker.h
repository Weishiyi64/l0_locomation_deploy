#pragma once

#include <algorithm>
#include <cmath>
#include <mutex>

namespace legged {

class SafetyChecker {
 public:
  struct Params {
    double roll_limit{0.5235987755982988};
    double pitch_limit{0.5235987755982988};
    int consecutive_violation_count{50};
  };

  explicit SafetyChecker(const Params& p) : p_(p) {
    const Params defaults;
    if (p_.roll_limit <= 0.0) p_.roll_limit = defaults.roll_limit;
    if (p_.pitch_limit <= 0.0) p_.pitch_limit = defaults.pitch_limit;
    if (p_.consecutive_violation_count <= 0) p_.consecutive_violation_count = defaults.consecutive_violation_count;
  }

  struct OrientationResult {
    bool ok{true};
    bool orientation_violation{false};
  };

  // Check orientation only (roll and pitch)
  OrientationResult checkOrientation(double roll, double pitch) const {
    OrientationResult r;
    if (roll < -p_.roll_limit || roll > p_.roll_limit || pitch < -p_.pitch_limit || pitch > p_.pitch_limit)
    {
      r.ok = false;
      r.orientation_violation = true;
    }
    return r;
  }

  int requiredConsecutiveOrientationViolations() const { return p_.consecutive_violation_count; }

  struct OrientationState {
    double roll{0.0};
    double pitch{0.0};
    int consecutive_violations{0};
  };

  void setOrientationMonitoring(bool enabled) {
    std::lock_guard<std::mutex> lock(orientation_mutex_);
    orientation_monitoring_ = enabled;
    if (!enabled) orientation_state_ = OrientationState{};
  }

  // Called once per accepted, normalized IMU frame, including frames between control ticks.
  void observeImuOrientation(double x, double y, double z, double w) {
    std::lock_guard<std::mutex> lock(orientation_mutex_);
    // Latch the threshold frame until the controller handles it, even if a normal frame follows.
    if (!orientation_monitoring_ || orientation_state_.consecutive_violations >= p_.consecutive_violation_count) return;

    orientation_state_.roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    orientation_state_.pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (w * y - z * x))));
    if (checkOrientation(orientation_state_.roll, orientation_state_.pitch).ok)
      orientation_state_.consecutive_violations = 0;
    else
      ++orientation_state_.consecutive_violations;
  }

  OrientationState orientationState() const {
    std::lock_guard<std::mutex> lock(orientation_mutex_);
    return orientation_state_;
  }

 private:
  Params p_;
  mutable std::mutex orientation_mutex_;
  bool orientation_monitoring_{false};
  OrientationState orientation_state_;
};
}  // namespace legged
