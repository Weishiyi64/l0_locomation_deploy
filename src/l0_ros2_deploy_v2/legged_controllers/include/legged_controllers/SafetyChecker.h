#pragma once

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

namespace legged {

class SafetyChecker {
 public:
  struct Params {
    double roll_limit{0.5235987755982988};
    double pitch_limit{0.5235987755982988};
    int consecutive_violation_count{3};
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

 private:
  Params p_;
};
}  // namespace legged
