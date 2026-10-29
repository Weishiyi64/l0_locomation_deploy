#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace legged {

struct JointCommandSample {
  double q{0.0};
  double dq{0.0};
  double kp{0.0};
  double kd{0.0};
  double tau_ff{0.0};
};

class JointCommandTransition {
 public:
  void resize(size_t num_joints) {
    active_joint_.assign(num_joints, false);
    start_cmd_.assign(num_joints, JointCommandSample());
  }

  void clear() {
    active_ = false;
    start_wall_sec_ = 0.0;
    duration_sec_ = 0.0;
    std::fill(active_joint_.begin(), active_joint_.end(), false);
  }

  bool start(const std::vector<bool>& joint_mask, const std::vector<JointCommandSample>& start_cmd, double wall_sec,
             double duration_sec) {
    clear();
    duration_sec_ = std::max(duration_sec, 0.0);
    if (duration_sec_ <= 1e-9 || joint_mask.empty()) return false;

    const size_t n = std::min(joint_mask.size(), start_cmd.size());
    if (active_joint_.size() != joint_mask.size()) resize(joint_mask.size());
    size_t count = 0;
    for (size_t j = 0; j < n; ++j)
    {
      if (!joint_mask[j]) continue;
      active_joint_[j] = true;
      start_cmd_[j] = start_cmd[j];
      ++count;
    }

    if (count == 0)
    {
      clear();
      return false;
    }

    active_ = true;
    start_wall_sec_ = wall_sec;
    return true;
  }

  bool active() const { return active_; }
  double duration() const { return duration_sec_; }
  size_t activeCount() const {
    return static_cast<size_t>(std::count(active_joint_.begin(), active_joint_.end(), true));
  }

  bool expired(double wall_sec) const {
    return !active_ || duration_sec_ <= 1e-9 || (wall_sec - start_wall_sec_) >= duration_sec_;
  }

  void clearIfExpired(double wall_sec) {
    if (expired(wall_sec)) clear();
  }

  bool apply(size_t global_j, double wall_sec, double& q, double& dq, double& kp, double& kd, double& tau_ff) const {
    if (!active_ || global_j >= active_joint_.size() || !active_joint_[global_j] || duration_sec_ <= 1e-9)
    {
      return false;
    }

    const double elapsed = std::max(wall_sec - start_wall_sec_, 0.0);
    const double alpha = smoothStep01(elapsed / duration_sec_);
    const JointCommandSample& from = start_cmd_[global_j];
    q = blend(from.q, q, alpha);
    dq = blend(from.dq, dq, alpha);
    kp = blend(from.kp, kp, alpha);
    kd = blend(from.kd, kd, alpha);
    tau_ff = blend(from.tau_ff, tau_ff, alpha);
    return true;
  }

 private:
  static double smoothStep01(double x) {
    const double u = std::max(0.0, std::min(1.0, x));
    return u * u * (3.0 - 2.0 * u);
  }

  static double blend(double from, double to, double alpha) { return from + alpha * (to - from); }

  bool active_{false};
  double start_wall_sec_{0.0};
  double duration_sec_{0.0};
  std::vector<bool> active_joint_;
  std::vector<JointCommandSample> start_cmd_;
};

}  // namespace legged
