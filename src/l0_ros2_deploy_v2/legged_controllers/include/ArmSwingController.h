#pragma once

#include <cstddef>
#include <rclcpp/logger.hpp>
#include <string>
#include <vector>

namespace legged {

class ArmSwingController {
 public:
  enum class Mode { kDefault = 0, kLow = 1, kHigh = 2 };

  struct Params {
    bool enabled{true};
    double stop_vx_threshold{0.05};
    double high_vx_threshold{0.8};
    double vx_hysteresis{0.03};
    double blend_sec{0.35};
    double kp{4.2};
    double kd{0.2};

    double low_period_sec{1.0};
    double low_pitch_amplitude{0.567};
    double low_left_shoulder_roll{-0.927};
    double low_left_elbow{0.298};
    double low_right_shoulder_roll{0.927};
    double low_right_elbow{-0.297};

    double high_period_sec{0.5};
    double high_left_shoulder_pitch{0.0};
    double high_right_shoulder_pitch{0.0};
    double high_roll_center{0.6265};
    double high_roll_amplitude{0.3005};
    double high_left_elbow{0.199};
    double high_right_elbow{-0.199};
  };

  ArmSwingController() = default;

  void initJoints(const std::vector<std::string>& joint_names, rclcpp::Logger logger);
  void configure(const Params& params, rclcpp::Logger logger);
  void reset();

  void update(double wall_sec, double vx, bool allow_control, const std::vector<double>& default_q);

  bool getJointCmd(size_t global_j, double& q, double& dq, double& kp, double& kd, double& tau_ff) const;

  Mode targetMode() const { return target_mode_; }
  Mode activeMode() const { return active_mode_; }
  bool outputActive() const { return output_active_; }
  size_t mappedJointCount() const { return mapped_joint_count_; }

 private:
  static constexpr size_t kInvalidIndex = static_cast<size_t>(-1);

  struct JointMap {
    size_t left_shoulder_pitch{kInvalidIndex};
    size_t left_shoulder_roll{kInvalidIndex};
    size_t left_elbow{kInvalidIndex};
    size_t right_shoulder_pitch{kInvalidIndex};
    size_t right_shoulder_roll{kInvalidIndex};
    size_t right_elbow{kInvalidIndex};
  };

  static const char* modeName(Mode mode);
  static double smoothStep01(double x);
  static double blend(double from, double to, double alpha);

  Mode selectMode(double vx_abs) const;
  void beginTransition(Mode next_mode, double wall_sec, const std::vector<double>& default_q);
  void applyInstantMode(Mode next_mode, double wall_sec, const std::vector<double>& default_q);
  void updateBlend(double wall_sec, const std::vector<double>& default_q);
  void sampleMode(Mode mode, double t_sec, const std::vector<double>& default_q, std::vector<double>& out_q) const;
  void samplePlaying(double wall_sec, const std::vector<double>& default_q);

  bool validIndex(size_t index) const { return index != kInvalidIndex && index < mapped_joint_.size(); }
  void markMapped(size_t index);
  size_t findJoint(const std::vector<std::string>& joint_names, const std::string& name) const;

  Params params_;
  rclcpp::Logger logger_{rclcpp::get_logger("arm_swing")};
  JointMap joints_;
  std::vector<bool> mapped_joint_;
  size_t mapped_joint_count_{0};

  Mode target_mode_{Mode::kDefault};
  Mode active_mode_{Mode::kDefault};
  Mode blend_to_mode_{Mode::kDefault};
  bool blending_{false};
  bool output_active_{false};
  double blend_start_wall_sec_{0.0};
  double play_anchor_wall_sec_{0.0};

  std::vector<double> q_cmd_;
  std::vector<double> blend_start_q_;
  std::vector<double> blend_target_q_;
};

}  // namespace legged
