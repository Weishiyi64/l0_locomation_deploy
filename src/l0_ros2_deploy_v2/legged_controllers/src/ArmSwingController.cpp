#include "ArmSwingController.h"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

namespace legged {

namespace {

constexpr double kPi = 3.14159265358979323846;

double positiveOrDefault(double value, double fallback) { return value > 1e-9 ? value : fallback; }

}  // namespace

const char* ArmSwingController::modeName(Mode mode) {
  switch (mode)
  {
    case Mode::kDefault:
      return "default";
    case Mode::kLow:
      return "low";
    case Mode::kHigh:
      return "high";
  }
  return "unknown";
}

double ArmSwingController::smoothStep01(double x) {
  const double u = std::max(0.0, std::min(1.0, x));
  return u * u * (3.0 - 2.0 * u);
}

double ArmSwingController::blend(double from, double to, double alpha) { return from + alpha * (to - from); }

void ArmSwingController::initJoints(const std::vector<std::string>& joint_names, rclcpp::Logger logger) {
  logger_ = logger;
  joints_ = JointMap();
  mapped_joint_.assign(joint_names.size(), false);
  mapped_joint_count_ = 0;
  q_cmd_.assign(joint_names.size(), 0.0);
  blend_start_q_.assign(joint_names.size(), 0.0);
  blend_target_q_.assign(joint_names.size(), 0.0);

  joints_.left_shoulder_pitch = findJoint(joint_names, "left_shoulder_pitch_joint");
  joints_.left_shoulder_roll = findJoint(joint_names, "left_shoulder_roll_joint");
  joints_.left_elbow = findJoint(joint_names, "left_elbow_roll_joint");
  if (joints_.left_elbow == kInvalidIndex) joints_.left_elbow = findJoint(joint_names, "left_elbow_joint");

  joints_.right_shoulder_pitch = findJoint(joint_names, "right_shoulder_pitch_joint");
  joints_.right_shoulder_roll = findJoint(joint_names, "right_shoulder_roll_joint");
  joints_.right_elbow = findJoint(joint_names, "right_elbow_roll_joint");
  if (joints_.right_elbow == kInvalidIndex) joints_.right_elbow = findJoint(joint_names, "right_elbow_joint");

  markMapped(joints_.left_shoulder_pitch);
  markMapped(joints_.left_shoulder_roll);
  markMapped(joints_.left_elbow);
  markMapped(joints_.right_shoulder_pitch);
  markMapped(joints_.right_shoulder_roll);
  markMapped(joints_.right_elbow);

  reset();

  if (mapped_joint_count_ == 0)
  {
    RCLCPP_WARN(logger_, "[ArmSwing] No L0-style arm joints found; automatic arm swing disabled.");
  }
  else
  {
    RCLCPP_INFO(logger_, "[ArmSwing] Mapped %zu arm joint(s) for procedural walking swing.", mapped_joint_count_);
  }
}

void ArmSwingController::configure(const Params& params, rclcpp::Logger logger) {
  logger_ = logger;
  params_ = params;

  params_.stop_vx_threshold = std::max(params_.stop_vx_threshold, 0.0);
  params_.high_vx_threshold = std::max(params_.high_vx_threshold, params_.stop_vx_threshold + 1e-6);
  params_.vx_hysteresis = std::max(params_.vx_hysteresis, 0.0);
  params_.blend_sec = std::max(params_.blend_sec, 0.0);
  params_.low_period_sec = positiveOrDefault(params_.low_period_sec, Params().low_period_sec);
  params_.high_period_sec = positiveOrDefault(params_.high_period_sec, Params().high_period_sec);

  RCLCPP_INFO(logger_,
              "[ArmSwing] enabled=%d stop_vx=%.3f high_vx=%.3f hysteresis=%.3f blend=%.3f kp=%.2f kd=%.2f "
              "low_period=%.3f high_period=%.3f",
              params_.enabled, params_.stop_vx_threshold, params_.high_vx_threshold, params_.vx_hysteresis,
              params_.blend_sec, params_.kp, params_.kd, params_.low_period_sec, params_.high_period_sec);
}

void ArmSwingController::reset() {
  target_mode_ = Mode::kDefault;
  active_mode_ = Mode::kDefault;
  blend_to_mode_ = Mode::kDefault;
  blending_ = false;
  output_active_ = false;
  blend_start_wall_sec_ = 0.0;
  play_anchor_wall_sec_ = 0.0;
  std::fill(q_cmd_.begin(), q_cmd_.end(), 0.0);
  std::fill(blend_start_q_.begin(), blend_start_q_.end(), 0.0);
  std::fill(blend_target_q_.begin(), blend_target_q_.end(), 0.0);
}

void ArmSwingController::update(double wall_sec, double vx, bool allow_control, const std::vector<double>& default_q) {
  if (!params_.enabled || !allow_control || mapped_joint_count_ == 0)
  {
    reset();
    return;
  }

  const Mode requested_mode = selectMode(std::abs(vx));

  if (requested_mode != target_mode_)
  {
    beginTransition(requested_mode, wall_sec, default_q);
  }

  if (blending_)
  {
    updateBlend(wall_sec, default_q);
    return;
  }

  if (active_mode_ != Mode::kDefault)
  {
    samplePlaying(wall_sec, default_q);
    return;
  }

  output_active_ = false;
}

bool ArmSwingController::getJointCmd(size_t global_j, double& q, double& dq, double& kp, double& kd,
                                     double& tau_ff) const {
  if (!output_active_ || global_j >= mapped_joint_.size() || !mapped_joint_[global_j]) return false;

  q = global_j < q_cmd_.size() ? q_cmd_[global_j] : 0.0;
  dq = 0.0;
  kp = params_.kp;
  kd = params_.kd;
  tau_ff = 0.0;
  return true;
}

ArmSwingController::Mode ArmSwingController::selectMode(double vx_abs) const {
  const double stop = params_.stop_vx_threshold;
  const double high = params_.high_vx_threshold;
  const double hyst = params_.vx_hysteresis;

  switch (target_mode_)
  {
    case Mode::kHigh:
      if (vx_abs < std::max(stop - hyst, 0.0)) return Mode::kDefault;
      if (vx_abs < std::max(high - hyst, stop)) return Mode::kLow;
      return Mode::kHigh;
    case Mode::kLow:
      if (vx_abs < std::max(stop - hyst, 0.0)) return Mode::kDefault;
      if (vx_abs >= high) return Mode::kHigh;
      return Mode::kLow;
    case Mode::kDefault:
      if (vx_abs >= high) return Mode::kHigh;
      if (vx_abs >= stop) return Mode::kLow;
      return Mode::kDefault;
  }
  return Mode::kDefault;
}

void ArmSwingController::beginTransition(Mode next_mode, double wall_sec, const std::vector<double>& default_q) {
  if (!output_active_)
  {
    for (size_t j = 0; j < q_cmd_.size(); ++j)
    {
      q_cmd_[j] = j < default_q.size() ? default_q[j] : 0.0;
    }
  }

  target_mode_ = next_mode;
  blend_to_mode_ = next_mode;

  if (params_.blend_sec <= 1e-9)
  {
    applyInstantMode(next_mode, wall_sec, default_q);
    return;
  }

  blend_start_q_ = q_cmd_;
  blend_start_wall_sec_ = wall_sec;
  blending_ = true;
  output_active_ = true;
  updateBlend(wall_sec, default_q);
}

void ArmSwingController::applyInstantMode(Mode next_mode, double wall_sec, const std::vector<double>& default_q) {
  blending_ = false;
  active_mode_ = next_mode;
  if (active_mode_ == Mode::kDefault)
  {
    output_active_ = false;
    return;
  }

  play_anchor_wall_sec_ = wall_sec;
  sampleMode(active_mode_, 0.0, default_q, q_cmd_);
  output_active_ = true;
}

void ArmSwingController::updateBlend(double wall_sec, const std::vector<double>& default_q) {
  const double elapsed = std::max(wall_sec - blend_start_wall_sec_, 0.0);
  const double alpha = smoothStep01(elapsed / std::max(params_.blend_sec, 1e-9));

  sampleMode(blend_to_mode_, 0.0, default_q, blend_target_q_);
  const size_t n = std::min(q_cmd_.size(), blend_start_q_.size());
  for (size_t j = 0; j < n; ++j)
  {
    if (!mapped_joint_[j]) continue;
    q_cmd_[j] = blend(blend_start_q_[j], blend_target_q_[j], alpha);
  }
  output_active_ = true;

  if (elapsed + 1e-9 < params_.blend_sec) return;

  blending_ = false;
  active_mode_ = blend_to_mode_;
  if (active_mode_ == Mode::kDefault)
  {
    output_active_ = false;
    return;
  }

  play_anchor_wall_sec_ = wall_sec;
  sampleMode(active_mode_, 0.0, default_q, q_cmd_);
  output_active_ = true;
}

void ArmSwingController::sampleMode(Mode mode, double t_sec, const std::vector<double>& default_q,
                                    std::vector<double>& out_q) const {
  if (out_q.size() != mapped_joint_.size()) out_q.assign(mapped_joint_.size(), 0.0);
  for (size_t j = 0; j < out_q.size(); ++j)
  {
    out_q[j] = j < default_q.size() ? default_q[j] : 0.0;
  }

  if (mode == Mode::kDefault) return;

  if (mode == Mode::kLow)
  {
    const double phase = 2.0 * kPi * t_sec / params_.low_period_sec;
    if (validIndex(joints_.left_shoulder_pitch))
      out_q[joints_.left_shoulder_pitch] = -params_.low_pitch_amplitude * std::cos(phase);
    if (validIndex(joints_.right_shoulder_pitch))
      out_q[joints_.right_shoulder_pitch] = params_.low_pitch_amplitude * std::cos(phase);
    if (validIndex(joints_.left_shoulder_roll)) out_q[joints_.left_shoulder_roll] = params_.low_left_shoulder_roll;
    if (validIndex(joints_.right_shoulder_roll)) out_q[joints_.right_shoulder_roll] = params_.low_right_shoulder_roll;
    if (validIndex(joints_.left_elbow)) out_q[joints_.left_elbow] = params_.low_left_elbow;
    if (validIndex(joints_.right_elbow)) out_q[joints_.right_elbow] = params_.low_right_elbow;
    return;
  }

  const double phase = 2.0 * kPi * t_sec / params_.high_period_sec;
  if (validIndex(joints_.left_shoulder_pitch)) out_q[joints_.left_shoulder_pitch] = params_.high_left_shoulder_pitch;
  if (validIndex(joints_.right_shoulder_pitch)) out_q[joints_.right_shoulder_pitch] = params_.high_right_shoulder_pitch;
  if (validIndex(joints_.left_shoulder_roll))
    out_q[joints_.left_shoulder_roll] = -params_.high_roll_center - params_.high_roll_amplitude * std::cos(phase);
  if (validIndex(joints_.right_shoulder_roll))
    out_q[joints_.right_shoulder_roll] = params_.high_roll_center + params_.high_roll_amplitude * std::cos(phase);
  if (validIndex(joints_.left_elbow)) out_q[joints_.left_elbow] = params_.high_left_elbow;
  if (validIndex(joints_.right_elbow)) out_q[joints_.right_elbow] = params_.high_right_elbow;
}

void ArmSwingController::samplePlaying(double wall_sec, const std::vector<double>& default_q) {
  const double t = std::max(wall_sec - play_anchor_wall_sec_, 0.0);
  sampleMode(active_mode_, t, default_q, q_cmd_);
  output_active_ = active_mode_ != Mode::kDefault;
}

void ArmSwingController::markMapped(size_t index) {
  if (!validIndex(index) || mapped_joint_[index]) return;
  mapped_joint_[index] = true;
  ++mapped_joint_count_;
}

size_t ArmSwingController::findJoint(const std::vector<std::string>& joint_names, const std::string& name) const {
  const auto it = std::find(joint_names.begin(), joint_names.end(), name);
  if (it == joint_names.end()) return kInvalidIndex;
  return static_cast<size_t>(std::distance(joint_names.begin(), it));
}

}  // namespace legged
