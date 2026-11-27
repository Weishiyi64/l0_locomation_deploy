#include "FaceFollow.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace legged {
namespace {

constexpr double kImageWidth = 640.0;
constexpr double kImageHeight = 480.0;
constexpr double kDeadZone = 0.05;
constexpr double kScale = 0.188;
constexpr int kInvalidFrameThreshold = 10;
constexpr double kReturnToNeutralSec = 1.0;

std::array<double, 3> readHead(const std::function<std::array<double, 3>()>& reader) {
  return reader ? reader() : std::array<double, 3>{{0.0, 0.0, 0.0}};
}

}  // namespace

void FaceFollowHeadOutput::initJoints(const std::vector<std::string>& joint_names, rclcpp::Logger logger) {
  jointIndices_.clear();

  const auto find_axis = [&joint_names](const char* axis) -> size_t {
    const std::string neck_axis = std::string("neck_") + axis;
    const std::string head_axis = std::string("head_") + axis;
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      if (joint_names[i].find(neck_axis) != std::string::npos || joint_names[i].find(head_axis) != std::string::npos)
      {
        return i;
      }
    }
    return joint_names.size();
  };

  for (const char* axis : {"yaw", "pitch", "roll"})
  {
    const size_t index = find_axis(axis);
    if (index < joint_names.size()) jointIndices_.push_back(index);
  }
  cmd_.assign(jointIndices_.size(), 0.0);
  cmd_dq_.assign(jointIndices_.size(), 0.0);

  if (!jointIndices_.empty())
  {
    RCLCPP_INFO(logger, "Head controller: %zu head/neck joint(s) found (yaw/pitch/roll axis order).",
                jointIndices_.size());
  }
}

void FaceFollowHeadOutput::setTopicBlendSec(double sec) { topic_blend_sec_ = std::max(0.002, std::min(0.5, sec)); }

void FaceFollowHeadOutput::seedCurrentPose(double t_now, double yaw, double pitch, double roll) {
  std::lock_guard<std::mutex> lock(mutex_);
  p_cmd_rpy_[0] = yaw;
  p_cmd_rpy_[1] = pitch;
  p_cmd_rpy_[2] = roll;
  // Mark as ready so the next setTarget takes the normal branch and uses these
  // as wp_start_, not the new target.
  wp_start_ = {t_now, yaw, pitch, roll};
  wp_end_ = {t_now, yaw, pitch, roll};
  wp_ready_ = true;
  returning_to_neutral_ = false;
}

void FaceFollowHeadOutput::setTarget(double t_recv, double yaw_target, double pitch_target) {
  std::lock_guard<std::mutex> lock(mutex_);

  const double span = std::max(0.002, std::min(0.5, topic_blend_sec_));

  if (!wp_ready_)
  {
    // First message: initialise both endpoints at the target; no motion on first cycle.
    wp_start_ = {t_recv, yaw_target, pitch_target, p_cmd_rpy_[2]};
    wp_end_ = {t_recv + span, yaw_target, pitch_target, p_cmd_rpy_[2]};
    p_cmd_rpy_[0] = yaw_target;
    p_cmd_rpy_[1] = pitch_target;
    wp_ready_ = true;
  }
  else
  {
    // Always blend from the current commanded pose toward the latest target over fixed wall time.
    wp_start_ = {t_recv, p_cmd_rpy_[0], p_cmd_rpy_[1], p_cmd_rpy_[2]};
    wp_end_ = {t_recv + span, yaw_target, pitch_target, p_cmd_rpy_[2]};
  }
}

void FaceFollowHeadOutput::stepHeadOutput(double t_wall_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!topicEnabled_) return;

  if (!topic_signal_valid_ && !returning_to_neutral_)
  {
    // Start once per loss of tracking; repeated invalid frames must not restart the return.
    wp_start_ = {t_wall_sec, p_cmd_rpy_[0], p_cmd_rpy_[1], p_cmd_rpy_[2]};
    wp_end_ = {t_wall_sec + kReturnToNeutralSec, 0.0, 0.0, 0.0};
    wp_ready_ = true;
    returning_to_neutral_ = true;
  }

  std::fill(cmd_dq_.begin(), cmd_dq_.end(), 0.0);
  if (wp_ready_)
  {
    const double span = wp_end_.t - wp_start_.t;
    if (span > 1e-9)
    {
      const double u = std::max(0.0, std::min(1.0, (t_wall_sec - wp_start_.t) / span));
      const double blend = topic_signal_valid_ ? u : u * u * (3.0 - 2.0 * u);
      const double rate = topic_signal_valid_ ? (u < 1.0 ? 1.0 / span : 0.0) : 6.0 * u * (1.0 - u) / span;
      const std::array<double, 3> start{{wp_start_.yaw, wp_start_.pitch, wp_start_.roll}};
      const std::array<double, 3> end{{wp_end_.yaw, wp_end_.pitch, wp_end_.roll}};
      for (size_t i = 0; i < 3; ++i)
      {
        const double delta = end[i] - start[i];
        p_cmd_rpy_[i] = start[i] + blend * delta;
        if (i < cmd_dq_.size()) cmd_dq_[i] = rate * delta;
      }
    }
  }
  if (topic_signal_valid_)
  {
    p_cmd_rpy_[2] = 0.0;
    if (cmd_dq_.size() >= 3) cmd_dq_[2] = 0.0;
  }
  for (size_t i = 0; i < cmd_.size(); ++i) cmd_[i] = p_cmd_rpy_[i];
}

bool FaceFollowHeadOutput::getJointCmd(size_t global_j, double& q, double& dq, double& kp, double& kd,
                                       double& tau_ff) const {
  if (jointIndices_.empty() || !topicEnabled_) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < jointIndices_.size(); ++i)
  {
    if (jointIndices_[i] == global_j && i < cmd_.size() && i < cmd_dq_.size())
    {
      q = cmd_[i];
      dq = cmd_dq_[i];
      kp = kd = tau_ff = 0.0;
      return true;
    }
  }
  return false;
}

bool FaceFollowController::parsePayload(const std::string& payload, Target& target, std::string& error) const {
  target = Target{};
  error.clear();
  try
  {
    const YAML::Node root = YAML::Load(payload);
    if (!root.IsMap() || !root["tracking"] || !root["visible"])
      throw std::runtime_error("required fields tracking/visible are missing");
    if (!root["tracking"].as<bool>() || !root["visible"].as<bool>()) return true;

    const YAML::Node offset = root["offset"];
    if (!offset.IsMap() || !offset["horizontal"] || !offset["vertical"])
      throw std::runtime_error("required offset.horizontal/vertical fields are missing");
    const double horizontal = offset["horizontal"].as<double>();
    const double vertical = offset["vertical"].as<double>();
    if (!std::isfinite(horizontal) || !std::isfinite(vertical))
      throw std::runtime_error("offset values must be finite numbers");

    target.x = std::max(0.0, std::min(1.0, 0.5 - horizontal / kImageWidth));
    target.y = std::max(0.0, std::min(1.0, 0.5 + vertical / kImageHeight));
    target.visible = true;
    return true;
  } catch (const std::exception& exception)
  {
    error = exception.what();
    return false;
  }
}

void FaceFollowController::setAllowed(bool allowed, double wall_sec, FaceFollowHeadOutput& head,
                                      const std::function<std::array<double, 3>()>& read_head) {
  std::lock_guard<std::mutex> lock(mutex_);
  setAllowedLocked(allowed, wall_sec, head, read_head);
}

void FaceFollowController::setAllowedLocked(bool allowed, double wall_sec, FaceFollowHeadOutput& head,
                                            const std::function<std::array<double, 3>()>& read_head) {
  if (!allowed)
  {
    if (!paused_)
    {
      head.setTargetValid(false);
      last_valid_ = false;
      invalid_count_ = kInvalidFrameThreshold;
      paused_ = true;
      reseed_ = true;
    }
    return;
  }
  if (!paused_) return;

  const auto current = readHead(read_head);
  yaw_ = current[0];
  pitch_ = current[1];
  head.seedCurrentPose(wall_sec, current[0], current[1], current[2]);
  head.setTargetValid(false);
  last_valid_ = false;
  invalid_count_ = kInvalidFrameThreshold;
  paused_ = false;
  reseed_ = true;
}

void FaceFollowController::processTarget(const Target& target, double wall_sec, FaceFollowHeadOutput& head,
                                         const std::function<std::array<double, 3>()>& read_head, bool allowed) {
  std::lock_guard<std::mutex> lock(mutex_);
  setAllowedLocked(allowed, wall_sec, head, read_head);
  if (paused_) return;

  if (!target.visible)
  {
    if (invalid_count_ < kInvalidFrameThreshold) ++invalid_count_;
    if (invalid_count_ < kInvalidFrameThreshold) return;
    head.setTargetValid(false);
    last_valid_ = false;
    return;
  }

  invalid_count_ = 0;
  head.setTargetValid(true);
  if (reseed_ || (!last_valid_ && !head.isWpReady()))
  {
    const auto current = readHead(read_head);
    head.seedCurrentPose(wall_sec, current[0], current[1], current[2]);
    yaw_ = current[0];
    pitch_ = current[1];
  }
  else if (!last_valid_)
  {
    double roll = 0.0;
    head.getFilteredPos(yaw_, pitch_, roll);
  }
  reseed_ = false;
  last_valid_ = true;

  const double dx = target.x - 0.5;
  const double dy = target.y - 0.5;
  if (std::abs(dx) > kDeadZone) yaw_ = std::max(-0.8, std::min(0.8, yaw_ + kScale * dx));
  if (std::abs(dy) > kDeadZone) pitch_ = std::max(-0.4, std::min(0.4, pitch_ + kScale * dy));
  head.setTarget(wall_sec, yaw_, pitch_);
}

}  // namespace legged
