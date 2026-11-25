#include "legged_controllers/FaceFollow.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace legged {
namespace {

constexpr double kImageWidth = 640.0;
constexpr double kImageHeight = 480.0;
constexpr double kDeadZone = 0.05;
constexpr double kScale = 0.232;
constexpr int kInvalidFrameThreshold = 10;

std::array<double, 3> readHead(const std::function<std::array<double, 3>()>& reader) {
  return reader ? reader() : std::array<double, 3>{{0.0, 0.0, 0.0}};
}

}  // namespace

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

void FaceFollowController::setAllowed(bool allowed, double wall_sec, HeadScriptController& head,
                                      const std::function<std::array<double, 3>()>& read_head) {
  std::lock_guard<std::mutex> lock(mutex_);
  setAllowedLocked(allowed, wall_sec, head, read_head);
}

void FaceFollowController::setAllowedLocked(bool allowed, double wall_sec, HeadScriptController& head,
                                            const std::function<std::array<double, 3>()>& read_head) {
  if (!allowed)
  {
    if (!paused_)
    {
      head.updateTopicSignalFromZ(HeadScriptController::kEyeTargetInvalidZ);
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
  head.updateTopicSignalFromZ(HeadScriptController::kEyeTargetInvalidZ);
  last_valid_ = false;
  invalid_count_ = kInvalidFrameThreshold;
  paused_ = false;
  reseed_ = true;
}

void FaceFollowController::processTarget(const Target& target, double wall_sec, HeadScriptController& head,
                                         const std::function<std::array<double, 3>()>& read_head, bool allowed) {
  std::lock_guard<std::mutex> lock(mutex_);
  setAllowedLocked(allowed, wall_sec, head, read_head);
  if (paused_) return;

  if (!target.visible)
  {
    if (invalid_count_ < kInvalidFrameThreshold) ++invalid_count_;
    if (invalid_count_ < kInvalidFrameThreshold) return;
    head.updateTopicSignalFromZ(HeadScriptController::kEyeTargetInvalidZ);
    last_valid_ = false;
    return;
  }

  invalid_count_ = 0;
  head.updateTopicSignalFromZ(0.0);
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
