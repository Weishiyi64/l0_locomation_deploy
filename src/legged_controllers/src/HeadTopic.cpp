//
// HeadTopicController — YAML parameters and /eye_target gaze integration only.
// ROS subscriptions, hardware reads, and motor publishing live in LeggedController.
//
#include "legged_controllers/HeadTopic.h"

#include <algorithm>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

namespace legged {

namespace {

std::pair<double, double> readNeckYawPitchRad(
    const std::function<std::pair<double, double>()>& read_neck_yaw_pitch_rad) {
  return read_neck_yaw_pitch_rad ? read_neck_yaw_pitch_rad() : std::make_pair(0.0, 0.0);
}

}  // namespace

bool HeadTopicController::loadConfig(const std::string& path, rclcpp::Logger logger) {
  logger_ = logger;

  if (path.empty())
  {
    RCLCPP_WARN(logger_, "[HeadTopicController] No config path provided; using built-in defaults.");
    return false;
  }

  std::ifstream f(path);
  if (!f.good())
  {
    RCLCPP_WARN(logger_, "[HeadTopicController] Config file not found: %s — using defaults.", path.c_str());
    return false;
  }

  YAML::Node root;
  try
  { root = YAML::LoadFile(path); } catch (const std::exception& e)
  {
    RCLCPP_WARN(logger_, "[HeadTopicController] YAML parse error in %s: %s — using defaults.", path.c_str(), e.what());
    return false;
  }

  YAML::Node cfg = root["head_topic_controller"];
  if (!cfg)
  {
    RCLCPP_WARN(logger_, "[HeadTopicController] Missing 'head_topic_controller' key in %s — using defaults.",
                path.c_str());
    return false;
  }

  params_.deadZone = cfg["dead_zone"].as<double>(params_.deadZone);
  params_.scale = cfg["scale"].as<double>(params_.scale);
  params_.yawClampMin = cfg["yaw_clamp_min"].as<double>(params_.yawClampMin);
  params_.yawClampMax = cfg["yaw_clamp_max"].as<double>(params_.yawClampMax);
  params_.pitchClampMin = cfg["pitch_clamp_min"].as<double>(params_.pitchClampMin);
  params_.pitchClampMax = cfg["pitch_clamp_max"].as<double>(params_.pitchClampMax);
  params_.invalidFrameThreshold = cfg["invalid_frame_threshold"].as<int>(params_.invalidFrameThreshold);

  auto load_arr3 = [&cfg](const char* key, double dst[3]) {
    if (!cfg[key]) return;
    if (cfg[key].IsScalar())
    {
      const double v = cfg[key].as<double>();
      dst[0] = dst[1] = dst[2] = v;
    }
    else if (cfg[key].IsSequence() && cfg[key].size() >= 3)
    {
      for (int i = 0; i < 3; ++i) dst[i] = cfg[key][i].as<double>(dst[i]);
    }
  };
  load_arr3("kp", params_.kp);
  load_arr3("kd", params_.kd);
  load_arr3("tau_ff", params_.tauFf);

  double blend_raw = 0.1;
  if (cfg["topic_blend_duration_sec"])
  {
    blend_raw = cfg["topic_blend_duration_sec"].as<double>();
  }
  params_.topicBlendSec = std::clamp(blend_raw, 0.002, 0.5);
  double fallback_blend_raw = 0.2;
  if (cfg["fallback_blend_duration_sec"])
  {
    fallback_blend_raw = cfg["fallback_blend_duration_sec"].as<double>();
  }
  params_.fallbackBlendSec = std::clamp(fallback_blend_raw, 0.02, 1.0);

  RCLCPP_INFO(logger_,
              "[HeadTopicController] Config loaded from %s: "
              "deadZone=%.3f scale=%.3f yawClamp=[%.2f,%.2f] pitchClamp=[%.2f,%.2f] "
              "topic PD kp=[%.1f %.1f %.1f] kd=[%.2f %.2f %.2f] blend_sec=%.3f fallback_blend_sec=%.3f",
              path.c_str(), params_.deadZone, params_.scale, params_.yawClampMin, params_.yawClampMax,
              params_.pitchClampMin, params_.pitchClampMax, params_.kp[0], params_.kp[1], params_.kp[2], params_.kd[0],
              params_.kd[1], params_.kd[2], params_.topicBlendSec, params_.fallbackBlendSec);
  return true;
}

void HeadTopicController::setIntegrationAllowed(
    bool allowed, double t_wall_sec, HeadScriptController& headCtrl,
    const std::function<std::pair<double, double>()>& read_neck_yaw_pitch_rad) {
  std::lock_guard<std::mutex> lock(mutex_);
  setIntegrationAllowedLocked(allowed, t_wall_sec, headCtrl, read_neck_yaw_pitch_rad);
}

void HeadTopicController::setIntegrationAllowedLocked(
    bool allowed, double t_wall_sec, HeadScriptController& headCtrl,
    const std::function<std::pair<double, double>()>& read_neck_yaw_pitch_rad) {
  if (!allowed)
  {
    if (!integrationPaused_)
    {
      headCtrl.updateTopicSignalFromZ(HeadScriptController::kEyeTargetInvalidZ);
      lastEyeTargetZValid_ = false;
      eyeTargetInvalidCount_ = params_.invalidFrameThreshold;
      integrationPaused_ = true;
      reseedOnNextValid_ = true;
    }
    return;
  }

  if (!integrationPaused_)
  {
    return;
  }

  const std::pair<double, double> neck = readNeckYawPitchRad(read_neck_yaw_pitch_rad);
  headTopicYaw_ = neck.first;
  headTopicPitch_ = neck.second;
  headCtrl.seedCurrentPose(t_wall_sec, headTopicYaw_, headTopicPitch_);
  headCtrl.updateTopicSignalFromZ(HeadScriptController::kEyeTargetInvalidZ);
  lastEyeTargetZValid_ = false;
  eyeTargetInvalidCount_ = params_.invalidFrameThreshold;
  integrationPaused_ = false;
  reseedOnNextValid_ = true;
}

void HeadTopicController::processEyeTarget(const geometry_msgs::msg::Point& msg, double t_wall_sec,
                                           HeadScriptController& headCtrl,
                                           const std::function<std::pair<double, double>()>& read_neck_yaw_pitch_rad,
                                           bool allow_integration) {
  std::lock_guard<std::mutex> lock(mutex_);
  setIntegrationAllowedLocked(allow_integration, t_wall_sec, headCtrl, read_neck_yaw_pitch_rad);
  if (integrationPaused_)
  {
    return;
  }

  const bool z_raw_valid = (msg.z != HeadScriptController::kEyeTargetInvalidZ);

  if (!z_raw_valid)
  {
    if (eyeTargetInvalidCount_ < params_.invalidFrameThreshold)
    {
      ++eyeTargetInvalidCount_;
    }
    if (eyeTargetInvalidCount_ < params_.invalidFrameThreshold)
    {
      return;
    }
    headCtrl.updateTopicSignalFromZ(msg.z);
    lastEyeTargetZValid_ = false;
    return;
  }

  eyeTargetInvalidCount_ = 0;
  headCtrl.updateTopicSignalFromZ(msg.z);
  const bool was_invalid = !lastEyeTargetZValid_;
  const bool force_reseed = reseedOnNextValid_;
  reseedOnNextValid_ = false;

  if (force_reseed)
  {
    const std::pair<double, double> neck = readNeckYawPitchRad(read_neck_yaw_pitch_rad);
    headCtrl.seedCurrentPose(t_wall_sec, neck.first, neck.second);
    headTopicYaw_ = neck.first;
    headTopicPitch_ = neck.second;
  }
  else if (was_invalid)
  {
    if (!headCtrl.isWpReady())
    {
      const std::pair<double, double> neck = readNeckYawPitchRad(read_neck_yaw_pitch_rad);
      headCtrl.seedCurrentPose(t_wall_sec, neck.first, neck.second);
      headTopicYaw_ = neck.first;
      headTopicPitch_ = neck.second;
    }
    else
    {
      headCtrl.getFilteredPos(headTopicYaw_, headTopicPitch_);
    }
  }
  lastEyeTargetZValid_ = true;

  const double dx = msg.x - 0.5;
  const double dy = msg.y - 0.5;
  const double dyaw = (std::abs(dx) > params_.deadZone) ? params_.scale * dx : 0.0;
  const double dpitch = (std::abs(dy) > params_.deadZone) ? params_.scale * dy : 0.0;

  headTopicYaw_ = std::clamp(headTopicYaw_ + dyaw, params_.yawClampMin, params_.yawClampMax);
  headTopicPitch_ = std::clamp(headTopicPitch_ + dpitch, params_.pitchClampMin, params_.pitchClampMax);

  headCtrl.setTarget(t_wall_sec, headTopicYaw_, headTopicPitch_);
}

}  // namespace legged
