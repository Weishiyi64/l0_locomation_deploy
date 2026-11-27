#pragma once

#include <array>
#include <functional>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <string>
#include <vector>

namespace legged {

class FaceFollowHeadOutput {
 public:
  /// Find head/neck joints in yaw/pitch/roll order.
  void initJoints(const std::vector<std::string>& joint_names, rclcpp::Logger logger);
  void seedCurrentPose(double t_now, double yaw, double pitch, double roll);
  void setTarget(double t_wall_sec, double yaw, double pitch);
  void stepHeadOutput(double t_wall_sec);
  bool getJointCmd(size_t global_j, double& q, double& dq, double& kp, double& kd, double& tau_ff) const;

  bool isTopicEnabled() const { return topicEnabled_; }
  void setTopicEnabled(bool enabled) { topicEnabled_ = enabled; }
  void setTopicBlendSec(double sec);

  bool isWpReady() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return wp_ready_;
  }

  void setTargetValid(bool valid) {
    std::lock_guard<std::mutex> lock(mutex_);
    topic_signal_valid_ = valid;
    if (valid) returning_to_neutral_ = false;
  }

  void getFilteredPos(double& yaw, double& pitch, double& roll) const {
    std::lock_guard<std::mutex> lock(mutex_);
    yaw = p_cmd_rpy_[0];
    pitch = p_cmd_rpy_[1];
    roll = p_cmd_rpy_[2];
  }

 private:
  struct TopicWaypoint {
    double t{0.0};
    double yaw{0.0};
    double pitch{0.0};
    double roll{0.0};
  };

  std::vector<size_t> jointIndices_;
  std::vector<double> cmd_;
  std::vector<double> cmd_dq_;
  mutable std::mutex mutex_;
  bool topicEnabled_{false};
  TopicWaypoint wp_start_;
  TopicWaypoint wp_end_;
  bool wp_ready_{false};
  double topic_blend_sec_{0.1};
  double p_cmd_rpy_[3]{0.0, 0.0, 0.0};
  bool topic_signal_valid_{false};
  bool returning_to_neutral_{false};
};

class FaceFollowController {
 public:
  struct Target {
    double x{0.5};
    double y{0.5};
    bool visible{false};
  };

  bool parsePayload(const std::string& payload, Target& target, std::string& error) const;
  void processTarget(const Target& target, double wall_sec, FaceFollowHeadOutput& head,
                     const std::function<std::array<double, 3>()>& read_head, bool allowed);
  void setAllowed(bool allowed, double wall_sec, FaceFollowHeadOutput& head,
                  const std::function<std::array<double, 3>()>& read_head);

 private:
  void setAllowedLocked(bool allowed, double wall_sec, FaceFollowHeadOutput& head,
                        const std::function<std::array<double, 3>()>& read_head);

  mutable std::mutex mutex_;
  double yaw_{0.0};
  double pitch_{0.0};
  bool last_valid_{false};
  bool paused_{true};
  bool reseed_{true};
  int invalid_count_{10};
};

}  // namespace legged
