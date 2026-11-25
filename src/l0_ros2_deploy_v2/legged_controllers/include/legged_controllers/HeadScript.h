#pragma once

#include <mutex>
#include <rclcpp/logger.hpp>
#include <string>
#include <vector>

namespace legged {
struct HeadScriptSegment {
  double t_start{0.0};
  double t_end{0.0};
  enum class AxisMode { Constant, Lerp, Sin };
  AxisMode yaw_mode{AxisMode::Constant};
  AxisMode pitch_mode{AxisMode::Constant};
  AxisMode roll_mode{AxisMode::Constant};
  double yaw{0.0}, pitch{0.0}, roll{0.0};
  double yaw_from{0.0}, yaw_to{0.0}, pitch_from{0.0}, pitch_to{0.0}, roll_from{0.0}, roll_to{0.0};
  double yaw_sin_amp_deg{0.0}, yaw_sin_hz{0.0};
  double pitch_sin_amp_deg{0.0}, pitch_sin_hz{0.0};
  double roll_sin_amp_deg{0.0}, roll_sin_hz{0.0};
};

class HeadScriptController {
 public:
  /// Internal marker for loss of the face-follow target.
  static constexpr double kEyeTargetInvalidZ = -1.0;

  HeadScriptController() : logger_(rclcpp::get_logger("head_script")) {}

  /// Find Y1 "head_*" or L0 "neck_*" joints in yaw/pitch/roll order.
  void initJoints(const std::vector<std::string>& joint_names, rclcpp::Logger logger);

  /// Load YAML head_script timeline from path. Returns false on error.
  bool loadConfig(const std::string& path);

  /// Evaluate timeline at t_sec and update cmd buffers (thread-safe).
  void updateFromTime(double t_sec);

  /// Seed the interpolator with the actual joint pose before the first head target arrives.
  /// Call once from the control layer (reading real motor state) so the first segment starts
  /// at the real head position rather than zero.
  void seedCurrentPose(double t_now, double yaw, double pitch, double roll);

  bool isWpReady() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return wp_ready_;
  }

  /// Register the latest face-follow target at wall time t_wall_sec.
  /// Blends from the current commanded pose to (yaw, pitch) over topic_blend_sec_ (YAML),
  /// sampled every control tick (e.g. 500 Hz); duration does not follow the topic rate.
  void setTarget(double t_wall_sec, double yaw, double pitch);

  /// Register a three-axis gamepad target. Robots without a roll joint simply have no
  /// third joint command to consume.
  void setTarget(double t_wall_sec, double yaw, double pitch, double roll);

  /// Single entry for the control loop (500 Hz): script-only, topic-only, or hybrid.
  /// t_wall_sec is the current wall-clock time (seconds) used to evaluate the interpolation segment.
  /// t_loop_sec is the looped script time; pass 0 when no script is active.
  void stepHeadOutput(double dt, double t_wall_sec, double t_loop_sec);

  /// If global_j is a head/neck joint and head overlay is active (script and/or topic enabled), fill q/dq/gains.
  /// If both script and topic are disabled, returns false so walking uses RL/imitation PosCmd like other joints.
  bool getJointCmd(size_t global_j, double& q, double& dq, double& kp, double& kd, double& tau_ff) const;

  bool isScriptEnabled() const { return scriptEnabled_; }
  bool isTopicEnabled() const { return topicEnabled_; }
  void setScriptEnabled(bool e) { scriptEnabled_ = e; }
  /// Enable face-follow (optionally hybrid with head_script when both are on).
  void setTopicEnabled(bool e) { topicEnabled_ = e; }

  /// Update face-follow signal validity.
  /// Returns true if z is valid (z != kEyeTargetInvalidZ).
  bool updateTopicSignalFromZ(double z) {
    std::lock_guard<std::mutex> lock(mutex_);
    topic_signal_valid_ = (z != kEyeTargetInvalidZ);
    return topic_signal_valid_;
  }

  /// Last commanded yaw/pitch/roll (rad), for re-seeding the topic integrator after signal loss.
  void getFilteredPos(double& yaw, double& pitch, double& roll) const {
    std::lock_guard<std::mutex> lock(mutex_);
    yaw = p_cmd_rpy_[0];
    pitch = p_cmd_rpy_[1];
    roll = p_cmd_rpy_[2];
  }

  /// Wall-clock blend duration for face-follow waypoints.
  void setTopicBlendSec(double sec);
  /// Wall-clock blend duration for face-follow-to-script fallback.
  void setFallbackBlendSec(double sec);

  bool empty() const { return jointIndices_.empty(); }
  size_t numJoints() const { return jointIndices_.size(); }
  double duration() const { return duration_; }
  size_t numSegments() const { return segments_.size(); }

 private:
  // Interpolation segment between two consecutive face-follow waypoints.
  // wp_start_ holds the commanded pose at message-arrival time;
  // wp_end_ holds the new target at (arrival + estimated period).
  // The 500 Hz loop linearly samples [wp_start_, wp_end_] by wall-clock time.
  struct TopicWaypoint {
    double t{0.0};
    double yaw{0.0};
    double pitch{0.0};
    double roll{0.0};
  };

  static constexpr double kDefaultTopicPeriod = 0.1;  // 10 Hz assumption

  std::vector<size_t> jointIndices_;
  std::vector<double> cmd_;
  std::vector<double> cmd_dq_;
  mutable std::mutex mutex_;
  bool scriptEnabled_ = true;
  bool topicEnabled_ = false;
  // Legacy source gains are retained for file compatibility; the composer applies policy gains.
  double kp_script_[3]{};
  double kd_script_[3]{};
  double tau_ff_script_[3]{};
  double duration_ = 0.0;
  std::vector<HeadScriptSegment> segments_;
  rclcpp::Logger logger_;

  // Forward-interpolation: fixed wall-clock blend from current cmd → latest target.
  // stepHeadOutput samples this at control rate (e.g. 500 Hz); span is independent of detector rate.
  TopicWaypoint wp_start_;
  TopicWaypoint wp_end_;
  bool wp_ready_{false};
  /// Seconds to move from commanded pose to new target after each setTarget (YAML: topic_blend_duration_sec).
  double topic_blend_sec_{kDefaultTopicPeriod};

  // Last commanded yaw/pitch/roll (rad); used by getFilteredPos for re-seeding on recovery.
  double p_cmd_rpy_[3] = {0.0, 0.0, 0.0};
  bool topic_roll_enabled_{false};

  bool topic_signal_valid_ = false;
  bool last_topic_signal_valid_ = true;

  // Smooth takeover when topic signal becomes invalid: blend from current command
  // to script pose over fallback_blend_sec_ instead of switching in one cycle.
  double fallback_blend_sec_{0.2};
  bool fallback_blend_active_{false};
  double fallback_blend_start_t_{0.0};
  double fallback_blend_start_yaw_{0.0};
  double fallback_blend_start_pitch_{0.0};
  double fallback_blend_start_roll_{0.0};
};

}  // namespace legged
