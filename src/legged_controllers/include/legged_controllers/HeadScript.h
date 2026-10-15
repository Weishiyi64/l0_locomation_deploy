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
  /// /eye_target Point.z must be exactly this value to mean "no gaze target" (fixed protocol constant).
  static constexpr double kEyeTargetInvalidZ = -1.0;

  HeadScriptController() : logger_(rclcpp::get_logger("head_script")) {}

  /// Find Y1 "head_*" or L0 "neck_*" joints in yaw/pitch/roll order.
  void initJoints(const std::vector<std::string>& joint_names, rclcpp::Logger logger);

  /// Load YAML head_script timeline from path. Returns false on error.
  bool loadConfig(const std::string& path);

  /// Evaluate timeline at t_sec and update cmd buffers (thread-safe).
  void updateFromTime(double t_sec);

  /// Seed the interpolator with the actual joint pose before the first /eye_target arrives.
  /// Call once from the control layer (reading real motor state) so the first segment starts
  /// at the real head position rather than zero.
  void seedCurrentPose(double t_now, double yaw, double pitch);

  bool isWpReady() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return wp_ready_;
  }

  /// Register the latest /eye_target integrated target at wall time t_wall_sec.
  /// Blends from the current commanded pose to (yaw, pitch) over topic_blend_sec_ (YAML),
  /// sampled every control tick (e.g. 500 Hz); duration does not follow the topic rate.
  void setTarget(double t_wall_sec, double yaw, double pitch);

  /// Single entry for the control loop (500 Hz): script-only, topic-only, or hybrid.
  /// t_wall_sec is the current wall-clock time (seconds) used to evaluate the interpolation segment.
  /// t_loop_sec is the looped script time; pass 0 when no script is active.
  void stepHeadOutput(double dt, double t_wall_sec, double t_loop_sec);

  /// If global_j is a head/neck joint and head overlay is active (script and/or topic enabled), fill q/dq/gains.
  /// If both script and topic are disabled, returns false so walking uses RL/imitation PosCmd like other joints.
  /// \p topic_kp/kd/tau_ff: neck PD from HeadTopicController::Params when gaze topic is active (same joint order).
  bool getJointCmd(size_t global_j, double& q, double& dq, double& kp, double& kd, double& tau_ff,
                   const double topic_kp[3], const double topic_kd[3], const double topic_tau_ff[3]) const;

  bool isScriptEnabled() const { return scriptEnabled_; }
  bool isTopicEnabled() const { return topicEnabled_; }
  void setScriptEnabled(bool e) { scriptEnabled_ = e; }
  /// Enable /eye_target gaze path (optionally hybrid with head_script when both are on).
  void setTopicEnabled(bool e) { topicEnabled_ = e; }

  /// Update topic signal validity from the z component of /eye_target.
  /// Returns true if z is valid (z != kEyeTargetInvalidZ).
  bool updateTopicSignalFromZ(double z) {
    std::lock_guard<std::mutex> lock(mutex_);
    topic_signal_valid_ = (z != kEyeTargetInvalidZ);
    return topic_signal_valid_;
  }

  /// Last commanded yaw/pitch (rad), for re-seeding topic integrator after signal loss (no separate filter state).
  void getFilteredPos(double& yaw, double& pitch) const {
    std::lock_guard<std::mutex> lock(mutex_);
    yaw = p_cmd_yaw_pitch_[0];
    pitch = p_cmd_yaw_pitch_[1];
  }

  /// Wall-clock blend duration for /eye_target waypoints (seconds); from head_topic YAML
  /// `topic_blend_duration_sec` via LeggedController.
  void setTopicBlendSec(double sec);
  /// Wall-clock blend duration for topic->script fallback (seconds); from head_topic YAML.
  void setFallbackBlendSec(double sec);

  bool empty() const { return jointIndices_.empty(); }
  size_t numJoints() const { return jointIndices_.size(); }
  double duration() const { return duration_; }
  size_t numSegments() const { return segments_.size(); }

 private:
  // Interpolation segment between two consecutive /eye_target waypoints.
  // wp_start_ holds the commanded pose at message-arrival time;
  // wp_end_ holds the new target at (arrival + estimated period).
  // The 500 Hz loop linearly samples [wp_start_, wp_end_] by wall-clock time.
  struct TopicWaypoint {
    double t{0.0};
    double yaw{0.0};
    double pitch{0.0};
  };

  static constexpr double kDefaultTopicPeriod = 0.1;  // 10 Hz assumption

  std::vector<size_t> jointIndices_;
  std::vector<double> cmd_;
  std::vector<double> cmd_dq_;
  mutable std::mutex mutex_;
  bool scriptEnabled_ = true;
  bool topicEnabled_ = false;
  // Script-mode neck PD (yaw, pitch, roll)：仅由 head_script.yaml 写入；话题 PD 在 HeadTopicController::Params。
  double kp_script_[3]{};
  double kd_script_[3]{};
  double tau_ff_script_[3]{};
  double duration_ = 0.0;
  std::vector<HeadScriptSegment> segments_;
  rclcpp::Logger logger_;

  // Forward-interpolation: fixed wall-clock blend from current cmd → latest target.
  // stepHeadOutput samples this at control rate (e.g. 500 Hz); span is NOT tied to /eye_target rate.
  TopicWaypoint wp_start_;
  TopicWaypoint wp_end_;
  bool wp_ready_{false};
  /// Seconds to move from commanded pose to new target after each setTarget (YAML: topic_blend_duration_sec).
  double topic_blend_sec_{kDefaultTopicPeriod};

  // Last commanded yaw/pitch (rad); used by getFilteredPos for re-seeding on recovery.
  double p_cmd_yaw_pitch_[2] = {0.0, 0.0};

  bool topic_signal_valid_ = false;
  bool last_topic_signal_valid_ = true;

  // Smooth takeover when topic signal becomes invalid: blend from current command
  // to script pose over fallback_blend_sec_ instead of switching in one cycle.
  double fallback_blend_sec_{0.2};
  bool fallback_blend_active_{false};
  double fallback_blend_start_t_{0.0};
  double fallback_blend_start_yaw_{0.0};
  double fallback_blend_start_pitch_{0.0};
};

}  // namespace legged
