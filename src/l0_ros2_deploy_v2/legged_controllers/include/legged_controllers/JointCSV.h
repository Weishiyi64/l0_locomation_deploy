//
// Joint position timeline from Blender-exported CSV (any subset of robot joints).
//
#pragma once

#include <rclcpp/logger.hpp>
#include <string>
#include <vector>

namespace legged {

class JointCsvController {
 public:
  JointCsvController() : logger_(rclcpp::get_logger("joint_csv")) {}

  void initJoints(const std::vector<std::string>& joint_names, rclcpp::Logger logger);

  /// Parse CSV; map header columns to global joint indices. Returns false on error.
  bool load(const std::string& path);

  void setEnabled(bool e);
  bool isEnabled() const { return enabled_; }
  void stop();

  /// Seconds to interpolate from current pose to CSV frame 0; 0 disables blending (immediate playback).
  void setBlendDuration(double sec);
  void setHoldLastFrame(bool hold) { hold_last_frame_after_finish_ = hold; }

  /// Rising edge: Idle/Done→blend then auto-play; during blend, rising edge skips to playback. Ignored while Playing.
  void onTrigger(const std::vector<double>& q_all_motor, double wall_sec);

  /// Per control tick: updates q_cmd_ from phase (Idle until first trigger).
  void step(double dt, double wall_sec);

  /// Looping timeline (seconds); wraps time; updates buffered q for CSV-driven joints.
  void updateFromTime(double t_loop_sec);

  /// Fill a command for CSV-driven joints, including default or CSV-provided gains.
  bool getJointCmd(size_t global_j, double& q, double& dq, double& kp, double& kd, double& tau_ff) const;

  /// One-shot playback completion edge, consumed by the caller.
  bool consumeFinishedEvent();

  /// Export gains for CSV-driven joints; non-CSV joints keep the caller's existing values.
  void copyGainCommands(std::vector<double>& kps, std::vector<double>& kds) const;

  bool isCsvJoint(size_t global_j) const { return global_j < is_csv_joint_.size() && is_csv_joint_[global_j]; }
  const std::vector<bool>& csvJointMask() const { return is_csv_joint_; }

  double duration() const { return static_cast<double>(frames_.size()) * frame_time_; }
  bool empty() const { return frames_.empty() || frame_time_ <= 0.0; }
  bool coversAllJoints() const;

 private:
  enum class Phase { Idle, Blending, Playing, Holding, Done };

  void setQCmdFromTimelineTime(double t_loop_sec);
  void setQCmdFromPlaybackTimeNoLoop(double t_play);
  void applyFirstFrameToQCmd();
  void applyLastFrameToQCmd();

  rclcpp::Logger logger_;
  std::vector<std::string> robot_joint_names_;
  std::vector<bool> is_csv_joint_;
  std::vector<double> q_cmd_;
  std::vector<double> kp_cmd_;
  std::vector<double> kd_cmd_;
  bool has_csv_kp_{false};
  bool has_csv_kd_{false};

  double frame_time_{0.0};
  std::vector<std::vector<double>> frames_;
  std::vector<size_t> col_to_global_;
  bool enabled_{false};
  bool hold_last_frame_after_finish_{false};

  Phase phase_{Phase::Idle};
  double blend_elapsed_{0.0};
  double blend_duration_sec_{1.5};
  double play_anchor_wall_sec_{0.0};
  std::vector<double> q_blend_start_;
  bool finished_event_{false};
};

}  // namespace legged
