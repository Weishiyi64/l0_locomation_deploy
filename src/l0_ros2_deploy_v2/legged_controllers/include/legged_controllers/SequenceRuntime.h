#pragma once

#include <legged_controllers/ActionSequence.h>
#include <legged_controllers/ControllerTypes.h>
#include <legged_controllers/IHardwareLayer.h>
#include <legged_controllers/JointCSV.h>
#include <legged_controllers/JointCommandTransition.h>

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace legged {

struct ActiveSequencePhaseSnapshot {
  bool active{false};
  ActionSequencePhase phase;
  double elapsed_sec{0.0};
  double remaining_sec{0.0};
  double base_vel_ramp_sec{0.0};
};

class SequenceRuntime {
 public:
  SequenceRuntime() = default;

  void init(rclcpp::Logger logger, const std::string& package_share_dir);
  void initJoints(const std::vector<std::string>& joint_names);
  void setEnabled(bool enabled);
  bool enabled() const { return sequence_engine_enabled_; }

  bool loadActionSequenceForPlayback(const std::string& path, const char* context);

  bool isLoaded() const;
  bool isActive() const;
  bool isFullCsvPhase() const;
  bool requiresWalkingPolicy() const;
  bool usesRlImitation() const;
  bool usesFullCsv() const;
  std::string imitationPolicy() const;
  double csvEntryBlendSec() const { return csv_entry_blend_sec_; }
  bool loadedPathMatches(const std::string& path) const;
  ActiveSequencePhaseSnapshot activePhaseSnapshot(double wall_sec) const;

  bool triggerLoaded();
  void step(double wall_sec);
  bool consumePhaseStartEvent(ActionSequencePhase& out);
  bool consumeFinishedEvent();
  void completeImitationPhaseByExternalEvent(double wall_sec, bool& next_phase_is_imitation);

  void setFullCsvPhase(bool active);
  void resetPhaseFlags();
  void resetForAbort();
  void resetForNormalFinish(double wall_sec, const std::vector<bool>& is_leg_joint);

  void clearReleaseHandoff();
  void beginReleaseHandoff(double wall_sec, const std::vector<bool>& is_leg_joint);
  JointCommandTransition& releaseHandoff() { return release_handoff_; }
  const JointCommandTransition& releaseHandoff() const { return release_handoff_; }
  bool releaseHandoffActive(double wall_sec) const;

  JointCsvController& jointCsv() { return joint_csv_; }
  const JointCsvController& jointCsv() const { return joint_csv_; }
  void triggerCsvBlend(const std::vector<double>& q_all, double wall_sec);

 private:
  rclcpp::Logger logger() const { return logger_; }

  rclcpp::Logger logger_{rclcpp::get_logger("sequence_runtime")};
  std::string package_share_dir_;
  bool sequence_engine_enabled_{false};

  JointCsvController joint_csv_;
  ActionSequenceController action_sequence_;
  mutable std::recursive_mutex mutex_;
  JointCommandTransition release_handoff_;

  std::string loaded_action_sequence_path_;
  double csv_entry_blend_sec_{0.0};
  double csv_exit_blend_sec_{0.0};
  bool full_csv_phase_{false};
};

}  // namespace legged
