#pragma once

#include <legged_controllers/ActionSequence.h>
#include <legged_controllers/ControllerTypes.h>
#include <legged_controllers/IHardwareLayer.h>
#include <legged_controllers/JointCSV.h>
#include <legged_controllers/JointCommandTransition.h>

#include <mutex>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
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
  bool loadEmotionSequenceMap(const std::string& path);
  bool loadKeyboardActionMap(const std::string& path);

  bool isLoaded() const;
  bool isActive() const;
  bool isFullCsvPhase() const;
  bool requiresWalkingPolicy() const;
  bool usesRlImitation() const;
  bool usesFullCsv() const;
  std::string imitationPolicy() const;
  bool softReleaseLatched() const { return soft_release_latched_; }
  const std::vector<double>& headHoldQ() const { return head_hold_q_; }
  double csvEntryBlendSec() const { return csv_entry_blend_sec_; }
  bool softReleaseOnFinish() const;
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
  void resetForSoftReleaseFinish(const std::vector<JointMotorState>& motor_states);
  void clearSoftReleaseLatch();

  void clearReleaseHandoff();
  void beginReleaseHandoff(double wall_sec, const std::vector<bool>& is_leg_joint);
  JointCommandTransition& releaseHandoff() { return release_handoff_; }
  const JointCommandTransition& releaseHandoff() const { return release_handoff_; }
  bool releaseHandoffActive(double wall_sec) const;

  JointCsvController& jointCsv() { return joint_csv_; }
  const JointCsvController& jointCsv() const { return joint_csv_; }
  void triggerCsvBlend(const std::vector<double>& q_all, double wall_sec);

  const KeyboardActionBinding* findKeyboardAction(const std::string& action_name) const;
  bool hasEmotionCode(const std::string& code) const;
  bool selectEmotionSequence(const std::string& code, std::string& selected_sequence_path);
  void markEmotionBusyIgnored(const std::string& code);
  void resetEmotionBusyIgnored();
  const std::string& keyboardAudioTopic() const { return keyboard_action_audio_topic_; }
  size_t emotionSequenceCount() const { return emotion_sequence_paths_.size(); }

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
  bool soft_release_latched_{false};
  std::vector<double> head_hold_q_;

  std::string keyboard_action_config_path_;
  std::string keyboard_action_audio_topic_;
  std::unordered_map<std::string, KeyboardActionBinding> keyboard_action_bindings_;
  std::unordered_map<std::string, std::vector<EmotionSequenceCandidate>> emotion_sequence_paths_;
  std::unordered_map<std::string, std::size_t> last_emotion_sequence_indices_;
  std::mt19937 emotion_sequence_rng_{std::random_device{}()};
  bool emotion_busy_ignore_logged_{false};
};

}  // namespace legged
