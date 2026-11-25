#pragma once

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/logger.hpp>
#include <string>
#include <vector>

namespace legged {

enum class ActionSequenceMode : std::uint8_t { kRlWithCsv = 0, kFullCsv = 1, kRlImitation = 2 };

/// One keyframe on the phase-local time axis (seconds from phase start). Used by `rl_with_csv` when
/// `velocity_timeline` is non-empty; otherwise `base_velocity` applies for the whole phase.
struct ActionSequenceVelocityKeyframe {
  double time_from_phase_start_sec{0.0};
  geometry_msgs::msg::Twist velocity;
};

struct ActionSequencePhase {
  std::string name;
  ActionSequenceMode mode{ActionSequenceMode::kRlWithCsv};
  /// Phase length in seconds (sim / ROS time). From YAML or CSV auto; may be increased at runtime by
  /// csv_entry_blend_sec.
  double duration{0.0};
  /// True if duration was set explicitly in YAML (warn when shorter than CSV playback).
  bool duration_explicit{true};
  /// Empty for `rl_imitation` phases; CSV path for `rl_with_csv` / `full_csv`.
  std::string csv_path;
  geometry_msgs::msg::Twist base_velocity;
  /// If non-empty, overrides `base_velocity` with piecewise-constant speeds keyed by phase time (see YAML
  /// `velocity_timeline`). Only valid for `rl_with_csv`.
  std::vector<ActionSequenceVelocityKeyframe> velocity_timeline;
  /// End-of-phase base_velocity ramp duration for `rl_with_csv` (seconds). `< 0` means use sequence default from YAML.
  double base_vel_ramp_sec{-1.0};
};

class ActionSequenceController {
 public:
  ActionSequenceController() : logger_(rclcpp::get_logger("action_sequence")) {}

  void init(rclcpp::Logger logger, const std::string& package_share_dir);
  bool load(const std::string& path);

  bool isLoaded() const { return loaded_; }
  bool isActive() const { return active_; }
  bool empty() const { return phases_.empty(); }

  /// Start sequence; phase timeline is anchored on the first controller step() time (same clock as update(time)).
  bool trigger();
  void step(double wall_sec);
  void stop();

  const ActionSequencePhase* activePhase() const;
  bool consumePhaseStartEvent(ActionSequencePhase& out);
  bool consumeFinishedEvent();

  /// Wall time remaining in the current phase (s). 0 if inactive or past phase end.
  double remainingPhaseSeconds(double wall_sec) const;

  /// Elapsed time since the current phase started (s), clamped to [0, phase.duration]. 0 if inactive or before first
  /// step() after trigger().
  double elapsedPhaseSeconds(double wall_sec) const;

  /// Extend the active phase wall duration by blend_sec after JointCsvController::onTrigger (same wall clock as
  /// step()).
  void addBlendToCurrentPhaseTimeline(double blend_sec);

  /// Effective `base_vel_ramp_sec` for the active phase. Phase-end base velocity ramp is currently disabled.
  double activePhaseBaseVelRampSec() const;

  /// Mark the active `rl_imitation` phase complete (only), then advance to the next phase or finish.
  void completeActivePhaseByExternalEvent(double wall_sec);

  /// Default-control→CSV first-frame blend duration (seconds), from `sequence.csv_entry_blend_sec`.
  double csvEntryBlendSec() const { return csv_entry_blend_sec_; }

  /// CSV last-frame→default-control blend duration (seconds), from `sequence.csv_exit_blend_sec`.
  double csvExitBlendSec() const { return csv_exit_blend_sec_; }

  /// True if any loaded phase uses `mode: rl_imitation`.
  bool usesRlImitation() const { return uses_rl_imitation_; }

  /// True if the sequence depends on the walking RL/AMP or imitation policy to support non-CSV joints.
  bool requiresWalkingPolicy() const { return requires_walking_policy_; }

  /// True if any loaded phase uses `mode: full_csv`.
  bool usesFullCsv() const { return uses_full_csv_; }

  /// Sequence-level deploy subpolicy used by all `rl_imitation` phases in this sequence.
  const std::string& imitationPolicy() const { return imitation_policy_; }

 private:
  bool parsePhase(const YAML::Node& node, std::size_t index, ActionSequencePhase& out);
  std::string resolvePath(const std::string& raw_path, const std::string& base_dir) const;
  bool pathExists(const std::string& path) const;
  static std::string parentDirectory(const std::string& path);

  rclcpp::Logger logger_;
  std::string package_share_dir_;
  std::string sequence_path_;
  std::string sequence_dir_;
  /// Default from YAML `sequence.base_vel_ramp_sec` (seconds); 0 = off.
  double sequence_base_vel_ramp_sec_{0.0};
  /// From YAML `sequence.csv_entry_blend_sec` (seconds); 0 = immediate CSV takeover.
  double csv_entry_blend_sec_{1.5};
  /// From YAML `sequence.csv_exit_blend_sec` (seconds); 0 = immediate return to default control.
  double csv_exit_blend_sec_{0.0};
  bool uses_rl_imitation_{false};
  bool requires_walking_policy_{false};
  bool uses_full_csv_{false};
  std::string imitation_policy_;
  std::vector<ActionSequencePhase> phases_;
  /// Snapshot after YAML load; `trigger()` resets `phases_` from this so runtime duration tweaks (blend) do not stack.
  std::vector<ActionSequencePhase> phases_template_;

  bool loaded_{false};
  bool active_{false};
  std::size_t current_phase_index_{0};
  double phase_start_wall_sec_{0.0};
  /// Until the first step(wall_sec) after trigger(), timeline is not aligned to the controller clock.
  bool pending_controller_timeline_sync_{false};
  bool phase_start_event_{false};
  bool finished_event_{false};
};

}  // namespace legged
