#include "legged_controllers/ActionSequence.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <utility>

#include "legged_controllers/JointCsvFormat.h"

namespace legged {

namespace {

bool isAbsolutePath(const std::string& path) { return !path.empty() && path.front() == '/'; }

std::string joinPath(const std::string& base, const std::string& leaf) {
  if (base.empty()) return leaf;
  if (leaf.empty()) return base;
  if (base.back() == '/') return base + leaf;
  return base + "/" + leaf;
}

std::string trimCopy(const std::string& s) {
  const auto first = std::find_if_not(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c); });
  const auto last = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char c) { return std::isspace(c); }).base();
  if (first >= last) return "";
  return std::string(first, last);
}

constexpr double kTimelineTimeEps = 1e-6;

bool parseVelocityTimeline(const YAML::Node& timeline_node, const std::string& phase_name, double phase_duration,
                           bool has_base_velocity_block, rclcpp::Logger logger,
                           std::vector<ActionSequenceVelocityKeyframe>& out) {
  out.clear();
  if (!timeline_node || timeline_node.IsNull()) return true;
  if (!timeline_node.IsSequence())
  {
    RCLCPP_ERROR(logger, "ActionSequence: phase '%s' velocity_timeline must be a sequence", phase_name.c_str());
    return false;
  }
  if (timeline_node.size() == 0) return true;

  if (has_base_velocity_block)
  {
    RCLCPP_WARN(logger,
                "ActionSequence: phase '%s' has both velocity and velocity_timeline; using velocity_timeline for "
                "rl_with_csv cmd_vel base",
                phase_name.c_str());
  }

  out.reserve(timeline_node.size());
  for (std::size_t i = 0; i < timeline_node.size(); ++i)
  {
    const YAML::Node kf = timeline_node[i];
    if (!kf || !kf.IsMap())
    {
      RCLCPP_ERROR(logger, "ActionSequence: phase '%s' velocity_timeline[%zu] must be a map", phase_name.c_str(), i);
      return false;
    }
    if (!kf["t"])
    {
      RCLCPP_ERROR(logger, "ActionSequence: phase '%s' velocity_timeline[%zu] missing t", phase_name.c_str(), i);
      return false;
    }
    ActionSequenceVelocityKeyframe key;
    key.time_from_phase_start_sec = kf["t"].as<double>();
    key.velocity.linear.x = kf["vx"] ? kf["vx"].as<double>(0.0) : 0.0;
    key.velocity.linear.y = kf["vy"] ? kf["vy"].as<double>(0.0) : 0.0;
    key.velocity.angular.z = kf["wz"] ? kf["wz"].as<double>(0.0) : 0.0;
    out.push_back(std::move(key));
  }

  if (std::abs(out.front().time_from_phase_start_sec) > kTimelineTimeEps)
  {
    RCLCPP_ERROR(logger, "ActionSequence: phase '%s' velocity_timeline must start with t: 0.0", phase_name.c_str());
    return false;
  }

  for (std::size_t i = 1; i < out.size(); ++i)
  {
    if (out[i].time_from_phase_start_sec + kTimelineTimeEps < out[i - 1].time_from_phase_start_sec)
    {
      RCLCPP_ERROR(logger, "ActionSequence: phase '%s' velocity_timeline t must be non-decreasing", phase_name.c_str());
      return false;
    }
  }

  for (const auto& k : out)
  {
    if (k.time_from_phase_start_sec > phase_duration + kTimelineTimeEps)
    {
      RCLCPP_ERROR(logger, "ActionSequence: phase '%s' velocity_timeline keyframe t=%.6f exceeds phase duration %.6fs",
                   phase_name.c_str(), k.time_from_phase_start_sec, phase_duration);
      return false;
    }
  }

  return true;
}

}  // namespace

void ActionSequenceController::init(rclcpp::Logger logger, const std::string& package_share_dir) {
  logger_ = logger;
  package_share_dir_ = package_share_dir;
}

bool ActionSequenceController::load(const std::string& path) {
  phases_.clear();
  phases_template_.clear();
  sequence_path_.clear();
  sequence_dir_.clear();
  csv_entry_blend_sec_ = 1.5;
  csv_exit_blend_sec_ = 0.0;
  uses_rl_imitation_ = false;
  requires_walking_policy_ = false;
  uses_full_csv_ = false;
  imitation_policy_.clear();
  loaded_ = false;
  active_ = false;
  current_phase_index_ = 0;
  phase_start_wall_sec_ = 0.0;
  pending_controller_timeline_sync_ = false;
  phase_start_event_ = false;
  finished_event_ = false;

  const std::string resolved_path = resolvePath(path, package_share_dir_);
  YAML::Node root;
  try
  { root = YAML::LoadFile(resolved_path); } catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger_, "ActionSequence: failed to load '%s': %s", resolved_path.c_str(), e.what());
    return false;
  }

  const YAML::Node sequence = root["sequence"];
  if (!sequence || !sequence["phases"] || !sequence["phases"].IsSequence() || sequence["phases"].size() == 0)
  {
    RCLCPP_ERROR(logger_, "ActionSequence: '%s' is missing a non-empty sequence.phases list", resolved_path.c_str());
    return false;
  }

  sequence_base_vel_ramp_sec_ = 0.0;
  if (sequence["base_vel_ramp_sec"])
  {
    sequence_base_vel_ramp_sec_ = std::max(sequence["base_vel_ramp_sec"].as<double>(0.0), 0.0);
  }

  if (sequence["csv_entry_blend_sec"])
  {
    csv_entry_blend_sec_ = std::max(sequence["csv_entry_blend_sec"].as<double>(0.0), 0.0);
  }
  else if (sequence["joint_csv_blend_sec"])
  {
    csv_entry_blend_sec_ = std::max(sequence["joint_csv_blend_sec"].as<double>(0.0), 0.0);
  }

  if (sequence["csv_exit_blend_sec"])
  {
    csv_exit_blend_sec_ = std::max(sequence["csv_exit_blend_sec"].as<double>(0.0), 0.0);
  }
  else if (sequence["release_blend_sec"])
  {
    csv_exit_blend_sec_ = std::max(sequence["release_blend_sec"].as<double>(0.0), 0.0);
  }

  const YAML::Node phases_node = sequence["phases"];
  sequence_path_ = resolved_path;
  sequence_dir_ = parentDirectory(resolved_path);

  for (std::size_t i = 0; i < phases_node.size(); ++i)
  {
    ActionSequencePhase phase;
    if (!parsePhase(phases_node[i], i, phase))
    {
      phases_.clear();
      phases_template_.clear();
      return false;
    }
    if (phase.mode == ActionSequenceMode::kRlImitation)
    {
      uses_rl_imitation_ = true;
    }
    if (phase.mode == ActionSequenceMode::kRlImitation || phase.mode == ActionSequenceMode::kRlWithCsv)
    {
      requires_walking_policy_ = true;
    }
    if (phase.mode == ActionSequenceMode::kFullCsv)
    {
      uses_full_csv_ = true;
    }
    phases_.push_back(std::move(phase));
  }

  if (sequence["imitation_policy"])
  {
    imitation_policy_ = trimCopy(sequence["imitation_policy"].as<std::string>(""));
  }
  if (imitation_policy_.empty() && sequence["imitation_policy_name"])
  {
    imitation_policy_ = trimCopy(sequence["imitation_policy_name"].as<std::string>(""));
  }
  if (uses_rl_imitation_ && imitation_policy_.empty())
  {
    RCLCPP_ERROR(logger_, "ActionSequence: '%s' has rl_imitation phase(s) but sequence.imitation_policy is missing",
                 resolved_path.c_str());
    phases_.clear();
    phases_template_.clear();
    uses_rl_imitation_ = false;
    return false;
  }
  loaded_ = !phases_.empty();
  if (loaded_)
  {
    phases_template_ = phases_;
  }
  return loaded_;
}

bool ActionSequenceController::trigger() {
  if (!loaded_ || phases_.empty())
  {
    RCLCPP_WARN(logger_, "ActionSequence: trigger ignored because no valid sequence is loaded");
    return false;
  }
  if (active_)
  {
    RCLCPP_WARN(logger_, "ActionSequence: trigger ignored because a sequence is already active");
    return false;
  }

  phases_ = phases_template_;
  active_ = true;
  current_phase_index_ = 0;
  pending_controller_timeline_sync_ = true;
  phase_start_wall_sec_ = 0.0;
  phase_start_event_ = true;
  finished_event_ = false;
  return true;
}

void ActionSequenceController::step(double wall_sec) {
  if (!active_ || phases_.empty()) return;

  if (pending_controller_timeline_sync_)
  {
    phase_start_wall_sec_ = wall_sec;
    pending_controller_timeline_sync_ = false;
  }

  const double current_time = std::max(wall_sec, phase_start_wall_sec_);
  if (current_phase_index_ < phases_.size())
  {
    const ActionSequencePhase& phase = phases_[current_phase_index_];
    if (phase.mode != ActionSequenceMode::kRlImitation &&
        (current_time - phase_start_wall_sec_) + kTimelineTimeEps >= phase.duration)
    {
      ++current_phase_index_;
      if (current_phase_index_ >= phases_.size())
      {
        active_ = false;
        finished_event_ = true;
        return;
      }

      phase_start_wall_sec_ = current_time;
      pending_controller_timeline_sync_ = false;
      phase_start_event_ = true;
    }
  }
}

void ActionSequenceController::stop() {
  active_ = false;
  current_phase_index_ = 0;
  phase_start_wall_sec_ = 0.0;
  pending_controller_timeline_sync_ = false;
  phase_start_event_ = false;
  finished_event_ = false;
}

const ActionSequencePhase* ActionSequenceController::activePhase() const {
  if (!active_ || current_phase_index_ >= phases_.size()) return nullptr;
  return &phases_[current_phase_index_];
}

bool ActionSequenceController::consumePhaseStartEvent(ActionSequencePhase& out) {
  if (!phase_start_event_ || current_phase_index_ >= phases_.size()) return false;
  phase_start_event_ = false;
  out = phases_[current_phase_index_];
  return true;
}

bool ActionSequenceController::consumeFinishedEvent() {
  const bool finished = finished_event_;
  finished_event_ = false;
  return finished;
}

double ActionSequenceController::remainingPhaseSeconds(double wall_sec) const {
  if (!active_ || current_phase_index_ >= phases_.size()) return 0.0;
  if (phases_[current_phase_index_].mode == ActionSequenceMode::kRlImitation)
  {
    return 1e9;
  }
  if (pending_controller_timeline_sync_)
  {
    return phases_[current_phase_index_].duration;
  }
  const double t_end = phase_start_wall_sec_ + phases_[current_phase_index_].duration;
  return std::max(0.0, t_end - wall_sec);
}

double ActionSequenceController::elapsedPhaseSeconds(double wall_sec) const {
  if (!active_ || current_phase_index_ >= phases_.size()) return 0.0;
  if (phases_[current_phase_index_].mode == ActionSequenceMode::kRlImitation) return 0.0;
  if (pending_controller_timeline_sync_) return 0.0;
  const ActionSequencePhase& p = phases_[current_phase_index_];
  const double elapsed = wall_sec - phase_start_wall_sec_;
  const double d = std::max(p.duration, 0.0);
  return std::max(0.0, std::min(d, elapsed));
}

void ActionSequenceController::addBlendToCurrentPhaseTimeline(double blend_sec) {
  if (!active_ || phases_.empty() || current_phase_index_ >= phases_.size()) return;
  if (phases_[current_phase_index_].mode == ActionSequenceMode::kRlImitation) return;
  if (blend_sec <= 1e-9) return;
  phases_[current_phase_index_].duration += blend_sec;
}

double ActionSequenceController::activePhaseBaseVelRampSec() const { return 0.0; }

void ActionSequenceController::completeActivePhaseByExternalEvent(double wall_sec) {
  if (!active_ || phases_.empty() || current_phase_index_ >= phases_.size()) return;
  if (phases_[current_phase_index_].mode != ActionSequenceMode::kRlImitation)
  {
    RCLCPP_WARN(logger_,
                "ActionSequence: completeActivePhaseByExternalEvent ignored (active phase is not rl_imitation)");
    return;
  }

  phase_start_wall_sec_ = std::max(wall_sec, phase_start_wall_sec_);
  pending_controller_timeline_sync_ = false;
  ++current_phase_index_;
  if (current_phase_index_ >= phases_.size())
  {
    active_ = false;
    finished_event_ = true;
    return;
  }

  phase_start_wall_sec_ = std::max(wall_sec, phase_start_wall_sec_);
  phase_start_event_ = true;
}

bool ActionSequenceController::parsePhase(const YAML::Node& node, std::size_t index, ActionSequencePhase& out) {
  if (!node || !node.IsMap())
  {
    RCLCPP_ERROR(logger_, "ActionSequence: phase %zu must be a map", index);
    return false;
  }

  out.name = node["name"] ? node["name"].as<std::string>() : ("phase_" + std::to_string(index));

  const std::string mode = node["mode"] ? node["mode"].as<std::string>() : "";
  if (mode == "rl_imitation")
  {
    if (node["velocity_timeline"])
    {
      RCLCPP_ERROR(logger_, "ActionSequence: phase '%s' (rl_imitation) does not support velocity_timeline",
                   out.name.c_str());
      return false;
    }
    out.mode = ActionSequenceMode::kRlImitation;
    out.csv_path.clear();
    out.duration = 0.0;
    out.duration_explicit = false;
    const YAML::Node velocity = node["velocity"];
    out.base_velocity.linear.x = velocity && velocity["vx"] ? velocity["vx"].as<double>(0.0) : 0.0;
    out.base_velocity.linear.y = velocity && velocity["vy"] ? velocity["vy"].as<double>(0.0) : 0.0;
    out.base_velocity.angular.z = velocity && velocity["wz"] ? velocity["wz"].as<double>(0.0) : 0.0;
    return true;
  }

  out.duration_explicit = true;
  bool auto_duration = false;
  if (!node["duration"])
  {
    auto_duration = true;
  }
  else
  {
    const YAML::Node dur_node = node["duration"];
    if (!dur_node.IsScalar())
    {
      RCLCPP_ERROR(logger_, "ActionSequence: phase '%s' duration must be a number or 'auto'", out.name.c_str());
      return false;
    }
    const std::string dur_str = dur_node.as<std::string>();
    if (dur_str == "auto" || dur_str == "csv")
    {
      auto_duration = true;
    }
    else
    {
      try
      { out.duration = std::stod(dur_str); } catch (...)
      {
        RCLCPP_ERROR(logger_, "ActionSequence: phase '%s' has invalid duration", out.name.c_str());
        return false;
      }
      if (out.duration <= 0.0)
      {
        RCLCPP_ERROR(logger_, "ActionSequence: phase '%s' must have duration > 0 (or omit / use auto)",
                     out.name.c_str());
        return false;
      }
    }
  }

  if (mode == "rl_with_csv")
  {
    out.mode = ActionSequenceMode::kRlWithCsv;
  }
  else if (mode == "full_csv")
  {
    out.mode = ActionSequenceMode::kFullCsv;
  }
  else
  {
    RCLCPP_ERROR(logger_, "ActionSequence: phase '%s' has invalid mode '%s'", out.name.c_str(), mode.c_str());
    return false;
  }

  const std::string raw_csv_path = node["csv"] ? node["csv"].as<std::string>() : "";
  if (raw_csv_path.empty())
  {
    RCLCPP_ERROR(logger_, "ActionSequence: phase '%s' is missing csv", out.name.c_str());
    return false;
  }
  out.csv_path = resolvePath(raw_csv_path, sequence_dir_);
  if (!pathExists(out.csv_path))
  {
    RCLCPP_ERROR(logger_, "ActionSequence: phase '%s' csv does not exist: %s", out.name.c_str(), out.csv_path.c_str());
    return false;
  }

  const YAML::Node velocity = node["velocity"];
  out.base_velocity.linear.x = velocity && velocity["vx"] ? velocity["vx"].as<double>(0.0) : 0.0;
  out.base_velocity.linear.y = velocity && velocity["vy"] ? velocity["vy"].as<double>(0.0) : 0.0;
  out.base_velocity.angular.z = velocity && velocity["wz"] ? velocity["wz"].as<double>(0.0) : 0.0;

  out.base_vel_ramp_sec = node["base_vel_ramp_sec"] ? std::max(node["base_vel_ramp_sec"].as<double>(0.0), 0.0) : -1.0;

  if (auto_duration)
  {
    if (!jointCsvPlaybackDurationFromFile(out.csv_path, out.duration, logger_)) return false;
    out.duration_explicit = false;
  }

  if (out.mode == ActionSequenceMode::kFullCsv)
  {
    if (node["velocity_timeline"])
    {
      RCLCPP_ERROR(logger_, "ActionSequence: phase '%s' (full_csv) does not support velocity_timeline",
                   out.name.c_str());
      return false;
    }
  }
  else if (out.mode == ActionSequenceMode::kRlWithCsv)
  {
    const bool has_velocity_block = static_cast<bool>(node["velocity"]);
    if (!parseVelocityTimeline(node["velocity_timeline"], out.name, out.duration, has_velocity_block, logger_,
                               out.velocity_timeline))
    {
      return false;
    }
  }

  return true;
}

std::string ActionSequenceController::resolvePath(const std::string& raw_path, const std::string& base_dir) const {
  if (raw_path.empty()) return raw_path;
  if (isAbsolutePath(raw_path)) return raw_path;

  const std::string relative_to_base = joinPath(base_dir, raw_path);
  if (pathExists(relative_to_base)) return relative_to_base;

  const std::string relative_to_share = joinPath(package_share_dir_, raw_path);
  if (pathExists(relative_to_share)) return relative_to_share;

  return relative_to_base;
}

bool ActionSequenceController::pathExists(const std::string& path) const {
  std::ifstream stream(path);
  return stream.good();
}

std::string ActionSequenceController::parentDirectory(const std::string& path) {
  const std::string::size_type pos = path.find_last_of('/');
  if (pos == std::string::npos) return ".";
  if (pos == 0) return "/";
  return path.substr(0, pos);
}

}  // namespace legged
