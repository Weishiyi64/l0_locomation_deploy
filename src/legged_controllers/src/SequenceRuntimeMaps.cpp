#include <yaml-cpp/yaml.h>

#include <random>
#include <utility>

#include "legged_controllers/ControllerFileUtils.h"
#include "legged_controllers/SequenceInspection.h"
#include "legged_controllers/SequenceRuntime.h"

namespace legged {

namespace {

constexpr const char* kDefaultKeyboardActionAudioTopic = "/audio_play_cmd";

}  // namespace

bool SequenceRuntime::loadEmotionSequenceMap(const std::string& path) {
  const std::string resolved_map_path = resolvePackagePath(trimCopy(path), package_share_dir_);
  if (resolved_map_path.empty() || !pathExists(resolved_map_path))
  {
    RCLCPP_ERROR(logger(), "EmotionSequence: map path is missing or not a file: %s", path.c_str());
    return false;
  }

  YAML::Node root;
  try
  { root = YAML::LoadFile(resolved_map_path); } catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger(), "EmotionSequence: failed to load map '%s': %s", resolved_map_path.c_str(), e.what());
    return false;
  }

  const YAML::Node map_node = root["emotion_sequences"];
  if (!map_node || !map_node.IsMap())
  {
    RCLCPP_ERROR(logger(), "EmotionSequence: '%s' must contain an emotion_sequences map", resolved_map_path.c_str());
    return false;
  }

  std::unordered_map<std::string, std::vector<EmotionSequenceCandidate>> next_paths;
  for (const auto& entry : map_node)
  {
    if (!entry.first.IsScalar() || (!entry.second.IsScalar() && !entry.second.IsSequence()))
    {
      RCLCPP_ERROR(logger(), "EmotionSequence: map entries must be code: path or code: [path, ...]");
      return false;
    }
    const std::string code = toUpperCopy(trimCopy(entry.first.as<std::string>("")));
    if (code.empty())
    {
      RCLCPP_ERROR(logger(), "EmotionSequence: map entries must have non-empty code");
      return false;
    }

    std::vector<std::string> raw_sequence_paths;
    if (entry.second.IsScalar())
    {
      raw_sequence_paths.push_back(trimCopy(entry.second.as<std::string>("")));
    }
    else
    {
      for (const auto& candidate : entry.second)
      {
        if (!candidate.IsScalar())
        {
          RCLCPP_ERROR(logger(), "EmotionSequence: code '%s' candidate entries must be scalar paths", code.c_str());
          return false;
        }
        raw_sequence_paths.push_back(trimCopy(candidate.as<std::string>("")));
      }
    }

    if (raw_sequence_paths.empty())
    {
      RCLCPP_ERROR(logger(), "EmotionSequence: code '%s' has no candidate sequence paths", code.c_str());
      return false;
    }

    std::vector<EmotionSequenceCandidate> resolved_sequence_paths;
    resolved_sequence_paths.reserve(raw_sequence_paths.size());
    for (const std::string& raw_sequence_path : raw_sequence_paths)
    {
      if (raw_sequence_path.empty())
      {
        RCLCPP_ERROR(logger(), "EmotionSequence: code '%s' has an empty candidate sequence path", code.c_str());
        return false;
      }

      const std::string resolved_sequence_path = resolvePackagePath(raw_sequence_path, package_share_dir_);
      if (!pathExists(resolved_sequence_path))
      {
        RCLCPP_ERROR(logger(), "EmotionSequence: code '%s' sequence file does not exist: %s", code.c_str(),
                     resolved_sequence_path.c_str());
        return false;
      }

      bool uses_rl_imitation = false;
      bool requires_walking_policy = false;
      std::string imitation_policy;
      if (!inspectSequenceImitation(resolved_sequence_path, logger(), "EmotionSequence", uses_rl_imitation,
                                    requires_walking_policy, imitation_policy))
      {
        return false;
      }
      EmotionSequenceCandidate candidate;
      candidate.sequence_path = resolved_sequence_path;
      resolved_sequence_paths.push_back(std::move(candidate));
    }

    next_paths[code] = resolved_sequence_paths;
  }

  if (next_paths.empty())
  {
    RCLCPP_ERROR(logger(), "EmotionSequence: '%s' has no usable entries", resolved_map_path.c_str());
    return false;
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  emotion_sequence_paths_ = std::move(next_paths);
  last_emotion_sequence_indices_.clear();
  return true;
}

bool SequenceRuntime::loadKeyboardActionMap(const std::string& path) {
  const std::string resolved_map_path = resolvePackagePath(trimCopy(path), package_share_dir_);
  if (resolved_map_path.empty() || !pathExists(resolved_map_path))
  {
    RCLCPP_ERROR(logger(), "KeyboardAction: map path is missing or not a file: %s", path.c_str());
    return false;
  }

  YAML::Node root;
  try
  { root = YAML::LoadFile(resolved_map_path); } catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger(), "KeyboardAction: failed to load map '%s': %s", resolved_map_path.c_str(), e.what());
    return false;
  }

  YAML::Node cfg = root["keyboard_actions"] ? root["keyboard_actions"] : root;
  const YAML::Node actions = cfg["actions"];
  if (!actions || !actions.IsSequence())
  {
    RCLCPP_ERROR(logger(), "KeyboardAction: '%s' must contain keyboard_actions.actions", resolved_map_path.c_str());
    return false;
  }

  std::string audio_topic =
      cfg["audio_topic"] ? trimCopy(cfg["audio_topic"].as<std::string>("")) : kDefaultKeyboardActionAudioTopic;
  if (audio_topic.empty()) audio_topic = kDefaultKeyboardActionAudioTopic;

  std::unordered_map<std::string, KeyboardActionBinding> bindings;
  for (const auto& action : actions)
  {
    if (!action || !action.IsMap())
    {
      RCLCPP_ERROR(logger(), "KeyboardAction: every actions[] entry must be a map");
      return false;
    }

    KeyboardActionBinding binding;
    binding.name = action["name"] ? trimCopy(action["name"].as<std::string>("")) : "";
    binding.key = action["key"] ? trimCopy(action["key"].as<std::string>("")) : "";
    binding.sequence_path = action["sequence"] ? trimCopy(action["sequence"].as<std::string>("")) : "";
    if (binding.sequence_path.empty() && action["sequence_path"])
    {
      binding.sequence_path = trimCopy(action["sequence_path"].as<std::string>(""));
    }
    binding.audio_cmd = action["audio_cmd"] ? trimCopy(action["audio_cmd"].as<std::string>("")) : "";

    if (binding.name.empty()) binding.name = binding.key;
    if (binding.name.empty() || binding.key.empty() || binding.sequence_path.empty())
    {
      RCLCPP_ERROR(logger(),
                   "KeyboardAction: each action needs non-empty name/key/sequence (name may be omitted if key exists)");
      return false;
    }
    if (bindings.find(binding.name) != bindings.end())
    {
      RCLCPP_ERROR(logger(), "KeyboardAction: duplicate action name '%s'", binding.name.c_str());
      return false;
    }

    const std::string resolved_sequence_path = resolvePackagePath(binding.sequence_path, package_share_dir_);
    if (!pathExists(resolved_sequence_path))
    {
      RCLCPP_ERROR(logger(), "KeyboardAction: action '%s' sequence does not exist: %s", binding.name.c_str(),
                   resolved_sequence_path.c_str());
      return false;
    }

    bool uses_rl_imitation = false;
    bool requires_walking_policy = false;
    std::string imitation_policy;
    if (!inspectSequenceImitation(resolved_sequence_path, logger(), "KeyboardAction", uses_rl_imitation,
                                  requires_walking_policy, imitation_policy))
    {
      return false;
    }
    if (uses_rl_imitation)
    {
      binding.uses_rl_imitation = true;
      binding.imitation_policy = imitation_policy;
    }
    binding.requires_walking_policy = requires_walking_policy;
    binding.sequence_path = resolved_sequence_path;
    bindings[binding.name] = std::move(binding);
  }

  if (bindings.empty())
  {
    RCLCPP_ERROR(logger(), "KeyboardAction: '%s' has no usable actions", resolved_map_path.c_str());
    return false;
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  keyboard_action_config_path_ = resolved_map_path;
  keyboard_action_audio_topic_ = audio_topic;
  keyboard_action_bindings_ = std::move(bindings);
  RCLCPP_INFO(logger(), "KeyboardAction: loaded %zu actions from %s (audio_topic=%s)", keyboard_action_bindings_.size(),
              keyboard_action_config_path_.c_str(), keyboard_action_audio_topic_.c_str());
  return true;
}

const KeyboardActionBinding* SequenceRuntime::findKeyboardAction(const std::string& action_name) const {
  auto it = keyboard_action_bindings_.find(action_name);
  if (it == keyboard_action_bindings_.end()) return nullptr;
  return &it->second;
}

bool SequenceRuntime::hasEmotionCode(const std::string& code) const {
  return emotion_sequence_paths_.find(code) != emotion_sequence_paths_.end();
}

bool SequenceRuntime::selectEmotionSequence(const std::string& code, std::string& selected_sequence_path) {
  selected_sequence_path.clear();
  const auto it = emotion_sequence_paths_.find(code);
  if (it == emotion_sequence_paths_.end())
  {
    RCLCPP_WARN(logger(), "[EmotionSequence] No sequence mapping for emotion code '%s'", code.c_str());
    return false;
  }

  const std::vector<EmotionSequenceCandidate>& candidates = it->second;
  if (candidates.empty())
  {
    RCLCPP_WARN(logger(), "[EmotionSequence] Emotion code '%s' has no candidate sequences", code.c_str());
    return false;
  }

  std::vector<std::size_t> selectable_indices;
  selectable_indices.reserve(candidates.size());
  const auto last_it = last_emotion_sequence_indices_.find(code);
  const bool skip_last = candidates.size() > 1 && last_it != last_emotion_sequence_indices_.end();
  for (std::size_t index = 0; index < candidates.size(); ++index)
  {
    if (skip_last && index == last_it->second) continue;
    selectable_indices.push_back(index);
  }
  if (selectable_indices.empty())
  {
    selectable_indices.reserve(candidates.size());
    for (std::size_t index = 0; index < candidates.size(); ++index)
    {
      selectable_indices.push_back(index);
    }
  }

  std::size_t selected_index = selectable_indices.front();
  if (selectable_indices.size() > 1)
  {
    std::uniform_int_distribution<std::size_t> dist(0, selectable_indices.size() - 1);
    selected_index = selectable_indices[dist(emotion_sequence_rng_)];
  }

  selected_sequence_path = candidates[selected_index].sequence_path;
  last_emotion_sequence_indices_[code] = selected_index;
  return true;
}

void SequenceRuntime::markEmotionBusyIgnored(const std::string& code) {
  if (!emotion_busy_ignore_logged_)
  {
    RCLCPP_WARN(logger(),
                "[EmotionSequence] Ignoring emotion code '%s' because the controller is busy or not in a "
                "sequence-safe state",
                code.c_str());
    emotion_busy_ignore_logged_ = true;
  }
}

void SequenceRuntime::resetEmotionBusyIgnored() { emotion_busy_ignore_logged_ = false; }

}  // namespace legged
