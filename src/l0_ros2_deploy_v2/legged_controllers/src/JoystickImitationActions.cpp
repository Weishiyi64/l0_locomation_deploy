#include "legged_controllers/JoystickImitationActions.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <rclcpp/logging.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "legged_controllers/ControllerFileUtils.h"

namespace legged {

namespace {

std::string normalizeName(std::string value) {
  value = trimCopy(value);
  std::string out;
  out.reserve(value.size());
  for (char ch : value)
  {
    if (ch == '_' || ch == '-' || ch == ' ') continue;
    out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
  }
  return out;
}

bool fileExists(const std::string& path) {
  std::ifstream f(path);
  return f.good();
}

bool modelAssetExists(const std::string& policy_dir, const std::string& model_name) {
  return fileExists(joinPath(policy_dir, model_name + ".trt")) ||
         fileExists(joinPath(policy_dir, model_name + ".onnx"));
}

int buttonIndexFromName(const std::string& raw_name) {
  const std::string name = normalizeName(raw_name);
  if (name == "a") return 0;
  if (name == "b") return 1;
  if (name == "x") return 3;
  if (name == "y") return 4;
  if (name == "lb" || name == "l1") return 6;
  if (name == "rb" || name == "r1") return 7;
  if (name == "lt" || name == "ltbutton" || name == "l2" || name == "l2button") return 8;
  if (name == "rt" || name == "rtbutton" || name == "r2" || name == "r2button") return 9;
  if (name == "back" || name == "select") return 10;
  if (name == "start") return 11;
  if (name == "ls" || name == "lsbutton" || name == "l3") return 13;
  if (name == "rs" || name == "rsbutton" || name == "r3") return 14;
  return -1;
}

int dpadAxisIndexFromName(const std::string& raw_name) {
  const std::string name = normalizeName(raw_name);
  if (name == "dpadlr" || name == "dpadx" || name == "dirlr" || name == "hatx") return 6;
  if (name == "dpadud" || name == "dpady" || name == "dirud" || name == "haty") return 7;
  return -1;
}

int axisSignFromDirection(const std::string& raw_direction, int axis_index) {
  const std::string direction = normalizeName(raw_direction);
  if (direction == "positive" || direction == "pos" || direction == "+") return 1;
  if (direction == "negative" || direction == "neg" || direction == "-") return -1;
  if (direction == "up") return axis_index == 7 ? 1 : 0;
  if (direction == "down") return axis_index == 7 ? -1 : 0;
  if (direction == "left") return axis_index == 6 ? 1 : 0;
  if (direction == "right") return axis_index == 6 ? -1 : 0;
  return 0;
}

std::string dpadAxisNameForDirection(const std::string& raw_direction) {
  const std::string direction = normalizeName(raw_direction);
  if (direction == "up" || direction == "down") return "DPad_ud";
  if (direction == "left" || direction == "right") return "DPad_lr";
  return "";
}

std::vector<std::string> asStringVector(const YAML::Node& node) {
  std::vector<std::string> out;
  if (!node) return out;
  if (node.IsScalar())
  {
    out.push_back(trimCopy(node.as<std::string>("")));
    return out;
  }
  if (!node.IsSequence())
  {
    throw std::runtime_error("expected scalar or sequence");
  }
  for (const auto& item : node)
  {
    if (!item.IsScalar())
    {
      throw std::runtime_error("sequence entries must be scalars");
    }
    out.push_back(trimCopy(item.as<std::string>("")));
  }
  return out;
}

std::string comboDescription(const std::vector<JoystickImitationActions::ButtonRequirement>& buttons,
                             const std::vector<JoystickImitationActions::AxisRequirement>& axes) {
  std::vector<std::string> parts;
  for (const auto& button : buttons)
  {
    parts.push_back(button.name);
  }
  for (const auto& axis : axes)
  {
    std::string direction;
    if (axis.index == 7)
    {
      direction = axis.sign > 0 ? "Up" : "Down";
    }
    else if (axis.index == 6)
    {
      direction = axis.sign > 0 ? "Left" : "Right";
    }
    else
    {
      direction = axis.sign > 0 ? "Positive" : "Negative";
    }
    parts.push_back(axis.name + ":" + direction);
  }

  std::ostringstream stream;
  for (std::size_t i = 0; i < parts.size(); ++i)
  {
    if (i > 0) stream << "+";
    stream << parts[i];
  }
  return stream.str();
}

std::string canonicalCombo(std::vector<JoystickImitationActions::ButtonRequirement> buttons,
                           std::vector<JoystickImitationActions::AxisRequirement> axes) {
  std::vector<std::string> parts;
  std::sort(buttons.begin(), buttons.end(), [](const auto& lhs, const auto& rhs) { return lhs.index < rhs.index; });
  std::sort(axes.begin(), axes.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.index != rhs.index) return lhs.index < rhs.index;
    return lhs.sign < rhs.sign;
  });
  for (const auto& button : buttons)
  {
    parts.push_back("b" + std::to_string(button.index));
  }
  for (const auto& axis : axes)
  {
    parts.push_back("a" + std::to_string(axis.index) + (axis.sign > 0 ? "+" : "-"));
  }
  std::ostringstream stream;
  for (std::size_t i = 0; i < parts.size(); ++i)
  {
    if (i > 0) stream << "|";
    stream << parts[i];
  }
  return stream.str();
}

bool validatePolicyAssets(const std::string& policy, const JoystickImitationActions::LoadOptions& options,
                          rclcpp::Logger logger) {
  if (policy.empty())
  {
    RCLCPP_WARN(logger, "JoystickImitation: enabled action has empty policy");
    return false;
  }
  if (options.policy_root_path.empty() || options.robot_type.empty())
  {
    RCLCPP_WARN(logger, "JoystickImitation: cannot validate policy '%s' without policy_root_path and robot_type",
                policy.c_str());
    return false;
  }

  const std::string policy_dir = joinPath(joinPath(options.policy_root_path, options.robot_type), policy);
  const std::string cfg_path = joinPath(policy_dir, "rl_config.yaml");
  if (!fileExists(cfg_path))
  {
    RCLCPP_WARN(logger, "JoystickImitation: policy '%s' is missing rl_config.yaml: %s", policy.c_str(),
                cfg_path.c_str());
    return false;
  }

  try
  {
    const YAML::Node root = YAML::LoadFile(cfg_path);
    const YAML::Node rc = root["RobotCfg"];
    if (!rc)
    {
      RCLCPP_WARN(logger, "JoystickImitation: policy '%s' config is missing RobotCfg: %s", policy.c_str(),
                  cfg_path.c_str());
      return false;
    }
    const std::string policy_type = trimCopy(rc["policy_type"].as<std::string>(""));
    if (!policy_type.empty() && policy_type != "rl_imitation")
    {
      RCLCPP_WARN(logger, "JoystickImitation: policy '%s' RobotCfg.policy_type must be rl_imitation, got '%s'",
                  policy.c_str(), policy_type.c_str());
      return false;
    }

    const std::string policy_model = trimCopy(rc["policy_name"].as<std::string>(""));
    const std::string motion_model = trimCopy(rc["motion_name"].as<std::string>(""));
    if (policy_model.empty() || motion_model.empty())
    {
      RCLCPP_WARN(logger, "JoystickImitation: policy '%s' needs non-empty policy_name and motion_name", policy.c_str());
      return false;
    }
    if (!modelAssetExists(policy_dir, policy_model))
    {
      RCLCPP_WARN(logger, "JoystickImitation: policy '%s' missing policy model asset %s.(trt|onnx)", policy.c_str(),
                  joinPath(policy_dir, policy_model).c_str());
      return false;
    }
    if (!modelAssetExists(policy_dir, motion_model))
    {
      RCLCPP_WARN(logger, "JoystickImitation: policy '%s' missing motion model asset %s.(trt|onnx)", policy.c_str(),
                  joinPath(policy_dir, motion_model).c_str());
      return false;
    }
  } catch (const std::exception& e)
  {
    RCLCPP_WARN(logger, "JoystickImitation: failed to validate policy '%s': %s", policy.c_str(), e.what());
    return false;
  }

  return true;
}

}  // namespace

bool JoystickImitationActions::load(const std::string& path, const LoadOptions& options, rclcpp::Logger logger) {
  bindings_.clear();
  last_combo_states_.clear();
  skipped_binding_count_ = 0;

  const std::string resolved_path = resolvePackagePath(trimCopy(path), options.package_share_dir);
  if (resolved_path.empty() || !fileExists(resolved_path))
  {
    RCLCPP_ERROR(logger, "JoystickImitation: config path is missing or not a file: %s", path.c_str());
    return false;
  }

  YAML::Node root;
  try
  { root = YAML::LoadFile(resolved_path); } catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "JoystickImitation: failed to load config '%s': %s", resolved_path.c_str(), e.what());
    return false;
  }

  const YAML::Node cfg = root["joystick_imitation_actions"] ? root["joystick_imitation_actions"] : root;
  if (!cfg || !cfg.IsMap())
  {
    RCLCPP_ERROR(logger, "JoystickImitation: '%s' must contain a YAML map", resolved_path.c_str());
    return false;
  }

  if (cfg["axis_threshold"])
  {
    axis_threshold_ = std::max(0.05, cfg["axis_threshold"].as<double>(0.5));
  }

  const YAML::Node actions = cfg["actions"];
  if (!actions || !actions.IsSequence())
  {
    RCLCPP_ERROR(logger, "JoystickImitation: '%s' must contain joystick_imitation_actions.actions",
                 resolved_path.c_str());
    return false;
  }

  std::set<std::string> names;
  std::set<std::string> combos;
  std::set<std::string> policies;

  for (const auto& action : actions)
  {
    if (!action || !action.IsMap())
    {
      RCLCPP_ERROR(logger, "JoystickImitation: every actions[] entry must be a map");
      return false;
    }

    const bool enabled = action["enabled"] ? action["enabled"].as<bool>(false) : false;
    std::string name = action["name"] ? trimCopy(action["name"].as<std::string>("")) : "";
    if (name.empty())
    {
      RCLCPP_ERROR(logger, "JoystickImitation: each action needs a non-empty name");
      return false;
    }
    if (!names.insert(name).second)
    {
      RCLCPP_ERROR(logger, "JoystickImitation: duplicate action name '%s'", name.c_str());
      return false;
    }

    const YAML::Node combo = action["combo"];
    if (!combo || !combo.IsMap())
    {
      RCLCPP_ERROR(logger, "JoystickImitation: action '%s' needs combo map", name.c_str());
      return false;
    }

    std::vector<ButtonRequirement> buttons;
    std::vector<AxisRequirement> axes;
    try
    {
      for (const std::string& button_name : asStringVector(combo["buttons"]))
      {
        const int index = buttonIndexFromName(button_name);
        if (index < 0)
        {
          RCLCPP_ERROR(logger, "JoystickImitation: action '%s' has unknown button '%s'", name.c_str(),
                       button_name.c_str());
          return false;
        }
        ButtonRequirement button;
        button.index = index;
        button.name = button_name;
        buttons.push_back(std::move(button));
      }
    } catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger, "JoystickImitation: action '%s' invalid combo.buttons: %s", name.c_str(), e.what());
      return false;
    }

    if (combo["dpad"])
    {
      const std::string direction = trimCopy(combo["dpad"].as<std::string>(""));
      const std::string axis_name = dpadAxisNameForDirection(direction);
      const int axis_index = dpadAxisIndexFromName(axis_name);
      const int sign = axisSignFromDirection(direction, axis_index);
      if (axis_index < 0 || sign == 0)
      {
        RCLCPP_ERROR(logger, "JoystickImitation: action '%s' has invalid combo.dpad '%s'", name.c_str(),
                     direction.c_str());
        return false;
      }
      AxisRequirement axis;
      axis.index = axis_index;
      axis.sign = sign;
      axis.name = axis_name;
      axes.push_back(std::move(axis));
    }

    if (combo["axes"])
    {
      if (!combo["axes"].IsSequence())
      {
        RCLCPP_ERROR(logger, "JoystickImitation: action '%s' combo.axes must be a sequence", name.c_str());
        return false;
      }
      for (const auto& axis_node : combo["axes"])
      {
        if (!axis_node || !axis_node.IsMap())
        {
          RCLCPP_ERROR(logger, "JoystickImitation: action '%s' combo.axes entries must be maps", name.c_str());
          return false;
        }
        const YAML::Node axis_name_node = axis_node["name"] ? axis_node["name"] : axis_node["axis"];
        const std::string axis_name = axis_name_node ? trimCopy(axis_name_node.as<std::string>("")) : "";
        const int axis_index = dpadAxisIndexFromName(axis_name);
        if (axis_index < 0)
        {
          RCLCPP_ERROR(logger, "JoystickImitation: action '%s' has unknown D-pad axis '%s'", name.c_str(),
                       axis_name.c_str());
          return false;
        }
        const std::string direction =
            axis_node["direction"] ? trimCopy(axis_node["direction"].as<std::string>("")) : "";
        const int sign = axisSignFromDirection(direction, axis_index);
        if (sign == 0)
        {
          RCLCPP_ERROR(logger, "JoystickImitation: action '%s' axis '%s' has invalid direction '%s'", name.c_str(),
                       axis_name.c_str(), direction.c_str());
          return false;
        }
        AxisRequirement axis;
        axis.index = axis_index;
        axis.sign = sign;
        axis.name = axis_name;
        axes.push_back(std::move(axis));
      }
    }

    if (buttons.empty() && axes.empty())
    {
      RCLCPP_ERROR(logger, "JoystickImitation: action '%s' combo has no button or axis requirements", name.c_str());
      return false;
    }

    const std::string canonical = canonicalCombo(buttons, axes);
    if (!combos.insert(canonical).second)
    {
      RCLCPP_ERROR(logger, "JoystickImitation: duplicate combo for action '%s' (%s)", name.c_str(),
                   comboDescription(buttons, axes).c_str());
      return false;
    }

    if (!enabled)
    {
      continue;
    }

    const std::string policy = action["policy"] ? trimCopy(action["policy"].as<std::string>("")) : "";
    if (policy.empty())
    {
      RCLCPP_ERROR(logger, "JoystickImitation: enabled action '%s' needs non-empty policy", name.c_str());
      return false;
    }
    if (options.validate_policy_assets && !policies.count(policy) && !validatePolicyAssets(policy, options, logger))
    {
      ++skipped_binding_count_;
      RCLCPP_WARN(logger, "JoystickImitation: skipping action '%s' because policy '%s' is unavailable", name.c_str(),
                  policy.c_str());
      continue;
    }
    policies.insert(policy);

    BindingState state;
    state.binding.name = name;
    state.binding.label = action["label"] ? trimCopy(action["label"].as<std::string>("")) : "";
    state.binding.policy = policy;
    state.binding.combo_description = comboDescription(buttons, axes);
    state.binding.shot_id = action["shot_id"] ? action["shot_id"].as<std::int64_t>(-1) : -1;
    state.buttons = std::move(buttons);
    state.axes = std::move(axes);
    state.canonical_combo = canonical;
    bindings_.push_back(std::move(state));
  }

  return true;
}

const JoystickImitationBinding* JoystickImitationActions::consumeTriggered(const sensor_msgs::msg::Joy& joy) {
  const JoystickImitationBinding* triggered = nullptr;

  for (auto& binding : bindings_)
  {
    const bool current = comboActive(binding, joy);
    const bool previous = last_combo_states_[binding.canonical_combo];
    last_combo_states_[binding.canonical_combo] = current;
    if (current && !previous && triggered == nullptr)
    {
      triggered = &binding.binding;
    }
  }

  return triggered;
}

bool JoystickImitationActions::comboActive(const BindingState& binding, const sensor_msgs::msg::Joy& joy) const {
  for (const auto& button : binding.buttons)
  {
    if (button.index < 0 || static_cast<std::size_t>(button.index) >= joy.buttons.size() ||
        joy.buttons[button.index] != 1)
    {
      return false;
    }
  }
  for (const auto& axis : binding.axes)
  {
    if (axis.index < 0 || static_cast<std::size_t>(axis.index) >= joy.axes.size())
    {
      return false;
    }
    const double value = joy.axes[axis.index];
    if (axis.sign > 0)
    {
      if (value < axis_threshold_) return false;
    }
    else
    {
      if (value > -axis_threshold_) return false;
    }
  }
  return true;
}

}  // namespace legged
