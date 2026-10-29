#include "legged_controllers/JoystickSequenceActions.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cctype>
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

std::string parentDirectory(const std::string& path) {
  const std::string::size_type pos = path.find_last_of('/');
  if (pos == std::string::npos) return ".";
  if (pos == 0) return "/";
  return path.substr(0, pos);
}

std::string resolveSequenceLocalPath(const std::string& raw_path, const std::string& sequence_dir,
                                     const std::string& package_share_dir) {
  if (raw_path.empty() || isAbsolutePath(raw_path)) return raw_path;

  const std::string relative_to_sequence = joinPath(sequence_dir, raw_path);
  if (pathExists(relative_to_sequence)) return relative_to_sequence;

  const std::string relative_to_share = joinPath(package_share_dir, raw_path);
  if (pathExists(relative_to_share)) return relative_to_share;

  return relative_to_sequence;
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

std::string comboDescription(const std::vector<JoystickSequenceActions::ButtonRequirement>& buttons,
                             const std::vector<JoystickSequenceActions::AxisRequirement>& axes) {
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

std::string canonicalCombo(std::vector<JoystickSequenceActions::ButtonRequirement> buttons,
                           std::vector<JoystickSequenceActions::AxisRequirement> axes) {
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

bool buttonRequired(const std::vector<JoystickSequenceActions::ButtonRequirement>& buttons, int index) {
  return std::find_if(buttons.begin(), buttons.end(), [index](const auto& button) { return button.index == index; }) !=
         buttons.end();
}

bool joyButtonPressed(const sensor_msgs::msg::Joy& joy, int index) {
  return index >= 0 && static_cast<std::size_t>(index) < joy.buttons.size() && joy.buttons[index] == 1;
}

bool validateRlWithCsvSequence(const std::string& raw_sequence_path,
                               const JoystickSequenceActions::LoadOptions& options, rclcpp::Logger logger,
                               const std::string& action_name, std::string& resolved_sequence_path) {
  resolved_sequence_path = resolvePackagePath(trimCopy(raw_sequence_path), options.package_share_dir);
  if (resolved_sequence_path.empty() || !pathExists(resolved_sequence_path))
  {
    RCLCPP_ERROR(logger, "JoystickSequence: action '%s' sequence does not exist: %s", action_name.c_str(),
                 resolved_sequence_path.c_str());
    return false;
  }

  YAML::Node root;
  try
  { root = YAML::LoadFile(resolved_sequence_path); } catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "JoystickSequence: failed to inspect action '%s' sequence '%s': %s", action_name.c_str(),
                 resolved_sequence_path.c_str(), e.what());
    return false;
  }

  const YAML::Node sequence = root["sequence"];
  const YAML::Node phases = sequence ? sequence["phases"] : YAML::Node();
  if (!sequence || !phases || !phases.IsSequence() || phases.size() == 0)
  {
    RCLCPP_ERROR(logger, "JoystickSequence: action '%s' sequence must contain non-empty sequence.phases",
                 action_name.c_str());
    return false;
  }

  const std::string sequence_dir = parentDirectory(resolved_sequence_path);
  for (std::size_t i = 0; i < phases.size(); ++i)
  {
    const YAML::Node phase = phases[i];
    if (!phase || !phase.IsMap())
    {
      RCLCPP_ERROR(logger, "JoystickSequence: action '%s' phase[%zu] must be a map", action_name.c_str(), i);
      return false;
    }
    const std::string phase_name = phase["name"] ? trimCopy(phase["name"].as<std::string>("")) : "";
    const std::string phase_label = phase_name.empty() ? ("phase_" + std::to_string(i)) : phase_name;
    const std::string mode = phase["mode"] ? trimCopy(phase["mode"].as<std::string>("")) : "";
    if (mode != "rl_with_csv")
    {
      RCLCPP_ERROR(logger, "JoystickSequence: action '%s' phase '%s' must use mode=rl_with_csv, got '%s'",
                   action_name.c_str(), phase_label.c_str(), mode.c_str());
      return false;
    }

    const std::string csv_path = phase["csv"] ? trimCopy(phase["csv"].as<std::string>("")) : "";
    const std::string resolved_csv_path = resolveSequenceLocalPath(csv_path, sequence_dir, options.package_share_dir);
    if (csv_path.empty() || !pathExists(resolved_csv_path))
    {
      RCLCPP_ERROR(logger, "JoystickSequence: action '%s' phase '%s' csv does not exist: %s", action_name.c_str(),
                   phase_label.c_str(), resolved_csv_path.c_str());
      return false;
    }
  }

  return true;
}

}  // namespace

bool JoystickSequenceActions::load(const std::string& path, const LoadOptions& options, rclcpp::Logger logger) {
  bindings_.clear();
  last_combo_states_.clear();

  const std::string resolved_path = resolvePackagePath(trimCopy(path), options.package_share_dir);
  if (resolved_path.empty() || !pathExists(resolved_path))
  {
    RCLCPP_ERROR(logger, "JoystickSequence: config path is missing or not a file: %s", path.c_str());
    return false;
  }

  YAML::Node root;
  try
  { root = YAML::LoadFile(resolved_path); } catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "JoystickSequence: failed to load config '%s': %s", resolved_path.c_str(), e.what());
    return false;
  }

  const YAML::Node cfg = root["joystick_sequence_actions"] ? root["joystick_sequence_actions"] : root;
  if (!cfg || !cfg.IsMap())
  {
    RCLCPP_ERROR(logger, "JoystickSequence: '%s' must contain a YAML map", resolved_path.c_str());
    return false;
  }

  if (cfg["axis_threshold"])
  {
    axis_threshold_ = std::max(0.05, cfg["axis_threshold"].as<double>(0.5));
  }

  const YAML::Node actions = cfg["actions"];
  if (!actions || !actions.IsSequence())
  {
    RCLCPP_ERROR(logger, "JoystickSequence: '%s' must contain joystick_sequence_actions.actions",
                 resolved_path.c_str());
    return false;
  }

  std::set<std::string> names;
  std::set<std::string> combos;

  for (const auto& action : actions)
  {
    if (!action || !action.IsMap())
    {
      RCLCPP_ERROR(logger, "JoystickSequence: every actions[] entry must be a map");
      return false;
    }

    const bool enabled = action["enabled"] ? action["enabled"].as<bool>(false) : false;
    std::string name = action["name"] ? trimCopy(action["name"].as<std::string>("")) : "";
    if (name.empty())
    {
      RCLCPP_ERROR(logger, "JoystickSequence: each action needs a non-empty name");
      return false;
    }
    if (!names.insert(name).second)
    {
      RCLCPP_ERROR(logger, "JoystickSequence: duplicate action name '%s'", name.c_str());
      return false;
    }

    const YAML::Node combo = action["combo"];
    if (!combo || !combo.IsMap())
    {
      RCLCPP_ERROR(logger, "JoystickSequence: action '%s' needs combo map", name.c_str());
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
          RCLCPP_ERROR(logger, "JoystickSequence: action '%s' has unknown button '%s'", name.c_str(),
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
      RCLCPP_ERROR(logger, "JoystickSequence: action '%s' invalid combo.buttons: %s", name.c_str(), e.what());
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
        RCLCPP_ERROR(logger, "JoystickSequence: action '%s' has invalid combo.dpad '%s'", name.c_str(),
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
        RCLCPP_ERROR(logger, "JoystickSequence: action '%s' combo.axes must be a sequence", name.c_str());
        return false;
      }
      for (const auto& axis_node : combo["axes"])
      {
        if (!axis_node || !axis_node.IsMap())
        {
          RCLCPP_ERROR(logger, "JoystickSequence: action '%s' combo.axes entries must be maps", name.c_str());
          return false;
        }
        const YAML::Node axis_name_node = axis_node["name"] ? axis_node["name"] : axis_node["axis"];
        const std::string axis_name = axis_name_node ? trimCopy(axis_name_node.as<std::string>("")) : "";
        const int axis_index = dpadAxisIndexFromName(axis_name);
        if (axis_index < 0)
        {
          RCLCPP_ERROR(logger, "JoystickSequence: action '%s' has unknown D-pad axis '%s'", name.c_str(),
                       axis_name.c_str());
          return false;
        }
        const std::string direction =
            axis_node["direction"] ? trimCopy(axis_node["direction"].as<std::string>("")) : "";
        const int sign = axisSignFromDirection(direction, axis_index);
        if (sign == 0)
        {
          RCLCPP_ERROR(logger, "JoystickSequence: action '%s' axis '%s' has invalid direction '%s'", name.c_str(),
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
      RCLCPP_ERROR(logger, "JoystickSequence: action '%s' combo has no button or axis requirements", name.c_str());
      return false;
    }

    const std::string canonical = canonicalCombo(buttons, axes);
    if (!combos.insert(canonical).second)
    {
      RCLCPP_ERROR(logger, "JoystickSequence: duplicate combo for action '%s' (%s)", name.c_str(),
                   comboDescription(buttons, axes).c_str());
      return false;
    }

    if (!enabled)
    {
      continue;
    }

    std::string sequence_path = action["sequence"] ? trimCopy(action["sequence"].as<std::string>("")) : "";
    if (sequence_path.empty() && action["sequence_path"])
    {
      sequence_path = trimCopy(action["sequence_path"].as<std::string>(""));
    }
    if (sequence_path.empty())
    {
      RCLCPP_ERROR(logger, "JoystickSequence: enabled action '%s' needs non-empty sequence", name.c_str());
      return false;
    }

    std::string resolved_sequence_path;
    if (options.validate_sequence_files &&
        !validateRlWithCsvSequence(sequence_path, options, logger, name, resolved_sequence_path))
    {
      return false;
    }
    if (resolved_sequence_path.empty())
    {
      resolved_sequence_path = resolvePackagePath(sequence_path, options.package_share_dir);
    }

    BindingState state;
    state.binding.name = name;
    state.binding.label = action["label"] ? trimCopy(action["label"].as<std::string>("")) : "";
    state.binding.sequence_path = resolved_sequence_path;
    state.binding.combo_description = comboDescription(buttons, axes);
    state.buttons = std::move(buttons);
    state.axes = std::move(axes);
    state.canonical_combo = canonical;
    bindings_.push_back(std::move(state));
  }

  return true;
}

const JoystickSequenceBinding* JoystickSequenceActions::consumeTriggered(const sensor_msgs::msg::Joy& joy) {
  const JoystickSequenceBinding* triggered = nullptr;

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

bool JoystickSequenceActions::comboActive(const BindingState& binding, const sensor_msgs::msg::Joy& joy) const {
  for (const auto& button : binding.buttons)
  {
    if (!joyButtonPressed(joy, button.index))
    {
      return false;
    }
  }

  // ABXY actions are fixed one-button triggers; ignore combos with extra face buttons to avoid
  // accidental overlap with existing B+Y walk and A+X calibration chords.
  static constexpr std::array<int, 4> kFaceButtons = {{0, 1, 3, 4}};
  for (const int index : kFaceButtons)
  {
    if (joyButtonPressed(joy, index) && !buttonRequired(binding.buttons, index))
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
