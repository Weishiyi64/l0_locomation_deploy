#pragma once

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "robot_interfaces/msg/policy_command.hpp"

// Convert published position targets, never measured joint positions, to the receiving actor's coordinates.
template <typename Defaults, typename Scales>
bool actionHistoryFromCommand(const robot_interfaces::msg::PolicyCommand& command,
                              const std::vector<std::string>& joint_names, const Defaults& defaults,
                              const Scales& scales, const rclcpp::Time& now,
                              std::vector<double>& actions, std::string& error) {
  if (command.header.stamp.sec < 0 || command.valid_until.sec < 0 ||
      command.header.stamp.nanosec >= 1000000000U || command.valid_until.nanosec >= 1000000000U)
  {
    error = "history seed contains an invalid timestamp";
    return false;
  }
  const rclcpp::Time stamp(command.header.stamp, now.get_clock_type());
  const rclcpp::Time deadline(command.valid_until, now.get_clock_type());
  if (stamp.nanoseconds() <= 0 || stamp > now || deadline <= now || deadline <= stamp ||
      joint_names.empty() || joint_names.size() > 64 || command.joint_names.size() != joint_names.size() ||
      command.target_positions.size() != joint_names.size() || defaults.size() != joint_names.size() ||
      scales.size() != joint_names.size())
  {
    error = "history seed has expired or has invalid joint arrays";
    return false;
  }
  std::unordered_map<std::string, double> targets;
  targets.reserve(command.joint_names.size());
  for (std::size_t i = 0; i < command.joint_names.size(); ++i)
  {
    if (command.joint_names[i].empty() || !std::isfinite(command.target_positions[i]) ||
        !targets.emplace(command.joint_names[i], command.target_positions[i]).second)
    {
      error = "history seed contains duplicate joints or invalid targets";
      return false;
    }
  }
  actions.resize(joint_names.size());
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    const auto target = targets.find(joint_names[i]);
    if (target == targets.end() || !std::isfinite(defaults[i]) || !std::isfinite(scales[i]))
    {
      error = "history seed joint mapping or receiving action parameters are invalid";
      return false;
    }
    actions[i] = scales[i] == 0.0 ? 0.0 : (target->second - defaults[i]) / scales[i];
    if (!std::isfinite(actions[i]))
    {
      error = "history seed conversion produced a non-finite action";
      return false;
    }
    targets.erase(target);
  }
  return true;
}
