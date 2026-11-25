#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "luvbot_control_interfaces/msg/policy_command.hpp"

namespace policy_handoff {

// Convert a fresh source target, never raw source actions, to destination joint order.
template <class Defaults, class Scales>
std::vector<double> initialActions(const luvbot_control_interfaces::msg::PolicyCommand& source,
                                   const std::string& expected_source, const rclcpp::Time& now,
                                   const std::vector<std::string>& joints, const Defaults& defaults,
                                   const Scales& scales) {
  const rclcpp::Time stamp(source.header.stamp, now.get_clock_type());
  const rclcpp::Time expiry(source.valid_until, now.get_clock_type());
  const double age = (now - stamp).seconds();
  if (source.source_id != expected_source || stamp.nanoseconds() <= 0 || age < 0.0 || age > 0.5 ||
      expiry <= now || (expiry - stamp).seconds() > 0.5 || joints.empty() || joints.size() > 64 ||
      source.joint_names.size() != joints.size() || source.target_positions.size() != joints.size() ||
      defaults.size() != joints.size() || scales.size() != joints.size())
    throw std::runtime_error("invalid/expired source command for policy history seed");
  std::unordered_map<std::string, double> targets;
  for (std::size_t i = 0; i < source.joint_names.size(); ++i) {
    if (source.joint_names[i].empty() || !std::isfinite(source.target_positions[i]) ||
        !targets.emplace(source.joint_names[i], source.target_positions[i]).second)
      throw std::runtime_error("invalid/duplicate source seed joint");
  }
  std::vector<double> actions;
  actions.reserve(joints.size());
  for (std::size_t j = 0; j < joints.size(); ++j) {
    const auto found = targets.find(joints[j]);
    if (found == targets.end() || !std::isfinite(defaults[j]) || !std::isfinite(scales[j]))
      throw std::runtime_error("seed joint mapping/default/scale mismatch");
    const double action = scales[j] == 0.0 ? 0.0 : (found->second - defaults[j]) / scales[j];
    if (!std::isfinite(action) || std::abs(action) > std::numeric_limits<float>::max())
      throw std::runtime_error("non-finite/overflowing seed action");
    actions.push_back(action);
    targets.erase(found);
  }
  return actions;
}

}  // namespace policy_handoff
