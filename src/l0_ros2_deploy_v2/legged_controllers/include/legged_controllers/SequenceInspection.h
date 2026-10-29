#pragma once

#include <rclcpp/logger.hpp>
#include <string>

namespace legged {

bool inspectSequenceImitation(const std::string& path, rclcpp::Logger logger, const char* context, bool& uses,
                              bool& requires_walking_policy, std::string& imitation_policy);

}  // namespace legged
